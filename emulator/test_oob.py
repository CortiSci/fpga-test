#!/usr/bin/env python3
"""Tests for the emulator's out-of-band playback endpoint (<base>_OOB).

The endpoint is one line in, one line out:

    client writes:    <path>\\n
    emulator replies: <open() error code>\\n        0 = success

and the emulator then streams that path as its sample source.  The path may be
a regular file or a FIFO / named pipe, which is the interesting half: a FIFO
cannot be mmap'd, its length is unknown until the writer closes, and a reader
that blocks on it naively would stall the emulator's 2.5 kHz tick loop.

The cases below are deliberately awkward -- empty paths, a path naming the
control pipe itself, NUL bytes, 8000-character paths, two requests in one
write, a half-line then a hang-up, a FIFO nobody ever writes to -- because that
is where a control channel breaks.

    python fpga-test/emulator/test_oob.py [--exe PATH] [-v]

Exit code 0 = all passed.
"""
from __future__ import annotations

import argparse
import os
import struct
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import diff_emulators as dx                      # PipeClient / Host / Frame / decode_normal

IS_WIN = os.name == "nt"
SENSORS = 4096
FRAME_BYTES = SENSORS * 2

if IS_WIN:
    import ctypes
    from ctypes import wintypes
    K32 = ctypes.WinDLL("kernel32", use_last_error=True)
    K32.CreateFileW.restype = wintypes.HANDLE
    K32.CreateNamedPipeW.restype = wintypes.HANDLE
    INVALID = wintypes.HANDLE(-1).value
    PIPE_ACCESS_OUTBOUND = 0x00000002
    PIPE_TYPE_BYTE, PIPE_WAIT = 0x0, 0x0


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def frame_bytes(value: int, n_words: int = SENSORS) -> bytes:
    return struct.pack(f"<{n_words}h", *([value] * n_words))


class Oob:
    """Client of the emulator's OOB endpoint."""

    def __init__(self, base: str):
        self.base = base
        self.f = None
        self.sk = None
        if IS_WIN:
            self.path = rf"\\.\pipe\{base}_OOB"
        else:
            self.path = os.path.join(dx.PIPE_DIR, f"{base}_OOB")

    def connect(self, timeout: float = 20.0):
        end = time.time() + timeout
        last = None
        while time.time() < end:
            try:
                if IS_WIN:
                    self.f = open(self.path, "r+b", buffering=0)
                else:
                    import socket
                    self.sk = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                    self.sk.connect(self.path)
                return self
            except OSError as e:
                last = e
                time.sleep(0.1)
        raise RuntimeError(f"cannot connect to {self.path}: {last}")

    def write_raw(self, data: bytes):
        if IS_WIN:
            self.f.write(data)
        else:
            self.sk.sendall(data)

    def read_line(self, timeout: float = 10.0) -> str:
        end = time.time() + timeout
        buf = b""
        while not buf.endswith(b"\n"):
            if time.time() > end:
                raise TimeoutError("no reply from OOB endpoint")
            if IS_WIN:
                b = self.f.read(1)
            else:
                self.sk.settimeout(max(0.05, end - time.time()))
                b = self.sk.recv(1)
            if not b:
                raise EOFError("OOB endpoint closed")
            buf += b
        return buf.decode(errors="replace").strip()

    def request(self, path: str, timeout: float = 10.0) -> int:
        self.write_raw(path.encode() + b"\n")
        return int(self.read_line(timeout))

    def close(self):
        try:
            if self.f:
                self.f.close()
            if self.sk:
                self.sk.close()
        except OSError:
            pass
        self.f = self.sk = None


class Emu:
    """A running emulator with a private endpoint base name."""

    _n = 0

    def __init__(self, exe: Path, extra: list[str] | None = None):
        Emu._n += 1
        self.base = f"OOBT{os.getpid() % 10000}_{Emu._n}"
        self.dir = Path(tempfile.mkdtemp(prefix="oobtest_"))
        self.log = self.dir / "emu.log"
        args = [str(exe), "-pipe", self.base, "-log", str(self.log)] + (extra or [])
        self.proc = subprocess.Popen(args, cwd=str(exe.parent),
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.pipe = None
        self.host = None

    def wait_ready(self, timeout: float = 20.0) -> bool:
        end = time.time() + timeout
        want = f"{self.base}_OOB"
        while time.time() < end:
            if self.proc.poll() is not None:
                return False
            try:
                if IS_WIN:
                    if want in os.listdir(r"\\.\pipe\\"):
                        return True
                elif os.path.exists(os.path.join(dx.PIPE_DIR, want)):
                    return True
            except OSError:
                pass
            time.sleep(0.05)
        return False

    def oob(self) -> Oob:
        return Oob(self.base).connect()

    def start_stream(self, cmd_timeout: float = 5.0):
        """Bring the ASIC stream up so played data is observable."""
        self.pipe = dx.PipeClient(self.base)
        self.pipe.wait_for_pipe(20)
        self.pipe.connect()
        self.host = dx.Host(self.pipe, cmd_timeout, lambda s: None)
        self.host.bringup_and_run(1)
        return self.host

    def frames(self, n: int, timeout: float = 5.0) -> list[dx.Frame]:
        out = []
        for _ in range(n):
            p = self.pipe.next_frame(timeout)
            if p is None:
                break
            out.append(dx.Frame.parse(p))
        return out

    def sensors_now(self, settle: int = 12, take: int = 3) -> list[list[int]]:
        """Decode `take` frames after skipping `settle`."""
        self.frames(settle, 5.0)
        return [dx.decode_normal(f.words, f.phases) for f in self.frames(take, 5.0)]

    def switch_to(self, oobc: "Oob", path: str) -> int:
        """Stop the stream, drain what is in flight, point the emulator at
        `path`, restart.  Decoding a frame in Python is far slower than the
        emulator's 2.5 kHz production rate, so without this a reader is always
        looking at a backlog of pre-switch frames."""
        self.host.stop()
        self.host.drain(0.4)
        rc = oobc.request(path)
        self.pipe.write_reg(dx.REG_ACQ_ALL_RUN, 0x000F, 5.0, [])
        return rc

    def wait_sensors(self, pred, max_frames: int = 400, timeout: float = 25.0, stride: int = 1):
        """Read frames until pred(sensors) holds.

        A source switch is not visible in the very next frame: the emulator runs
        at 2.5 kHz while this reader is far slower, so a backlog of pre-switch
        frames is in flight.  Waiting for the condition instead of skipping a
        guessed number of frames is what makes these checks deterministic.
        Returns (ok, last_sensors, frames_read)."""
        end = time.time() + timeout
        last: list[int] | None = None
        n = 0
        while n < max_frames and time.time() < end:
            p = self.pipe.next_frame(3.0)
            if p is None:
                break
            n += 1
            if n % stride:            # skip the decode, just keep up with the stream
                continue
            f = dx.Frame.parse(p)
            last = dx.decode_normal(f.words, f.phases)
            if pred(last):
                return True, last, n
        return False, last, n

    def alive(self) -> bool:
        return self.proc.poll() is None

    def log_text(self) -> str:
        try:
            return self.log.read_text(errors="replace")
        except OSError:
            return ""

    def stop(self):
        try:
            if self.host:
                self.host.stop()
            if self.pipe:
                self.pipe.close()
        except Exception:
            pass
        try:
            self.proc.terminate()
            self.proc.wait(timeout=10)
        except Exception:
            try:
                self.proc.kill()
            except Exception:
                pass


class FifoSource:
    """A FIFO (POSIX) / named pipe (Windows) the emulator can be told to play.
    Created up front so the emulator's open() succeeds before any data exists."""

    def __init__(self, tmp: Path, name: str):
        self.name = name
        if IS_WIN:
            self.path = rf"\\.\pipe\{name}"
            self.h = K32.CreateNamedPipeW(self.path, PIPE_ACCESS_OUTBOUND,
                                          PIPE_TYPE_BYTE | PIPE_WAIT, 1,
                                          1 << 20, 1 << 20, 0, None)
            if self.h == INVALID:
                raise OSError(f"CreateNamedPipe failed: {ctypes.get_last_error()}")
            self._connected = False
        else:
            self.path = str(tmp / name)
            os.mkfifo(self.path)
            self.fd = None

    def _ensure_connected(self):
        if IS_WIN:
            if not self._connected:
                K32.ConnectNamedPipe(self.h, None)   # returns once the emulator opened it
                self._connected = True
        elif self.fd is None:
            self.fd = os.open(self.path, os.O_WRONLY)

    def write(self, data: bytes):
        self._ensure_connected()
        if IS_WIN:
            n = wintypes.DWORD(0)
            if not K32.WriteFile(self.h, data, len(data), ctypes.byref(n), None):
                raise OSError(f"WriteFile failed: {ctypes.get_last_error()}")
        else:
            os.write(self.fd, data)

    def close(self):
        try:
            if IS_WIN:
                if self.h and self.h != INVALID:
                    K32.CloseHandle(self.h)
                    self.h = None
            else:
                if self.fd is not None:
                    os.close(self.fd)
                    self.fd = None
        except OSError:
            pass


# ---------------------------------------------------------------------------
# test registry
# ---------------------------------------------------------------------------
RESULTS: list[tuple[str, bool, str]] = []
VERBOSE = False


def check(name: str, ok: bool, detail: str = ""):
    RESULTS.append((name, bool(ok), detail))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  — {detail}" if detail else ""), flush=True)


def only(values: list[int]) -> set[int]:
    return set(values)


# ---------------------------------------------------------------------------
# Group 1 — the protocol: strange paths and strange writes
# ---------------------------------------------------------------------------
def group_protocol(exe: Path):
    print("\n[protocol] odd paths and odd writes on one connection")
    emu = Emu(exe)
    try:
        if not emu.wait_ready():
            check("emulator starts without -f", False, "endpoint never appeared")
            return
        check("emulator starts without -f", True, "OOB endpoint up")

        tmp = emu.dir
        good = tmp / "good.dat"
        good.write_bytes(frame_bytes(1234))

        c = emu.oob()

        rc = c.request(str(good))
        check("regular file -> 0", rc == 0, f"rc={rc}")

        rc = c.request(str(tmp / "no_such_file.dat"))
        check("missing file -> error", rc != 0, f"rc={rc}")

        rc = c.request(str(tmp))
        check("directory -> error", rc != 0, f"rc={rc}")

        rc = c.request("")
        check("empty path -> error", rc != 0, f"rc={rc}")

        rc = c.request("   ")
        check("blank path -> error", rc != 0, f"rc={rc}")

        # A Windows client sending CRLF must not have the CR treated as part of
        # the name.
        c.write_raw(str(good).encode() + b"\r\n")
        rc = int(c.read_line())
        check("trailing CR stripped", rc == 0, f"rc={rc}")

        # Relative path: resolved against the emulator's cwd, not the client's.
        rc = c.request("definitely_not_here_12345.dat")
        check("bogus relative path -> error", rc != 0, f"rc={rc}")

        long_path = str(tmp / ("x" * 8000))
        rc = c.request(long_path)
        check("8000-char path -> error, no wedge", rc != 0, f"rc={rc}")
        rc = c.request(str(good))
        check("connection usable after long path", rc == 0, f"rc={rc}")

        # Embedded NUL: the syscall would silently truncate the name.
        c.write_raw(str(good).encode() + b"\x00tail\n")
        rc = int(c.read_line())
        check("path with NUL byte -> error", rc != 0, f"rc={rc}")
        rc = c.request(str(good))
        check("stream in sync after NUL path", rc == 0, f"rc={rc}")

        # Two requests in a single write: two replies, in order.
        c.write_raw(str(good).encode() + b"\n" + str(tmp / "nope").encode() + b"\n")
        r1, r2 = int(c.read_line()), int(c.read_line())
        check("two paths in one write -> 2 replies in order", r1 == 0 and r2 != 0, f"rc={r1},{r2}")

        # A bare newline is a request for the empty path.
        c.write_raw(b"\n")
        rc = int(c.read_line())
        check("bare newline answered", rc != 0, f"rc={rc}")

        # Byte-at-a-time delivery must not confuse the line reader.
        for ch in (str(good) + "\n").encode():
            c.write_raw(bytes([ch]))
            time.sleep(0.001)
        rc = int(c.read_line())
        check("path delivered one byte at a time", rc == 0, f"rc={rc}")

        # Many back-to-back requests stay in lockstep.
        n_ok = 0
        for _ in range(50):
            if c.request(str(good)) == 0:
                n_ok += 1
        check("50 sequential requests answered", n_ok == 50, f"{n_ok}/50")

        # Pointing the emulator at its OWN control pipe: must not deadlock.
        target = c.path if IS_WIN else os.path.join(dx.PIPE_DIR, f"{emu.base}_FAULT_CTRL")
        try:
            rc = c.request(target, timeout=15)
            ok = True
        except (TimeoutError, EOFError, OSError) as e:
            rc, ok = -1, False
            check("play the control pipe itself -> answered", False, f"{type(e).__name__}")
        if ok:
            check("play the control pipe itself -> answered", True, f"rc={rc}")
            rc2 = c.request(str(good))
            check("still serving after self-reference", rc2 == 0, f"rc={rc2}")

        c.close()
        check("emulator alive after protocol abuse", emu.alive())
    finally:
        emu.stop()


# ---------------------------------------------------------------------------
# Group 2 — connection lifecycle
# ---------------------------------------------------------------------------
def group_connections(exe: Path):
    print("\n[connections] hang-ups, half lines, reconnects")
    emu = Emu(exe)
    try:
        if not emu.wait_ready():
            check("lifecycle emulator starts", False, "no endpoint")
            return
        good = emu.dir / "g.dat"
        good.write_bytes(frame_bytes(7))

        # Connect and drop without sending anything, repeatedly.
        for _ in range(10):
            Oob(emu.base).connect().close()
        c = emu.oob()
        check("serves after 10 silent connect/disconnects", c.request(str(good)) == 0)
        c.close()

        # A path with no trailing newline, then hang up: no reply is owed, and
        # the emulator must go back to listening rather than wedge.
        c = emu.oob()
        c.write_raw(str(good).encode())        # no "\n"
        c.close()
        time.sleep(0.3)
        c = emu.oob()
        check("serves after a truncated request", c.request(str(good)) == 0)
        c.close()

        # A second client while the first holds the connection.  One at a time
        # is the contract; what matters is that the first keeps working and the
        # second is served once the first leaves.
        c1 = emu.oob()
        check("first client works", c1.request(str(good)) == 0)
        second: dict = {}

        def open_second():
            try:
                c2 = Oob(emu.base).connect(timeout=25)
                second["rc"] = c2.request(str(good))
                c2.close()
            except Exception as e:                     # noqa: BLE001
                second["err"] = f"{type(e).__name__}: {e}"

        t = threading.Thread(target=open_second, daemon=True)
        t.start()
        time.sleep(0.5)
        check("first client still served with a second waiting",
              c1.request(str(good)) == 0)
        c1.close()
        t.join(timeout=25)
        check("second client served after the first left",
              second.get("rc") == 0, second.get("err", f"rc={second.get('rc')}"))

        check("emulator alive after lifecycle abuse", emu.alive())
    finally:
        emu.stop()


# ---------------------------------------------------------------------------
# Group 3 — playback of regular files
# ---------------------------------------------------------------------------
def group_playback(exe: Path):
    print("\n[playback] regular files reach the telemetry stream")
    emu = Emu(exe)
    try:
        if not emu.wait_ready():
            check("playback emulator starts", False, "no endpoint")
            return
        tmp = emu.dir
        two = tmp / "two.dat"
        two.write_bytes(frame_bytes(111) + frame_bytes(222))
        shortf = tmp / "short.dat"
        shortf.write_bytes(struct.pack("<100h", *([333] * 100)))
        emptyf = tmp / "empty.dat"
        emptyf.write_bytes(b"")
        oddf = tmp / "odd.dat"
        oddf.write_bytes(struct.pack("<10h", *([444] * 10)) + b"\x7f")

        emu.start_stream()
        c = emu.oob()

        # Before any OOB path the ASIC has no file: sensors read 0.
        pre = emu.sensors_now()
        check("no source -> zeros", all(only(s) == {0} for s in pre),
              f"values={sorted(only(pre[0]))[:4]}")

        check("play 2-frame file -> 0", emu.switch_to(c, str(two)) == 0)
        ok, s, n = emu.wait_sensors(lambda s: only(s) in ({111}, {222}), max_frames=120)
        check("2-frame file values appear", ok,
              f"after {n} frames values={sorted(only(s))[:4] if s else None}")

        check("play short file -> 0", emu.switch_to(c, str(shortf)) == 0)
        ok, s, n = emu.wait_sensors(lambda s: s.count(333) == 100, max_frames=120)
        check("short file zero-padded to 4096 words",
              ok and s.count(0) == SENSORS - 100,
              f"after {n} frames 333x{s.count(333) if s else '?'} 0x{s.count(0) if s else '?'}")

        check("play empty file -> 0", emu.switch_to(c, str(emptyf)) == 0)
        ok, s, n = emu.wait_sensors(lambda s: only(s) == {0}, max_frames=120)
        check("empty file -> one all-zero frame", ok, f"after {n} frames")

        check("play odd-length file -> 0", emu.switch_to(c, str(oddf)) == 0)
        ok, s, n = emu.wait_sensors(lambda s: s.count(444) == 10, max_frames=120)
        check("odd trailing byte tolerated", ok and emu.alive(),
              f"after {n} frames 444x{s.count(444) if s else '?'}")

        # A switch with the stream left RUNNING — no stop, no drain.  The reader
        # strides over frames so it can outrun the 2.5 kHz backlog.
        c.request(str(two))
        ok_a, _, na = emu.wait_sensors(lambda s: only(s) in ({111}, {222}),
                                       max_frames=4000, timeout=40.0, stride=25)
        c.request(str(shortf))
        ok_b, _, nb = emu.wait_sensors(lambda s: s.count(333) == 100,
                                       max_frames=4000, timeout=40.0, stride=25)
        check("source switch mid-stream takes effect", ok_a and ok_b,
              f"two.dat after {na} frames, short.dat after {nb}")

        # Re-playing the same path restarts it from the beginning.
        check("replay same path -> 0", c.request(str(two)) == 0)
        check("emulator alive after playback churn", emu.alive())
        c.close()
    finally:
        emu.stop()


# ---------------------------------------------------------------------------
# Group 4 — FIFO / named-pipe sources (the reason select() is needed)
# ---------------------------------------------------------------------------
def group_fifo(exe: Path):
    print("\n[fifo] a source that is a pipe, not a file")
    emu = Emu(exe)
    fifo = None
    try:
        if not emu.wait_ready():
            check("fifo emulator starts", False, "no endpoint")
            return
        emu.start_stream()
        c = emu.oob()

        # 1. A FIFO with NO writer: the open must succeed and the emulator must
        #    keep streaming and answering commands.  A naive blocking read here
        #    would freeze the model.
        fifo = FifoSource(emu.dir, f"{emu.base}_SRC")
        rc = c.request(fifo.path)
        check("open a writer-less FIFO -> 0", rc == 0, f"rc={rc}")

        frames = emu.frames(10, 5.0)
        check("telemetry keeps flowing with an idle FIFO", len(frames) == 10,
              f"{len(frames)}/10 frames")
        try:
            val = emu.pipe.read_reg(0x0140, 5.0, [])
            reg_ok = True
        except Exception as e:                          # noqa: BLE001
            val, reg_ok = None, False
        check("register reads still answered with an idle FIFO", reg_ok, f"ACQ_ALL_RUN={val}")
        got = emu.sensors_now(settle=5, take=2)
        check("idle FIFO plays zeros", all(only(s) == {0} for s in got),
              f"values={sorted(only(got[0]))[:4]}")

        # 2. Now write two frames into it: they must appear.
        w = threading.Thread(target=lambda: fifo.write(frame_bytes(555) + frame_bytes(666)),
                             daemon=True)
        w.start()
        w.join(timeout=10)
        ok, s, n = emu.wait_sensors(lambda s: only(s) in ({555}, {666}))
        check("FIFO data plays once written", ok,
              f"after {n} frames values={sorted(only(s))[:4] if s else None}")

        # 3. A partial frame then close: zero-padded, not discarded.
        fifo.write(struct.pack("<64h", *([777] * 64)))
        fifo.close()
        fifo = None
        ok, s, n = emu.wait_sensors(lambda s: s.count(777) == 64)
        check("FIFO closed mid-frame -> tail zero-padded",
              ok and s.count(0) == SENSORS - 64,
              f"after {n} frames 777x{s.count(777) if s else '?'}")

        check("emulator alive after FIFO play", emu.alive())
        c.close()
    finally:
        if fifo:
            fifo.close()
        emu.stop()


# ---------------------------------------------------------------------------
# Group 5 — shutdown while a source is live
# ---------------------------------------------------------------------------
def group_shutdown(exe: Path):
    print("\n[shutdown] stopping while a writer-less FIFO is open")
    emu = Emu(exe)
    fifo = None
    try:
        if not emu.wait_ready():
            check("shutdown emulator starts", False, "no endpoint")
            return
        fifo = FifoSource(emu.dir, f"{emu.base}_SRC2")
        c = emu.oob()
        check("open FIFO for shutdown test", c.request(fifo.path) == 0)
        c.close()
        t0 = time.time()
        emu.proc.terminate()
        try:
            emu.proc.wait(timeout=15)
            dt = time.time() - t0
            check("exits promptly with a blocked reader", dt < 15, f"{dt:.1f}s")
        except subprocess.TimeoutExpired:
            check("exits promptly with a blocked reader", False, "still running after 15 s")
    finally:
        if fifo:
            fifo.close()
        emu.stop()


# ---------------------------------------------------------------------------
def main() -> int:
    global VERBOSE
    ap = argparse.ArgumentParser()
    default_exe = (HERE / ".." / ".." / "Software Emulator" / "build" / "test_app" /
                   "Release" / ("ionm_emulator.exe" if IS_WIN else "ionm_emulator")).resolve()
    ap.add_argument("--exe", type=Path, default=default_exe)
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    VERBOSE = a.verbose

    if not a.exe.exists():
        print(f"SKIP: emulator not built at {a.exe}")
        print("RESULTS: 0 passed, 0 failed")
        print("STATUS: SKIP")
        return 0

    print(f"OOB endpoint tests — {a.exe}")
    for g in (group_protocol, group_connections, group_playback, group_fifo, group_shutdown):
        try:
            g(a.exe)
        except Exception as e:                          # noqa: BLE001
            check(f"{g.__name__} completed", False, f"{type(e).__name__}: {e}")

    npass = sum(1 for _, ok, _ in RESULTS if ok)
    nfail = len(RESULTS) - npass
    print()
    if nfail:
        print("failures:")
        for n, ok, d in RESULTS:
            if not ok:
                print(f"  - {n}  {d}")
    print(f"RESULTS: {npass} passed, {nfail} failed")
    print("STATUS: " + ("PASS" if nfail == 0 else "FAIL"))
    return 1 if nfail else 0


if __name__ == "__main__":
    sys.exit(main())
