#!/usr/bin/env python3
"""A host that reads slower than the stream, against the SW emulator.

The full four-leg stream is 2500 frames/s, 20.5 MB/s.  A host that cannot keep
up -- a laptop running the bring-up GUI, a Python decoder -- must see what the
hardware gives it: frames lost (the frame counter, a timestamp, shows the gap),
command responses still answered, and memory that stays put.  2026-10-05 a
colleague's laptop showed the opposite: the emulator path climbed to 3 GB and
went quiet for 1-2 s at a time.  Two buffers on that path had no bound:

  * the emulator's outbound pipe buffer (pipe_server.cpp) took every frame the
    model produced, whatever the host read -- since the 2026-09-29 pacing fix
    the model no longer sheds load when its own loop runs late;
  * the FTD3XX shim's Mode 2 (IONM_FTD_ACCURATE=1) feeder appended every frame
    and each 1 KB read erased the front of the backlog -- a copy of the whole
    backlog per read, so a reader that fell behind slowed down further.

Checks (RESULTS:/STATUS: contract):
  * emulator: private memory stays bounded with a host reading 1000 frames/s;
  * emulator: the dropped frames show as frame-counter gaps, never repeats;
  * emulator: a register read is answered while the host is behind;
  * shim Mode 2 (Windows): an application reading 8 MB/s gets 8 MB/s, and the
    application's memory stays bounded.

    python fpga-test/emulator/test_slow_host.py [--exe PATH] [--shim PATH] [--seconds S]
"""
from __future__ import annotations

import argparse
import ctypes
import os
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import diff_emulators as dx                      # PipeClient / Host / register map

IS_WIN = os.name == "nt"
PIPE = "IONM_SLOW_HOST"
RESULTS: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    RESULTS.append((name, ok, detail))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f" -- {detail}" if detail else ""), flush=True)


# ---- process memory -----------------------------------------------------------
if IS_WIN:
    from ctypes import wintypes

    class _PMC(ctypes.Structure):
        _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD)] + [
            (n, ctypes.c_size_t) for n in (
                "PeakWorkingSetSize", "WorkingSetSize", "QuotaPeakPagedPoolUsage", "QuotaPagedPoolUsage",
                "QuotaPeakNonPagedPoolUsage", "QuotaNonPagedPoolUsage", "PagefileUsage",
                "PeakPagefileUsage", "PrivateUsage")]

    _k32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _k32.OpenProcess.restype = wintypes.HANDLE

    def private_mb(pid: int | None = None) -> float:
        h = wintypes.HANDLE(-1) if pid is None else _k32.OpenProcess(0x1000 | 0x0010, False, pid)
        m = _PMC()
        m.cb = ctypes.sizeof(m)
        _k32.K32GetProcessMemoryInfo(h, ctypes.byref(m), m.cb)
        if pid is not None:
            _k32.CloseHandle(h)
        return m.PrivateUsage / 2**20
else:
    def private_mb(pid: int | None = None) -> float:
        with open(f"/proc/{pid or 'self'}/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1]) / 1024
        return 0.0


def launch(exe: Path, dat: Path) -> subprocess.Popen:
    return subprocess.Popen([str(exe), "-f", str(dat), "-loop", "-pipe", PIPE],
                            cwd=str(exe.parent), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


# ---- the emulator with a slow host at the pipe -----------------------------------
def emulator_slow_host(exe: Path, dat: Path, seconds: float, rate: float) -> None:
    print(f"== emulator, host reading {rate:g} frames/s ({rate * dx.TELEM_WORDS * 2 / 2**20:.1f} MB/s) "
          f"against 2500 frames/s", flush=True)
    proc = launch(exe, dat)
    pipe = dx.PipeClient(PIPE)
    counters: list[int] = []

    def throttled_reader(self=pipe) -> None:
        nxt = time.time()
        while not self._stop.is_set():
            hdr = self._read_exact(2)
            if hdr is None:
                self.q.put(None)
                return
            (n,) = struct.unpack("<H", hdr)
            payload = self._read_exact(n) if n else b""
            if payload is None:
                self.q.put(None)
                return
            if n == dx.TELEM_WORDS * 2:
                w1, w2 = struct.unpack_from("<2H", payload, 2)
                counters.append(w1 | ((w2 & 0x1FFF) << 16))
                nxt += 1.0 / rate
                delay = nxt - time.time()
                if delay > 0:
                    time.sleep(delay)
            else:
                self.q.put(payload)                  # responses go to do_cmd

    pipe._reader = throttled_reader
    try:
        if not pipe.wait_for_pipe(30):
            check("emulator: endpoints appear", False, "emulator never created its pipes")
            return
        pipe.connect()
        host = dx.Host(pipe, 30.0, lambda s: None)
        host.bringup_and_run(1)
        t0 = time.time()
        time.sleep(seconds / 2)
        mid = private_mb(proc.pid)
        t_cmd = time.time()
        try:
            pipe.read_reg(dx.REG_ACQ_ALL_RUN, 5.0)
            answered, latency = True, time.time() - t_cmd
        except Exception as e:                       # noqa: BLE001
            answered, latency = False, float("nan")
        time.sleep(max(0.0, seconds - (time.time() - t0)))
        end = private_mb(proc.pid)
        steps = [(b - a) & 0x1FFFFFFF for a, b in zip(counters, counters[1:])]
        got_rate = len(counters) / (time.time() - t0)
        check("emulator: private memory bounded with a slow host", end < 64,
              f"{mid:.1f} MB at {seconds / 2:.0f} s, {end:.1f} MB at {seconds:.0f} s (limit 64)")
        check("emulator: host still gets its own rate", got_rate > 0.8 * rate,
              f"{got_rate:.0f} frames/s received")
        check("emulator: dropped frames show as counter gaps, never repeats",
              any(s > 1 for s in steps) and not any(s == 0 for s in steps),
              f"{sum(1 for s in steps if s > 1)} gaps, {sum(1 for s in steps if s == 0)} repeats "
              f"over {len(counters)} frames")
        check("emulator: register read answered while the host is behind", answered and latency < 2.0,
              f"{latency * 1000:.0f} ms" if answered else "no answer in 5 s")
    finally:
        try:
            pipe.close()
        except Exception:                            # noqa: BLE001
            pass
        proc.kill()
        proc.wait(timeout=10)


# ---- the shim's Mode 2 with a slow application -------------------------------------
def shim_mode2(exe: Path, dll: Path, dat: Path, seconds: float, mb_per_s: float) -> None:
    print(f"== FTD3XX shim Mode 2, application reading {mb_per_s:g} MB/s in 1 KB reads", flush=True)
    os.environ["IONM_FTD_ACCURATE"] = "1"
    os.environ["IONM_PIPE_NAME"] = PIPE
    proc = launch(exe, dat)
    from ctypes import wintypes
    ft = ctypes.WinDLL(str(dll))
    h = ctypes.c_void_p()
    n = wintypes.ULONG()
    buf = (ctypes.c_ubyte * 1024)()
    try:
        for _ in range(300):                         # wait for the emulator's endpoints
            if ft.FT_Create(None, 0, ctypes.byref(h)) == 0:
                break
            time.sleep(0.1)
        else:
            check("shim: FT_Create", False, "could not connect")
            return

        class ShimPipe:                              # register access over FT_WritePipe / FT_ReadPipe
            def do_cmd(self, flags, addr, data, timeout_s, stray=None):
                cmd = (ctypes.c_ubyte * 8).from_buffer_copy(struct.pack("<4H", dx.CMD_MAGIC, flags, addr, data))
                if ft.FT_WritePipe(h, 0x02, cmd, 8, ctypes.byref(n), None) != 0:
                    raise IOError("FT_WritePipe")
                acc, t0 = b"", time.time()
                while time.time() - t0 < timeout_s:
                    if ft.FT_ReadPipe(h, 0x82, buf, 1024, ctypes.byref(n), None) == 0 and n.value:
                        acc += bytes(buf[:n.value])
                        i = acc.find(struct.pack("<H", 0x55AA))
                        if i >= 0 and len(acc) >= i + 8:
                            return struct.unpack_from("<4H", acc, i)[3]
                raise TimeoutError(f"no response to 0x{addr:04X}")

            def write_reg(self, a, v, t, stray=None):
                self.do_cmd(dx.FLAG_WRITE, a, v, t)

            def read_reg(self, a, t, stray=None):
                return self.do_cmd(0, a, 0, t)

        dx.Host(ShimPipe(), 10.0, lambda s: None).bringup_and_run(1)
        start_mb = private_mb()
        limit = mb_per_s * 2**20
        total, t0 = 0, time.time()
        while time.time() - t0 < seconds:
            if total > limit * (time.time() - t0):
                time.sleep(0.001)
                continue
            if ft.FT_ReadPipe(h, 0x82, buf, 1024, ctypes.byref(n), None) == 0:
                total += n.value
        got = total / 2**20 / (time.time() - t0)
        grew = private_mb() - start_mb
        check("shim: application gets the rate it reads at", got > 0.9 * mb_per_s,
              f"{got:.1f} MB/s delivered (reading {mb_per_s:g})")
        check("shim: application memory bounded", grew < 32, f"grew {grew:.1f} MB in {seconds:.0f} s (limit 32)")
    finally:
        try:
            ft.FT_Close(h)
        except Exception:                            # noqa: BLE001
            pass
        proc.kill()
        proc.wait(timeout=10)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", default=str(dx.RELEASE / f"ionm_emulator{dx.EXE}"))
    ap.add_argument("--shim", default=str(dx.RELEASE / "FTD3XX.dll"))
    ap.add_argument("--seconds", type=float, default=12.0)
    a = ap.parse_args()
    exe = Path(a.exe)
    if not exe.exists():
        print(f"SKIP: {exe} not built")
        print("RESULTS: 0 passed, 0 failed")
        print("STATUS: SKIP")
        return 0
    dat = dx.RELEASE / "diff_emulators_dc.dat"
    dx.make_dc_file(dat)
    emulator_slow_host(exe, dat, a.seconds, 1000.0)
    if IS_WIN and Path(a.shim).exists():
        shim_mode2(exe, Path(a.shim), dat, a.seconds, 8.0)
    else:
        print("  (shim checks skipped: Windows only, needs FTD3XX.dll)")
    npass = sum(1 for _, ok, _ in RESULTS if ok)
    nfail = len(RESULTS) - npass
    print(f"RESULTS: {npass} passed, {nfail} failed")
    print("STATUS: " + ("PASS" if nfail == 0 else "FAIL"))
    return 0 if nfail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
