#!/usr/bin/env python3
"""Differential test: SW behavioural emulator vs RTL (Verilator) emulator.

Stream the SAME sample file (and the SAME impedance file) through
ionm_emulator.exe (pure C++ model) and ionm_emu_rtl.exe (Verilator-compiled
consolidator_v2 + 4x tail_fpga_small), drive both with the SAME host command
sequence, decode what comes out of each with the SAME parser, and report where
they differ.  The purpose is to verify that the software emulator behaves like
the RTL — not to verify the RTL itself.

Why this is a host program and not an Icarus/Verilator bench
------------------------------------------------------------
The only interface the two emulators have in common is the named-pipe transport
(the same bytes the FTD3XX shim hands to host software).  The SW emulator is C++,
so neither simulator can host it; the RTL side is already a Verilator build.  So
the comparison is made at the pipe, by a host, exactly where application code
would see a difference.  Verilator is used only to BUILD the RTL side:

    consolidator_v2/verilator/build_rtl_emulator.sh      (MSYS2 UCRT64)

Both processes speak the same protocol, so no shim DLL is involved:

    CTRL pipe (host -> emulator):  [u16 LE byte_len][payload]   8-byte commands
    DATA pipe (emulator -> host):  [u16 LE byte_len][payload]   8-byte responses,
                                                                8210-byte frames
    command  = {0xAA55, flags(1=write), addr, data}   response = {0x55AA, flags, addr, rdata}
    words are native little-endian uint16 (usb_acq_pipeline.cpp raw_send casts the array).

Legs
----
  normal    TELEM_EN=0x01.  Each side must deliver the -f file.
  imp_even  TELEM_EN=0x02.  Each side must deliver the -if file on the even SD
  imp_odd   TELEM_EN=0x03.  lanes / odd lanes, in the tail's impedance packing
            (impedance_mode_spec.md: one word per pair of 5.12 MHz ticks,
            output[2k] = lane[k]@T0, output[2k+1] = lane[k]@T1; 512 words per
            sweep, two sweeps per 1024-word consolidator frame).

Both files are always given to both emulators, so a side that reads the wrong
file in a mode is caught (the SW ASIC BFM used to switch to -if whenever it was
merely OPEN; the RTL harness used to ignore -if entirely — both fixed 2026-09-09).

What is compared — three layers
-------------------------------
  structural  frame length 4105, tag 0x0001, counter words, phase-word shape,
              CRC-32 validity, counter monotonic, phase-reporting style.  Per
              emulator; a DIFF is when one side passes a check the other fails.
  recovered   the electrode image the host recovers from each side after that
              side's DOCUMENTED mapping (contract §7 transpose + de-rotation, then
              the SW emulator's SubQv3 electrode map).  Equal images = the host
              cannot tell the emulators apart.  This is the test.  In impedance
              legs the image covers the selected lanes only.
  vs file     each side's recovered image against the file it should be showing
              — tells the reader WHICH side to believe when they differ.

Known divergences (known_divergences.json beside this file) turn an EXPECTED
difference into a named row instead of a red verdict, the same way the sim suite
uses .github/workflows/baseline.json.  `--strict` ignores that list.

Output ends with the same two sentinel lines the SV benches emit:

    RESULTS: <n_pass> passed, <n_fail> failed
    STATUS: PASS|FAIL
"""
from __future__ import annotations

import argparse
import json
import os
import re
import struct
import subprocess
import sys
import threading
import time
import zlib
from dataclasses import dataclass, field
from pathlib import Path
from queue import Empty, Queue

HERE = Path(__file__).resolve().parent
TEST_ROOT = HERE.parent
# The design repo: the parent of the fpga-test mount when running as a submodule;
# overridable so the runner can point at any checkout of the design.
DUT_ROOT = Path(os.environ.get("IONM_DUT_ROOT", TEST_ROOT.parent))
RELEASE = DUT_ROOT / "Software Emulator" / "build" / "test_app" / "Release"
ELEC_MAP_H = DUT_ROOT / "Software Emulator" / "emulator" / "src" / "subqv3_elec_map.h"

# --- protocol constants (mirrors Software Emulator/test_app/src/ionm_test.cpp) ---
CMD_MAGIC, RESP_MAGIC, FLAG_WRITE = 0xAA55, 0x55AA, 0x0001
TELEM_WORDS, PHASE_IDX, CRC_IDX = 4105, 4099, 4103
REG_SPI_ENABLE_MASK = 0x0002
REG_SPI_CFG_DATA, REG_SPI_CFG_CTRL, REG_SPI_CFG_DATA2 = 0x0030, 0x0031, 0x0032
REG_SPI_CFG_RD01, REG_SPI_CFG_RD23, REG_SPI_CFG_RD45 = 0x0034, 0x0035, 0x0036
REG_SPI_CLK_DIV = 0x0060
REG_ACQ_ALL_RUN = 0x0140
SPI_GO, SPI_RW, N_BYTES_SHIFT = 0x0008, 0x0004, 5
TAIL_PING, TAIL_CTRL, TAIL_TELEM_EN = 0xAA, 0x01, 0x02
CTRL_RUN = 0x11            # RO_RSTn=1 + MCLK_EN=1
N_SENSORS = 4096

# --- file patterns: invertible, so a decoded value names its source frame ---
# sample:    value = file_frame * 4096 + sensor_index          (0x0000..0x3FFF)
# impedance: value = 0x4000 | file_frame * 4096 | sensor_index (0x4000..0x7FFF)
# Both positive int16; the top nibble tells the two files apart on sight.
N_FILE_FRAMES = 4
IMP_TAG = 0x4000
NOVAL = -1                 # placeholder for a sensor a leg does not carry


# =============================================================================
# Pipe client
# =============================================================================
class PipeClient:
    """Length-prefixed named-pipe client for either emulator.  A reader thread
    feeds complete DATA-pipe messages into a queue so reads can time out."""

    def __init__(self, base: str = "IONM_EMU"):
        self.ctrl_path = rf"\\.\pipe\{base}_CTRL"
        self.data_path = rf"\\.\pipe\{base}_DATA"
        self.ctrl = None
        self.data = None
        self.q: Queue = Queue()
        self._stop = threading.Event()
        self._thr: threading.Thread | None = None
        self.n_frames = 0
        self.n_short = 0          # messages that were neither 8 bytes nor a frame

    def wait_for_pipe(self, timeout_s: float) -> bool:
        base = self.ctrl_path.rsplit("\\", 1)[1]
        t_end = time.time() + timeout_s
        while time.time() < t_end:
            try:
                if base in os.listdir(r"\\.\pipe\\"):
                    return True
            except OSError:
                pass
            time.sleep(0.1)
        return False

    def connect(self) -> None:
        # Same order as the shim's FT_Create: CTRL first, then DATA.
        for _ in range(50):
            try:
                self.ctrl = open(self.ctrl_path, "wb", buffering=0)
                break
            except OSError:
                time.sleep(0.1)
        if self.ctrl is None:
            raise RuntimeError("could not open CTRL pipe")
        for _ in range(50):
            try:
                self.data = open(self.data_path, "rb", buffering=0)
                break
            except OSError:
                time.sleep(0.1)
        if self.data is None:
            raise RuntimeError("could not open DATA pipe")
        self._thr = threading.Thread(target=self._reader, daemon=True)
        self._thr.start()

    def close(self) -> None:
        self._stop.set()
        for h in (self.ctrl, self.data):
            try:
                if h:
                    h.close()
            except OSError:
                pass

    def _read_exact(self, n: int) -> bytes | None:
        buf = bytearray()
        while len(buf) < n:
            try:
                chunk = self.data.read(n - len(buf))
            except (OSError, ValueError):
                return None
            if not chunk:
                return None
            buf += chunk
        return bytes(buf)

    def _reader(self) -> None:
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
            if n == TELEM_WORDS * 2:
                self.n_frames += 1
            elif n != 8:
                self.n_short += 1
            self.q.put(payload)

    def send_cmd(self, flags: int, addr: int, data: int) -> None:
        payload = struct.pack("<4H", CMD_MAGIC, flags, addr, data)
        self.ctrl.write(struct.pack("<H", len(payload)) + payload)

    def do_cmd(self, flags: int, addr: int, data: int, timeout_s: float,
               stray: list | None = None) -> int:
        """Send a command and wait for its 4-word response.  Telemetry frames
        that arrive in between are kept (appended to `stray`) rather than lost."""
        self.send_cmd(flags, addr, data)
        t_end = time.time() + timeout_s
        while True:
            remaining = t_end - time.time()
            if remaining <= 0:
                raise TimeoutError(f"no response to addr=0x{addr:04X}")
            try:
                msg = self.q.get(timeout=remaining)
            except Empty:
                raise TimeoutError(f"no response to addr=0x{addr:04X}")
            if msg is None:
                raise RuntimeError("DATA pipe closed by emulator")
            if len(msg) == 8:
                w = struct.unpack("<4H", msg)
                if w[0] == RESP_MAGIC:
                    return w[3]
            elif len(msg) == TELEM_WORDS * 2 and stray is not None:
                stray.append(msg)

    def write_reg(self, addr: int, data: int, timeout_s: float, stray=None) -> None:
        self.do_cmd(FLAG_WRITE, addr, data, timeout_s, stray)

    def read_reg(self, addr: int, timeout_s: float, stray=None) -> int:
        return self.do_cmd(0, addr, 0, timeout_s, stray)

    def next_frame(self, timeout_s: float) -> bytes | None:
        t_end = time.time() + timeout_s
        while True:
            remaining = t_end - time.time()
            if remaining <= 0:
                return None
            try:
                msg = self.q.get(timeout=remaining)
            except Empty:
                return None
            if msg is None:
                raise RuntimeError("DATA pipe closed by emulator")
            if len(msg) == TELEM_WORDS * 2:
                return msg


# =============================================================================
# Host command sequence (identical for both emulators)
# =============================================================================
class Host:
    def __init__(self, pipe: PipeClient, cmd_timeout: float, log):
        self.p, self.t, self.log = pipe, cmd_timeout, log
        self.stray: list[bytes] = []

    def spi_xact(self, ch: int, n_bytes: int, tx: list[int], want_rx: bool) -> list[int]:
        # One SPI_CFG transaction under a single SS_N assertion (ionm_test.cpp:263).
        tx = (tx + [0] * 6)[:6]
        self.p.write_reg(REG_SPI_CFG_DATA, (tx[0] << 8) | tx[1], self.t, self.stray)
        if n_bytes > 2:
            self.p.write_reg(REG_SPI_CFG_DATA2, (tx[2] << 8) | tx[3], self.t, self.stray)
        ctrl = ((n_bytes & 7) << N_BYTES_SHIFT) | SPI_GO | (0 if want_rx else SPI_RW) | (ch & 3)
        self.p.write_reg(REG_SPI_CFG_CTRL, ctrl, self.t, self.stray)
        for _ in range(100000):
            if not (self.p.read_reg(REG_SPI_CFG_CTRL, self.t, self.stray) & SPI_GO):
                break
        if not want_rx:
            return []
        rd = [self.p.read_reg(r, self.t, self.stray) for r in (REG_SPI_CFG_RD01, REG_SPI_CFG_RD23, REG_SPI_CFG_RD45)]
        return [rd[0] >> 8, rd[0] & 0xFF, rd[1] >> 8, rd[1] & 0xFF, rd[2] >> 8, rd[2] & 0xFF]

    def tail_ping(self, ch: int) -> int:
        return self.spi_xact(ch, 3, [TAIL_PING, 0, 0], True)[2]

    def bringup_and_run(self, telem_mode: int) -> dict:
        """The sequence ionm_test.cpp uses to get frames flowing, minus its
        assertions.  telem_mode is the TELEM_EN data byte: 1 normal, 2 imp-even,
        3 imp-odd.  The consolidator watchdog is deliberately left disabled so a
        slow (RTL) run cannot trip an 819 ms fault frame mid-capture."""
        info = {"ping": []}
        # 800 kHz config SPI before any tail command (reset default 25.6 MHz is
        # faster than the tail's 20.48 MHz MCLK and every command is dropped).
        self.p.write_reg(REG_SPI_CLK_DIV, 31, self.t, self.stray)
        self.p.write_reg(REG_SPI_ENABLE_MASK, 0x000F, self.t, self.stray)
        for ch in range(4):
            info["ping"].append(self.tail_ping(ch))
            self.spi_xact(ch, 2, [TAIL_CTRL, CTRL_RUN], False)
            self.spi_xact(ch, 2, [TAIL_TELEM_EN, telem_mode], False)
        self.p.write_reg(REG_ACQ_ALL_RUN, 0x000F, self.t, self.stray)
        return info

    def stop(self) -> None:
        try:
            self.p.write_reg(REG_ACQ_ALL_RUN, 0x0000, self.t, self.stray)
        except Exception:
            pass


# =============================================================================
# Frame decode — the software contract (consolidator_v2/docs/telemetry_v3_software_contract.md)
# =============================================================================
def crc32_ieee_bigendian_words(words: list[int]) -> int:
    # CRC-32/IEEE 802.3 over words[0..4102], each word fed high byte first.
    b = bytearray()
    for w in words:
        b += bytes(((w >> 8) & 0xFF, w & 0xFF))
    return zlib.crc32(bytes(b)) & 0xFFFFFFFF


@dataclass
class Frame:
    words: list[int]
    frame_cnt: int
    tag_ok: bool
    cnt_hi_ok: bool
    phase_ok: bool
    crc_ok: bool
    phases: list[int]

    @staticmethod
    def parse(payload: bytes) -> "Frame":
        w = list(struct.unpack(f"<{TELEM_WORDS}H", payload))
        frame_cnt = w[1] | ((w[2] & 0x1FFF) << 16)
        phases = [w[PHASE_IDX + i] for i in range(4)]
        crc = (w[CRC_IDX] << 16) | w[CRC_IDX + 1]
        return Frame(words=w, frame_cnt=frame_cnt,
                     tag_ok=(w[0] == 0x0001),
                     cnt_hi_ok=((w[2] >> 13) == 0),
                     # phase word = {1'b0, par, ovfl, undf, 2'b00, phase[9:0]}: bits [11:10] zero
                     phase_ok=all((p & 0x0C00) == 0 for p in phases),
                     crc_ok=(crc == crc32_ieee_bigendian_words(w[:CRC_IDX])),
                     phases=phases)


def leg_words(w: list[int], ch: int, phase: int) -> list[int]:
    """The 1024 words of one leg in TRUE sweep order: de-interleave, then undo
    the group rotation the phase word reports (contract §4/§7)."""
    pv = phase & 0x3FF
    P = pv if pv <= 63 else 0
    out = [0] * 1024
    for group in range(64):
        gg = (group + P) & 63
        base = 3 + ch + 4 * 16 * group
        for k in range(16):
            out[16 * gg + k] = w[base + 4 * k]
    return out


def decode_normal(w: list[int], phases: list[int]) -> list[int]:
    """Normal mode: 16 consecutive bit-planes -> one 16-bit sample per lane
    (ionm_test reconstruct_asic_frame).  Returns sensors[ch*1024 + s*16 + lane]."""
    out = [0] * N_SENSORS
    for ch in range(4):
        lw = leg_words(w, ch, phases[ch])
        for s in range(64):
            planes = lw[16 * s:16 * s + 16]
            for lane in range(16):
                v = 0
                for k in range(16):
                    v |= ((planes[k] >> lane) & 1) << (15 - k)
                out[ch * 1024 + s * 16 + lane] = v
    return out


def decode_impedance(w: list[int], phases: list[int], sel: int) -> list[list[int]]:
    """Impedance mode (impedance_mode_spec.md): word j of a 512-word sweep carries
    sample s=j//8, tick pair p=j%8, for the 8 selected lanes 2k+sel:
        bit[2k]   = sample bit (15-2p)      (T0)
        bit[2k+1] = sample bit (15-2p-1)    (T1)
    A 1024-word leg holds TWO consecutive sweeps.  Returns [sweep0, sweep1], each a
    4096-array with the selected lanes filled and NOVAL elsewhere."""
    sweeps = [[NOVAL] * N_SENSORS for _ in range(2)]
    for ch in range(4):
        lw = leg_words(w, ch, phases[ch])
        for sw in range(2):
            vals = [0] * 16
            for s in range(64):
                for k in range(8):
                    vals[2 * k + sel] = 0
                for p in range(8):
                    word = lw[512 * sw + 8 * s + p]
                    for k in range(8):
                        vals[2 * k + sel] |= ((word >> (2 * k)) & 1) << (15 - 2 * p)
                        vals[2 * k + sel] |= ((word >> (2 * k + 1)) & 1) << (15 - 2 * p - 1)
                for k in range(8):
                    lane = 2 * k + sel
                    sweeps[sw][ch * 1024 + s * 16 + lane] = vals[lane]
    return sweeps


# =============================================================================
# SubQv3 electrode map (the SW emulator's documented remap) — parsed from the header
# =============================================================================
def load_elec_map(path: Path) -> list[list[int]] | None:
    if not path.exists():
        return None
    txt = path.read_text(encoding="utf-8", errors="replace")
    m = re.search(r"kSubQv3Cell\s*\[4\]\s*\[1024\]\s*=\s*\{(.*?)\n\};", txt, re.S)
    if not m:
        return None
    nums = [int(x) for x in re.findall(r"\d+", m.group(1))]
    if len(nums) != 4 * 1024:
        return None
    return [nums[i * 1024:(i + 1) * 1024] for i in range(4)]


def to_elec_image(sensors: list[int], cell: list[list[int]] | None) -> list[int]:
    """Sensor array -> 64x64 electrode image via the FORWARD SubQv3 map.  With no
    map (identity model) the image IS the sensor array.  NOVAL travels along."""
    if cell is None:
        return list(sensors)
    img = [NOVAL] * N_SENSORS
    for ch in range(4):
        row = cell[ch]
        base = ch * 1024
        for idx in range(1024):
            img[row[idx]] = sensors[base + idx]
    return img


# =============================================================================
# One emulator run
# =============================================================================
@dataclass
class RunResult:
    name: str
    exe: str
    telem_mode: int
    frames: list[Frame]
    ping: list[int]
    n_short: int
    launch_ok: bool
    error: str = ""
    seconds: float = 0.0


def run_emulator(name: str, exe: Path, args: list[str], telem_mode: int, n_frames: int,
                 first_frame_timeout: float, frame_timeout: float, cmd_timeout: float,
                 log) -> RunResult:
    res = RunResult(name, str(exe), telem_mode, [], [], 0, False)
    if not exe.exists():
        res.error = f"missing {exe}"
        return res
    t0 = time.time()
    kw = {}
    if os.name == "nt":
        kw["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW
    proc = subprocess.Popen([str(exe), *args], cwd=str(exe.parent),
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, **kw)
    pipe = PipeClient()
    try:
        if not pipe.wait_for_pipe(30):
            res.error = "emulator never created its pipes"
            return res
        pipe.connect()
        res.launch_ok = True
        host = Host(pipe, cmd_timeout, log)
        info = host.bringup_and_run(telem_mode)
        res.ping = info["ping"]
        log(f"  [{name}] ping={['0x%02X' % p for p in res.ping]}  TELEM_EN={telem_mode}  waiting for frames…")
        payloads: list[bytes] = list(host.stray)
        tmo = first_frame_timeout
        while len(payloads) < n_frames:
            m = pipe.next_frame(tmo)
            if m is None:
                break
            payloads.append(m)
            tmo = frame_timeout
        host.stop()
        res.frames = [Frame.parse(p) for p in payloads[:n_frames]]
        res.n_short = pipe.n_short
    except Exception as e:      # noqa: BLE001 — report, do not crash the other leg
        res.error = f"{type(e).__name__}: {e}"
    finally:
        pipe.close()
        try:
            proc.kill()
            proc.wait(timeout=10)
        except Exception:
            pass
        res.seconds = time.time() - t0
    return res


# =============================================================================
# Comparison
# =============================================================================
@dataclass
class Check:
    leg: str
    name: str
    ok: bool
    detail: str
    known: str = ""          # non-empty = matched a known divergence


def structural(r: RunResult) -> dict:
    fs = r.frames
    if not fs:
        return {"frames": 0}
    cnts = [f.frame_cnt for f in fs]
    mono = all(((cnts[i + 1] - cnts[i]) & 0x1FFFFFFF) == 1 for i in range(len(cnts) - 1))
    return {
        "frames": len(fs),
        "tag_ok": all(f.tag_ok for f in fs),
        "cnt_hi_ok": all(f.cnt_hi_ok for f in fs),
        "phase_ok": all(f.phase_ok for f in fs),
        "crc_ok": all(f.crc_ok for f in fs),
        "monotonic": mono,
        "frame_cnts": cnts,
        "phase_values": sorted({p & 0x3FF for f in fs for p in f.phases}),
        "phase_flags": sorted({(p >> 12) & 7 for f in fs for p in f.phases}),
    }


def file_frame_of(values: list[int], tag: int) -> tuple[int | None, float]:
    """Which source-file frame a decoded image came from (majority vote over the
    values that carry the expected tag nibble), and the agreeing fraction."""
    votes: dict[int, int] = {}
    n = 0
    for v in values:
        if v == NOVAL:
            continue
        n += 1
        if (v & 0xC000) == tag:
            ff = (v >> 12) & 0x3
            votes[ff] = votes.get(ff, 0) + 1
    if not votes or not n:
        return None, 0.0
    ff, cnt = max(votes.items(), key=lambda kv: kv[1])
    return ff, cnt / n


def expected_image(tag: int, ff: int) -> list[int]:
    return [tag | (ff << 12) | i for i in range(N_SENSORS)]


def compare_defined(a: list[int], b: list[int]) -> tuple[int, int, list[tuple[int, int, int]]]:
    """Compare where BOTH sides carry a value.  Returns (n_compared, n_diff, examples)."""
    n = 0
    diffs = []
    for i, (x, y) in enumerate(zip(a, b)):
        if x == NOVAL or y == NOVAL:
            continue
        n += 1
        if x != y:
            diffs.append((i, x, y))
    return n, len(diffs), diffs[:6]


def classify(a_img: list[int], b_img: list[int]) -> str:
    """Name the shape of a semantic difference so the report says WHAT diverged."""
    a = [v for v in a_img if v != NOVAL]
    b = [v for v in b_img if v != NOVAL]
    if len(set(b)) == 1:
        return f"rtl side is constant 0x{b[0]:04X}"
    if len(set(a)) == 1:
        return f"sw side is constant 0x{a[0]:04X}"
    ta = {(v & 0xC000) for v in a}
    tb = {(v & 0xC000) for v in b}
    if ta != tb:
        name = {0x0000: "-f (sample) file", IMP_TAG: "-if (impedance) file"}
        return (f"different FILES: sw shows {', '.join(name.get(t, hex(t)) for t in ta)}; "
                f"rtl shows {', '.join(name.get(t, hex(t)) for t in tb)}")
    if sorted(a) == sorted(b):
        return "same values, different positions (index/lane mapping)"
    return "different values"


def make_pattern_files(dat: Path, imp: Path) -> None:
    with dat.open("wb") as f:
        for ff in range(N_FILE_FRAMES):
            f.write(struct.pack(f"<{N_SENSORS}h", *[(ff << 12) | i for i in range(N_SENSORS)]))
    with imp.open("wb") as f:
        for ff in range(N_FILE_FRAMES):
            f.write(struct.pack(f"<{N_SENSORS}h", *[IMP_TAG | (ff << 12) | i for i in range(N_SENSORS)]))


# =============================================================================
# main
# =============================================================================
LEGS = {"normal": 1, "imp_even": 2, "imp_odd": 3}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--release-dir", default=str(RELEASE), help="dir holding ionm_emulator.exe + ionm_emu_rtl.exe")
    ap.add_argument("--frames", type=int, default=6, help="frames to capture per emulator per leg")
    ap.add_argument("--legs", default=",".join(LEGS), help="comma list of: " + ", ".join(LEGS))
    ap.add_argument("--sw-only", action="store_true"); ap.add_argument("--rtl-only", action="store_true")
    ap.add_argument("--first-frame-timeout", type=float, default=180.0, help="s to wait for the first frame")
    ap.add_argument("--frame-timeout", type=float, default=120.0)
    ap.add_argument("--cmd-timeout", type=float, default=60.0)
    ap.add_argument("--strict", action="store_true", help="ignore known_divergences.json — any difference fails")
    ap.add_argument("--out", default="", help="write a JSON report here")
    ap.add_argument("--keep-dat", action="store_true")
    a = ap.parse_args()

    def log(s: str) -> None:
        print(s, flush=True)

    rel = Path(a.release_dir)
    sw_exe, rtl_exe = rel / "ionm_emulator.exe", rel / "ionm_emu_rtl.exe"
    dat, imp = rel / "diff_emulators_pattern.dat", rel / "diff_emulators_imp.dat"
    make_pattern_files(dat, imp)
    cell = load_elec_map(ELEC_MAP_H)
    known = {} if a.strict else json.loads((HERE / "known_divergences.json").read_text(encoding="utf-8")).get("divergences", {})

    log(f"design root : {DUT_ROOT}")
    log(f"sw emulator : {sw_exe} ({'present' if sw_exe.exists() else 'MISSING'})")
    log(f"rtl emulator: {rtl_exe} ({'present' if rtl_exe.exists() else 'MISSING'})")
    log(f"elec map    : {'parsed 4x1024 from ' + ELEC_MAP_H.name if cell else 'NOT FOUND — SW image = raw'}")
    log(f"patterns    : -f value = frame<<12 | sensor;  -if value = 0x4000 | frame<<12 | sensor;  {N_FILE_FRAMES} frames each")
    log("")

    checks: list[Check] = []
    report: dict = {"legs": {}}

    def run_pair(leg: str, mode: int) -> None:
        log(f"=== leg {leg}: TELEM_EN=0x{mode:02X}  (-f + -if given to both) ===")
        emu_args = ["-f", str(dat), "-if", str(imp), "-loop"]
        sides = {}
        if not a.rtl_only:
            sides["sw"] = run_emulator("sw", sw_exe, emu_args, mode, a.frames, 30.0, 15.0, a.cmd_timeout, log)
            log(f"  [sw ] {len(sides['sw'].frames)} frames in {sides['sw'].seconds:.1f}s  {sides['sw'].error}")
        if not a.sw_only:
            sides["rtl"] = run_emulator("rtl", rtl_exe, emu_args, mode, a.frames,
                                        a.first_frame_timeout, a.frame_timeout, a.cmd_timeout, log)
            log(f"  [rtl] {len(sides['rtl'].frames)} frames in {sides['rtl'].seconds:.1f}s  {sides['rtl'].error}")

        legrep = {"mode": mode, "sides": {}}
        for k, r in sides.items():
            legrep["sides"][k] = {"frames": len(r.frames), "structural": structural(r), "ping": r.ping,
                                  "error": r.error, "seconds": round(r.seconds, 1), "short_msgs": r.n_short}
        report["legs"][leg] = legrep
        if len(sides) < 2:
            return
        sw, rtl = sides["sw"], sides["rtl"]

        def add(name: str, ok: bool, detail: str) -> None:
            key = f"{leg}/{name}"
            kn = known.get(key, "")
            checks.append(Check(leg, name, ok, detail, kn if not ok else ""))
            mark = "PASS" if ok else ("KNOWN" if kn else "FAIL")
            log(f"  {mark:5s} {key}: {detail}" + (f"  [known: {kn}]" if (kn and not ok) else ""))

        add("both_launch", sw.launch_ok and rtl.launch_ok, f"sw={sw.launch_ok} rtl={rtl.launch_ok} {sw.error} {rtl.error}")
        add("ping", sw.ping == rtl.ping and all(p == 0x55 for p in sw.ping),
            f"sw={['0x%02X' % p for p in sw.ping]} rtl={['0x%02X' % p for p in rtl.ping]}")
        add("frames_flow", (len(sw.frames) > 0) == (len(rtl.frames) > 0),
            f"sw={len(sw.frames)} rtl={len(rtl.frames)} frames captured")
        if not sw.frames or not rtl.frames:
            return
        ss, sr = structural(sw), structural(rtl)
        for chk in ("tag_ok", "cnt_hi_ok", "phase_ok", "crc_ok", "monotonic"):
            add(f"structural/{chk}", ss[chk] == sr[chk], f"sw={ss[chk]} rtl={sr[chk]}")
        log(f"        frame_cnt sw={ss['frame_cnts']} rtl={sr['frame_cnts']}")
        add("structural/phase_flags", ss["phase_flags"] == sr["phase_flags"],
            f"bits[14:12] seen sw={ss['phase_flags']} rtl={sr['phase_flags']}")
        # Phase VALUES are timing-dependent, so only their zero/non-zero character
        # is compared: a side that always reports 0 has anchored its legs, a side
        # that reports varying values is modelling per-leg start lag.
        add("structural/phase_reporting", (ss["phase_values"] == [0]) == (sr["phase_values"] == [0]),
            f"phase values sw={ss['phase_values'][:8]} rtl={sr['phase_values'][:8]}")

        # --- semantic: recover per side, match frames by the source frame the data names
        tag = 0x0000 if mode == 1 else IMP_TAG
        expect_name = "-f file" if mode == 1 else "-if file"

        def recover(r: RunResult, use_map: bool) -> dict[int, list[int]]:
            imgs: dict[int, list[int]] = {}
            for f in r.frames:
                if mode == 1:
                    arrays = [decode_normal(f.words, f.phases)]
                else:
                    arrays = decode_impedance(f.words, f.phases, mode & 1)
                for arr in arrays:
                    img = to_elec_image(arr, cell if use_map else None)
                    ff, agree = file_frame_of(img, tag)
                    if ff is not None:
                        imgs.setdefault(ff, img)
            return imgs

        sw_imgs, rtl_imgs = recover(sw, True), recover(rtl, False)
        log(f"        source frames seen sw={sorted(sw_imgs)} rtl={sorted(rtl_imgs)}")
        common = sorted(ff for ff in sw_imgs if ff in rtl_imgs)
        add("semantic/source_frames_overlap", bool(common), f"{len(common)} source frame(s) captured by both")
        if not common:
            # still say which file each side is showing — the most useful diagnostic
            for k, r in (("sw", sw), ("rtl", rtl)):
                f0 = r.frames[0]
                arr = decode_normal(f0.words, f0.phases) if mode == 1 else decode_impedance(f0.words, f0.phases, mode & 1)[0]
                tags = sorted({v & 0xC000 for v in arr if v != NOVAL})
                log(f"        info {k} first frame carries tag nibbles {[hex(t) for t in tags]} (expected {hex(tag)})")
            return
        n_cmp = n_diff = 0
        shape = ""
        for ff in common:
            n, d, ex = compare_defined(sw_imgs[ff], rtl_imgs[ff])
            n_cmp += n
            n_diff += d
            if d and not shape:
                shape = classify(sw_imgs[ff], rtl_imgs[ff]) + "; e.g. " + ", ".join(
                    f"[{i}] sw=0x{x:04X} rtl=0x{y:04X}" for i, x, y in ex[:4])
        add("semantic/recovered_image_equal", n_diff == 0,
            f"{n_diff} differing of {n_cmp} sensors compared across {len(common)} matched frame(s)"
            + (f" — {shape}" if shape else ""))
        # Each side vs the file it SHOULD be showing.  A pass/fail check per side:
        # this is "does the emulator report the emulation data you gave it".
        for k, imgs in (("sw", sw_imgs), ("rtl", rtl_imgs)):
            ff = common[0]
            exp = expected_image(tag, ff)
            n, d, ex = compare_defined(imgs[ff], exp)
            add(f"semantic/{k}_reports_{'sample' if mode == 1 else 'impedance'}_file", d == 0,
                f"{n - d}/{n} carried sensors equal the {expect_name} frame {ff}"
                + ("" if d == 0 else "; e.g. " + ", ".join(f"[{i}] got=0x{x:04X} want=0x{y:04X}" for i, x, y in ex[:3])))
            report["legs"][leg]["sides"][k]["vs_file"] = {"compared": n, "equal": n - d}
        report["legs"][leg]["semantic"] = {"matched_frames": len(common), "compared": n_cmp,
                                           "diff_sensors": n_diff, "shape": shape}

    for leg in [s.strip() for s in a.legs.split(",") if s.strip()]:
        if leg not in LEGS:
            log(f"unknown leg {leg}")
            return 2
        run_pair(leg, LEGS[leg])

    if not a.keep_dat:
        dat.unlink(missing_ok=True)
        imp.unlink(missing_ok=True)

    n_pass = sum(1 for c in checks if c.ok)
    n_known = sum(1 for c in checks if not c.ok and c.known)
    n_fail = sum(1 for c in checks if not c.ok and not c.known)
    report["checks"] = [c.__dict__ for c in checks]
    report["summary"] = {"pass": n_pass, "known_divergent": n_known, "fail": n_fail}
    if a.out:
        Path(a.out).write_text(json.dumps(report, indent=2), encoding="utf-8")
    log("")
    log(f"checks: {n_pass} pass, {n_known} known-divergent, {n_fail} fail")
    for c in checks:
        if not c.ok and c.known:
            log(f"  known : {c.leg}/{c.name} — {c.known}")
    for c in checks:
        if not c.ok and not c.known:
            log(f"  FAIL  : {c.leg}/{c.name} — {c.detail}")
    # Sentinel lines: the same contract the SV benches emit (result_contract.py).
    log(f"RESULTS: {n_pass} passed, {n_fail} failed")
    log(f"STATUS: {'PASS' if n_fail == 0 and checks else 'FAIL'}")
    return 0 if (n_fail == 0 and checks) else 1


if __name__ == "__main__":
    sys.exit(main())
