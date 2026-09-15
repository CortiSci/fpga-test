#!/usr/bin/env python3
"""The BER Test's loopback against the SW emulator, at the named pipe.

The bring-up tool's "BER Test" (CannedFunctions::berTest, tools/ber_loopback.py)
burst-writes a PRBS-23 into the ASIC's 1536-bit Pixel chain through the tail's
CS2_PASS passthrough and burst-reads it back on the MISO echo.  The SW emulator
models that echo (tail_fpga.cpp returns asic_pixel_model.h::chain_out() during
an SS1 passthrough), so the same protocol is driven here with the differential
test's PipeClient/Host helpers -- the exact register writes the tool issues --
and scored with the tool's rule:

    align each burst on its own (stride {27, 24} bits/read, offset -4..4);
    live = at least half the candidate reads match; errors = popcount(XOR).

Checks (RESULTS:/STATUS: contract):
  * every leg: one 64-word burst is live, bit-exact (0 errors), at stride 24
    (the emulator's byte model returns exactly the 24-bit word per read; the
    hardware's stride is 27 -- see the RTL bench consolidator_ber_loopback);
  * a second burst on leg 0 after its read-out drained the chain is clean too;
  * the comparator sees one flipped bit as exactly one error;
  * tools/ber_loopback.py itself, through the FTD3XX shim (Windows, where the
    shim is built): --endpoint asic passes, --endpoint token --inject 3 fails.

    python fpga-test/emulator/test_ber.py [--exe PATH] [-v]
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import diff_emulators as dx                      # PipeClient / Host / register map

IS_WIN = os.name == "nt"
WORDS = 64                                       # one chain-full per burst
TAIL_CTRL = 0x01
RESULTS: list[tuple[str, bool, str]] = []
VERBOSE = False


def check(name: str, ok: bool, detail: str = "") -> None:
    RESULTS.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail and (VERBOSE or not ok) else ""))


# ---------------------------------------------------------------------------
# the tool's PRBS and scoring, re-implemented (deliberately not imported from
# tools/ber_loopback.py: an independent copy is also a check on the tool)
# ---------------------------------------------------------------------------
class Prbs23:
    def __init__(self, seed: int = 0x7FFFF):
        self.s = seed & 0x7FFFFF

    def next16(self) -> int:
        w = 0
        for _ in range(16):
            bit = ((self.s >> 22) ^ (self.s >> 17)) & 1
            self.s = ((self.s << 1) | bit) & 0x7FFFFF
            w = (w << 1) | bit
        return w

    def word24(self) -> int:
        return ((self.next16() << 8) | (self.next16() & 0xFF)) & 0xFFFFFF


def bit_window(words: list[int], bitoff: int) -> int | None:
    total = len(words) * 24
    if bitoff < 0 or bitoff + 24 > total:
        return None
    v = 0
    for t in range(24):
        b = bitoff + t
        v = (v << 1) | ((words[b // 24] >> (23 - b % 24)) & 1)
    return v


def align(sent: list[int], got: list[int]) -> tuple[int, int, int, int]:
    best = None
    for S in (27, 24):
        for k in range(-4, 5):
            hits = cand = 0
            for j, g in enumerate(got):
                e = bit_window(sent, k + S * j)
                if e is None:
                    continue
                cand += 1
                hits += (e == g)
            if best is None or hits > best[2]:
                best = (S, k, hits, cand)
    return best


def score(sent: list[int], got: list[int], S: int, k: int) -> tuple[int, int]:
    bits = errs = 0
    for j, g in enumerate(got):
        e = bit_window(sent, k + S * j)
        if e is None:
            continue
        errs += bin(e ^ g).count("1")
        bits += 24
    return bits, errs


# ---------------------------------------------------------------------------
class Emu:
    """A running emulator on a private pipe base.  `connect=False` leaves the
    CTRL/DATA pipes free for another client (the FTD3XX shim in group_tool)."""

    _n = 0

    def __init__(self, exe: Path, connect: bool = True):
        Emu._n += 1
        self.base = f"BERT{os.getpid() % 10000}_{Emu._n}"
        self.dir = Path(tempfile.mkdtemp(prefix="bertest_"))
        self.proc = subprocess.Popen([str(exe), "-pipe", self.base, "-log", str(self.dir / "emu.log")],
                                     cwd=str(exe.parent), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.pipe = dx.PipeClient(self.base)
        if not self.pipe.wait_for_pipe(20):
            raise RuntimeError("emulator did not create its pipes")
        self.host = None
        if connect:
            self.pipe.connect()
            self.host = dx.Host(self.pipe, 5.0, lambda s: None)

    def stop(self) -> None:
        try:
            if self.host is not None:
                self.pipe.close()
        except Exception:
            pass
        self.proc.terminate()
        try:
            self.proc.wait(10)
        except subprocess.TimeoutExpired:
            self.proc.kill()


def wake(h: dx.Host, legs: list[int]) -> None:
    """berTest's preamble: 800 kHz cfg clock, MCLK_EN on every tail, RO_RSTn on the legs under test."""
    h.p.write_reg(dx.REG_SPI_CLK_DIV, 31, h.t, h.stray)
    h.p.write_reg(dx.REG_SPI_ENABLE_MASK, 0x000F, h.t, h.stray)
    for c in range(4):
        h.spi_xact(c, 2, [TAIL_CTRL, 0x10], False)
    for c in legs:
        h.spi_xact(c, 2, [TAIL_CTRL, 0x11], False)


def asic_read(h: dx.Host, ch: int) -> int:
    """asicPassthroughRead: arm, then a read-mode zero shift-in that captures the echo."""
    h.spi_xact(ch, 2, [dx.TAIL_CS2_PASS, 0], True)
    rx = h.spi_xact(ch, 3, [0, 0, 0], True)
    return (rx[0] << 16) | (rx[1] << 8) | rx[2]


def burst(h: dx.Host, ch: int, prbs: Prbs23) -> tuple[list[int], list[int]]:
    sent = [prbs.word24() for _ in range(WORDS)]
    for w in sent:
        h.asic_write(ch, dx.TAIL_CS2_PASS, [(w >> 16) & 0xFF, (w >> 8) & 0xFF, w & 0xFF])
    got = [asic_read(h, ch) for _ in range(WORDS)]
    return sent, got


def group_pipe(exe: Path) -> None:
    emu = Emu(exe)
    try:
        h = emu.host
        wake(h, [0, 1, 2, 3])
        prbs = Prbs23()
        t0 = time.time()
        for ch in range(4):
            sent, got = burst(h, ch, prbs)
            S, k, hits, cand = align(sent, got)
            live = cand > 0 and hits >= cand // 2
            check(f"leg {5 + ch}: burst live", live, f"{hits}/{cand} match at stride {S} offset {k}")
            bits, errs = score(sent, got, S, k)
            check(f"leg {5 + ch}: bit-exact", live and bits > 0 and errs == 0,
                  f"{errs} errors over N={bits} bits (demonstrates BER <= {3.0 / max(bits, 1):.2e})")
            check(f"leg {5 + ch}: stride 24 offset 0 (emulator byte model)", live and S == 24 and k == 0,
                  f"stride {S} offset {k}")
            if ch == 0:
                bad = list(got)
                bad[10] ^= 0x000400
                _, errs2 = score(sent, bad, S, k)
                check("comparator sees one flipped bit as one error", errs2 == errs + 1, f"{errs} -> {errs2}")
        # a second burst on leg 0: the read-out shifted zeros through the chain; the
        # next write re-seeds it
        sent, got = burst(h, 0, prbs)
        S, k, hits, cand = align(sent, got)
        bits, errs = score(sent, got, S, k)
        check("leg 5: second burst after a read-out is live and clean",
              cand > 0 and hits >= cand // 2 and errs == 0, f"{hits}/{cand}, {errs} errors, stride {S}")
        if VERBOSE:
            print(f"  (5 bursts in {time.time() - t0:.1f} s)")
    finally:
        emu.stop()


def group_tool(exe: Path) -> None:
    """tools/ber_loopback.py through the FTD3XX shim, which honours IONM_PIPE_NAME."""
    tool = (HERE / ".." / ".." / "tools" / "ber_loopback.py").resolve()
    shim = exe.parent / "FTD3XX.dll"
    if not (IS_WIN and tool.exists() and shim.exists()):
        print(f"  (skipping the tool half: needs Windows, {tool.name} and {shim.name})")
        return
    emu = Emu(exe, connect=False)
    try:
        env = dict(os.environ, IONM_PIPE_NAME=emu.base, PYTHONIOENCODING="utf-8")

        def run(*args: str) -> subprocess.CompletedProcess:
            return subprocess.run([sys.executable, str(tool), "--dll", str(shim), *args],
                                  cwd=str(tool.parent.parent), env=env, capture_output=True, text=True, timeout=600)

        # positive: the ASIC endpoint, three chain-fulls, at an acceptance the run can reach
        r = run("--endpoint", "asic", "--ch", "1", "--bits", "6144", "--bmax", "1e-3")
        check("ber_loopback.py --endpoint asic passes via the shim",
              r.returncode == 0 and "RESULT: PASS" in r.stdout and "0 dead" in r.stdout,
              (r.stdout.strip().splitlines() or [r.stderr.strip()])[-1:][0] if (r.stdout or r.stderr) else f"rc={r.returncode}")
        # negative: the token endpoint with three injected flips must FAIL -- at k=3
        # U = 7.75/N, so 100 kbit gives 7.75e-5, above a 1e-5 acceptance
        r = run("--endpoint", "token", "--bits", "100000", "--inject", "3", "--bmax", "1e-5")
        check("ber_loopback.py --inject 3 fails via the shim",
              r.returncode == 1 and "RESULT: FAIL" in r.stdout and "bit errors k         3" in r.stdout,
              (r.stdout.strip().splitlines() or [r.stderr.strip()])[-1:][0] if (r.stdout or r.stderr) else f"rc={r.returncode}")
    finally:
        emu.stop()


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

    print(f"BER loopback against the SW emulator — {a.exe}")
    for g in (group_pipe, group_tool):
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
    raise SystemExit(main())
