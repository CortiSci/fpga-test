#!/usr/bin/env python3
"""The BER Test over the acquisition stream, against the SW emulator.

The bring-up tool's "BER Test" (CannedFunctions::berTest) runs all four tails in
self-test -- asic_self_test.v's 16-bit counter, +1 per stream word, continuous
across frames -- through the acquisition path and compares every delivered raw
word with the word the counter must produce next.  The SW emulator models the
same counter (tail_fpga.cpp: SELF_TEST_START, +1 per word), so the same protocol
is driven here at the named pipe with the differential test's helpers and scored
with the tool's rule:

    per leg, anchor on the first word delivered, then expected = previous + 1;
    a leg-frame whose phase word carries par or undf, or reports 1023, is skipped
    (ovf is sticky on hardware and is not an exclusion by itself)
    (right-or-flagged: a flagged frame is not a bit error) and the leg re-anchors;
    a frame whose CRC fails is skipped whole.

Checks (RESULTS:/STATUS: contract):
  * all four legs deliver scorable frames;
  * bit-exact: 0 errors over every compared bit (N reported with 3/N);
  * one flipped bit in one word scores exactly one error;
  * the run's bit budget: >= 1e6 compared bits per leg in the frames taken.

    python fpga-test/emulator/test_ber_stream.py [--exe PATH] [--frames N] [-v]
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
import diff_emulators as dx                      # PipeClient / Host / Frame / register map

IS_WIN = os.name == "nt"
TAIL_SELF_TEST = 0x06
RESULTS: list[tuple[str, bool, str]] = []
VERBOSE = False


def check(name: str, ok: bool, detail: str = "") -> None:
    RESULTS.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail and (VERBOSE or not ok) else ""))


class Emu:
    def __init__(self, exe: Path):
        self.base = f"BERS{os.getpid() % 10000}"
        self.dir = Path(tempfile.mkdtemp(prefix="berstream_"))
        self.proc = subprocess.Popen([str(exe), "-pipe", self.base, "-log", str(self.dir / "emu.log")],
                                     cwd=str(exe.parent), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.pipe = dx.PipeClient(self.base)
        if not self.pipe.wait_for_pipe(20):
            raise RuntimeError("emulator did not create its pipes")
        self.pipe.connect()
        self.host = dx.Host(self.pipe, 5.0, lambda s: None)

    def stop(self) -> None:
        try:
            self.host.stop()
            self.host.drain(0.3)
            self.pipe.close()
        except Exception:
            pass
        self.proc.terminate()
        try:
            self.proc.wait(10)
        except subprocess.TimeoutExpired:
            self.proc.kill()


class LegScore:
    """The tool's per-leg counter tracker."""

    def __init__(self) -> None:
        self.exp: int | None = None
        self.bits = 0
        self.errs = 0
        self.frames_scored = 0
        self.frames_flagged = 0

    def frame(self, words: list[int], phase: int) -> None:
        if (phase & 0x5000) or (phase & 0x3FF) == 0x3FF:   # par | undf | no phase; ovf (0x2000) is sticky, not an exclusion
            self.frames_flagged += 1
            self.exp = None
            return
        exp = self.exp
        if exp is None:
            exp = words[0]
            words = words[1:]                    # the anchor word itself is not scored
            exp = (exp + 1) & 0xFFFF
        expected = [(exp + i) & 0xFFFF for i in range(len(words))]
        if words != expected:                    # fast path: an exact frame costs one list compare
            self.errs += sum(bin(a ^ b).count("1") for a, b in zip(words, expected))
        self.bits += 16 * len(words)
        self.exp = (exp + len(words)) & 0xFFFF
        self.frames_scored += 1


def poisson_upper_95(k: int) -> float:
    """95% upper bound on a Poisson mean given k observed = chi2(0.95; 2k+2)/2 (bisection on the CDF)."""
    import math
    def cdf(lam: float) -> float:
        term = math.exp(-lam); total = term
        for i in range(1, k + 1):
            term *= lam / i; total += term
        return total
    lo, hi = 0.0, 1.0
    while cdf(hi) > 0.05:
        hi *= 2
    for _ in range(200):
        mid = (lo + hi) / 2
        if cdf(mid) > 0.05: lo = mid
        else: hi = mid
    return (lo + hi) / 2


def decide(bits: int, errs: int, n_target: int, accept: float, excluded_frac: float, max_excluded: float) -> tuple[str, float]:
    """IONMA-171 v14, one run: Pass / Fail / Inconclusive with the demonstrated BER U."""
    u = poisson_upper_95(errs) / bits if bits else float("inf")
    if bits < n_target or excluded_frac > max_excluded:
        return "Inconclusive", u
    return ("Pass" if u <= accept else "Fail"), u


def decide_with_retest(run1: tuple[int, int, float], run2: tuple[int, int, float] | None,
                       n_target: int, accept: float, retest_mult: int, max_excluded: float) -> tuple[str, float]:
    """Pooled retest (REQ 12): a Fail/Inconclusive first run is followed once by R x N more bits and the
    decision is made on the combined count.  run = (bits, errs, excluded_frac)."""
    out, u = decide(run1[0], run1[1], n_target, accept, run1[2], max_excluded)
    if out == "Pass" or retest_mult == 0 or run2 is None:
        return out, u
    b = run1[0] + run2[0]; k = run1[1] + run2[1]
    excl = (run1[2] * run1[0] + run2[2] * run2[0]) / max(b, 1)
    return decide(b, k, n_target * (1 + retest_mult), accept, excl, max_excluded)


def group_decision() -> None:
    """The requirement's arithmetic, offline: the table in the IONMA-171 v14 redline."""
    N, B = 100_000_000, 1e-7
    check("k=0 over N=1e8 passes at 1e-7 (U=3.0e-8)", decide(N, 0, N, B, 0, 0.05)[0] == "Pass",
          f"U={decide(N, 0, N, B, 0, 0.05)[1]:.3e}")
    u4 = decide(N, 4, N, B, 0, 0.05)
    check("k=4 over N=1e8 passes (U=9.15e-8)", u4[0] == "Pass" and abs(u4[1] - 9.15e-8) < 0.05e-8, f"U={u4[1]:.3e}")
    u5 = decide(N, 5, N, B, 0, 0.05)
    check("k=5 over N=1e8 fails (U=1.05e-7)", u5[0] == "Fail" and u5[1] > B, f"U={u5[1]:.3e}")
    check("fewer than N bits is Inconclusive, not Pass", decide(N // 2, 0, N, B, 0, 0.05)[0] == "Inconclusive")
    check("excluded frames above the maximum is Inconclusive", decide(N, 0, N, B, 0.06, 0.05)[0] == "Inconclusive")
    check("minimum passing run is N = 3/B", decide(int(3 / B) + 1, 0, int(3 / B), B, 0, 0.05)[0] == "Pass"
          and decide(int(2.9 / B), 0, int(2.9 / B), B, 0, 0.05)[0] == "Fail")
    pooled = decide_with_retest((N, 5, 0.0), (2 * N, 0, 0.0), N, B, 2, 0.05)
    check("pooled retest: k=5/N then 0/2N -> 5 over 3N passes (U=3.5e-8)", pooled[0] == "Pass" and pooled[1] < B, f"{pooled}")
    pooled2 = decide_with_retest((N, 5, 0.0), (2 * N, 6, 0.0), N, B, 2, 0.05)
    check("pooled retest: 11 errors over 3N still passes (U=6.1e-8)", pooled2[0] == "Pass" and abs(pooled2[1] - 6.07e-8) < 0.1e-8, f"{pooled2}")
    pooled3 = decide_with_retest((N, 5, 0.0), (2 * N, 20, 0.0), N, B, 2, 0.05)
    check("pooled retest: 25 errors over 3N fails (U=1.16e-7)", pooled3[0] == "Fail" and pooled3[1] > B, f"{pooled3}")
    check("no retest when R = 0", decide_with_retest((N, 5, 0.0), (2 * N, 0, 0.0), N, B, 0, 0.05)[0] == "Fail")


def start_self_test_stream(h: dx.Host) -> None:
    """startSelfTestStream(): 800 kHz cfg SPI, all legs enabled, per tail CTRL 0x11 + SELF_TEST + TELEM_EN, then RUN."""
    h.p.write_reg(dx.REG_SPI_CLK_DIV, 31, h.t, h.stray)
    h.p.write_reg(dx.REG_SPI_ENABLE_MASK, 0x000F, h.t, h.stray)
    for ch in range(4):
        h.spi_xact(ch, 2, [dx.TAIL_CTRL, dx.CTRL_RUN], False)
        h.spi_xact(ch, 2, [TAIL_SELF_TEST, 1], False)
        h.spi_xact(ch, 2, [dx.TAIL_TELEM_EN, 1], False)
    h.p.write_reg(dx.REG_ACQ_ALL_RUN, 0x000F, h.t, h.stray)


def run(exe: Path, n_frames: int) -> None:
    emu = Emu(exe)
    try:
        start_self_test_stream(emu.host)
        legs = [LegScore() for _ in range(4)]
        crc_bad = skipped_start = 0
        last_cnt = None
        t0 = time.time()
        taken = 0
        while taken < n_frames + 2 and time.time() - t0 < 60:
            p = emu.pipe.next_frame(5.0)
            if p is None:
                break
            fr = dx.Frame.parse(p)
            taken += 1
            if taken <= 2:                       # start-up frames, as the tool discards them
                skipped_start += 1
                continue
            if not fr.crc_ok:
                crc_bad += 1
                for l in legs:
                    l.exp = None
                continue
            if last_cnt is not None and fr.frame_cnt != last_cnt + 1:
                for l in legs:                   # a gap in the counter: re-anchor
                    l.exp = None
            last_cnt = fr.frame_cnt
            for ch in range(4):
                legs[ch].frame(fr.words[3 + ch:3 + 4096:4], fr.phases[ch])
        dt = time.time() - t0

        bits_all = sum(l.bits for l in legs)
        errs_all = sum(l.errs for l in legs)
        for ch, l in enumerate(legs):
            if VERBOSE:
                print(f"  leg {5 + ch}: {l.frames_scored} frames scored, {l.frames_flagged} flagged, N={l.bits}, k={l.errs}")
        check("all four legs delivered scorable frames", all(l.frames_scored > 0 for l in legs),
              "/".join(str(l.frames_scored) for l in legs) + f" of {taken - 2} (crc bad {crc_bad})")
        check("bit-exact on the acquisition stream", bits_all > 0 and errs_all == 0,
              f"{errs_all} errors over N={bits_all} bits (demonstrates BER <= {3.0 / max(bits_all, 1):.2e}); {dt:.1f} s")
        check(">= 1e6 compared bits per leg", all(l.bits >= 1_000_000 for l in legs),
              "/".join(f"{l.bits}" for l in legs))

        # the comparator: one flipped bit in one word of an otherwise exact frame
        probe = LegScore()
        base = 1234
        words = [(base + i) & 0xFFFF for i in range(1024)]
        probe.frame(words, 0)
        words2 = [(base + 1024 + i) & 0xFFFF for i in range(1024)]
        words2[100] ^= 0x0040
        probe.frame(words2, 0)
        check("comparator sees one flipped bit as one error", probe.errs == 1 and probe.bits == 16 * 2047,
              f"k={probe.errs}, N={probe.bits}")
    finally:
        emu.stop()


def main() -> int:
    global VERBOSE
    ap = argparse.ArgumentParser()
    default_exe = (HERE / ".." / ".." / "Software Emulator" / "build" / "test_app" /
                   "Release" / ("ionm_emulator.exe" if IS_WIN else "ionm_emulator")).resolve()
    ap.add_argument("--exe", type=Path, default=default_exe)
    ap.add_argument("--frames", type=int, default=400, help="frames scored (4 x 16384 bits each)")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    VERBOSE = a.verbose

    if not a.exe.exists():
        print(f"SKIP: emulator not built at {a.exe}")
        print("RESULTS: 0 passed, 0 failed")
        print("STATUS: SKIP")
        return 0

    print(f"BER Test over the acquisition stream — {a.exe}")
    try:
        group_decision()
        run(a.exe, a.frames)
    except Exception as e:                              # noqa: BLE001
        check("run completed", False, f"{type(e).__name__}: {e}")

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
