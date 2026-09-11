#!/usr/bin/env python3
"""Suite host target `emulator_integration`: the SW emulator's integration test.

Runs `ionm_test.exe` (Software Emulator/test_app/src/ionm_test.cpp — the 72-check
bring-up + streaming + BIST + watchdog sequence through the FTD3XX shim against
ionm_emulator.exe) from its own build directory, where it finds the emulator and
the shim DLL and writes its sample/received files, and re-emits its verdict in
the suite's result contract (`RESULTS: N passed, M failed` / `STATUS: PASS|FAIL`).

Windows only: the FTD3XX shim and the test app are not built elsewhere; the
suite reports SKIP there.

Known flake (2026-09-11): BL-11 ("BIST counter contiguous (0 words...)") and
BL-12 ("reconnect for watchdog test") — both the first step after the test's
own pipe close/re-open — fail in roughly one run in four; the race is between
the test app's reconnect and the emulator re-listening, not in the model under
test (it was present before the transport port, see CLAUDE.md).  Until it is
fixed the target retries ONCE, only when every failure is one of those two
checks, and says so loudly: the first transcript is kept in the log and the
summary line carries "(flaky: retried once)".  Any other failure is final.

    python fpga-test/emulator/run_ionm_test.py            # default: --emulator=sw
    python fpga-test/emulator/run_ionm_test.py --emulator=rtl
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
TEST_ROOT = HERE.parent
DUT_ROOT = Path(os.environ.get("IONM_DUT_ROOT", TEST_ROOT.parent))
RELEASE = DUT_ROOT / "Software Emulator" / "build" / "test_app" / "Release"
EXE = RELEASE / "ionm_test.exe"

RETRYABLE = ("BL-11:", "BL-12: reconnect")
SUMMARY_RE = re.compile(r"^\s*PASS:\s*(\d+)\s+FAIL:\s*(\d+)\s*$", re.M)


def run_once(args: list[str], timeout: float) -> tuple[int, int, int, list[str], str]:
    """(exit, n_pass, n_fail, failing check lines, transcript)."""
    try:
        c = subprocess.run([str(EXE), *args], cwd=str(RELEASE), capture_output=True, text=True,
                           encoding="utf-8", errors="replace", timeout=timeout)
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or "") if isinstance(e.stdout, str) else ""
        return 124, 0, 0, [f"ionm_test timed out after {timeout:.0f} s"], out
    out = (c.stdout or "") + (("\n--- stderr ---\n" + c.stderr) if c.stderr else "")
    m = SUMMARY_RE.search(c.stdout or "")
    n_pass, n_fail = (int(m.group(1)), int(m.group(2))) if m else (0, 0)
    fails = [ln.strip() for ln in (c.stdout or "").splitlines() if ln.startswith("[FAIL]")]
    if c.returncode != 0 and not fails:
        fails = [f"ionm_test exited {c.returncode} without a summary"]
    return c.returncode, n_pass, n_fail, fails, out


def main() -> int:
    args = [a for a in sys.argv[1:] if a.startswith("--")]
    if sys.platform != "win32":
        print(f"SKIP: ionm_test is Windows-only (this is {sys.platform})")
        print("RESULTS: 0 passed, 0 failed"); print("STATUS: SKIP")
        return 0
    if not EXE.exists():
        print(f"SKIP: {EXE} not built")
        print("RESULTS: 0 passed, 0 failed"); print("STATUS: SKIP")
        return 0
    print(f"ionm_test : {EXE}")
    print(f"emulator  : {RELEASE / 'ionm_emulator.exe'} ({'present' if (RELEASE / 'ionm_emulator.exe').exists() else 'MISSING'})")
    print(f"args      : {args or ['(default --emulator=sw)']}")
    t0 = time.time()
    rc, n_pass, n_fail, fails, out = run_once(args, 600)
    print(out)
    retried = False
    if rc != 0 and fails and all(any(f.startswith("[FAIL] " + k) for k in RETRYABLE) for f in fails):
        print("\n=== first run failed only on the known reconnect-race checks — retrying once ===")
        for f in fails:
            print("   first run:", f)
        retried = True
        rc, n_pass, n_fail, fails, out = run_once(args, 600)
        print(out)
    secs = time.time() - t0
    for f in fails:
        print("FAIL:", f)
    note = "  (flaky: retried once — reconnect race BL-11/BL-12, see run_ionm_test.py)" if retried else ""
    print(f"ionm_test: {n_pass} passed, {n_fail} failed in {secs:.0f} s{note}")
    print(f"RESULTS: {n_pass} passed, {n_fail} failed")
    print(f"STATUS: {'PASS' if rc == 0 and n_fail == 0 else 'FAIL'}")
    return 0 if (rc == 0 and n_fail == 0) else 1


if __name__ == "__main__":
    sys.exit(main())
