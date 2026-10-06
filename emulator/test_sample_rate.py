#!/usr/bin/env python3
"""Measure delivered normal telemetry: 2500 full-grid frames/s (per-sensor Hz).

Three independent 2 s windows, after 0.5 s warmup, must each be within 5%.
This is a wall-clock integration test, not a frame-counter speed estimate:
missing/repeated counters also fail. OS/pipe batching is allowed. A unique
endpoint and looping one-frame fixture isolate this from interactive emulators.
Run: python -B fpga-test/emulator/test_sample_rate.py [--exe PATH]
"""
import argparse
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import threading
import time

import diff_emulators as dx


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exe', type=Path, default=dx.RELEASE / f'ionm_emulator{dx.EXE}')
    args = parser.parse_args()
    results = []

    def check(name, ok, detail=''):
        results.append(ok)
        print(f"{'PASS' if ok else 'FAIL'}: {name}: {detail}", flush=True)

    pipe = dx.PipeClient(f'IONM_RATE_{os.getpid()}')
    samples = []
    lock = threading.Lock()
    errors = []

    def reader():
        # Consume telemetry immediately, timestamp at the transport boundary,
        # and keep only metadata. Commands still use the existing response queue.
        try:
            while not pipe._stop.is_set():
                hdr = pipe._read_exact(2)
                if hdr is None:
                    return
                size, = struct.unpack('<H', hdr)
                payload = pipe._read_exact(size)
                if payload is None:
                    return
                if size == dx.TELEM_WORDS * 2:
                    stamp = time.perf_counter()
                    tag, lo, hi = struct.unpack_from('<3H', payload)
                    with lock:
                        samples.append((stamp, lo | ((hi & 0x1fff) << 16), tag == 1 and hi < 8192))
                else:
                    pipe.q.put(payload)
        except Exception as exc:
            errors.append(str(exc))

    pipe._reader = reader
    proc = None
    with tempfile.TemporaryDirectory(prefix='ionm_rate_') as tmp:
        dat = Path(tmp) / 'loop.dat'
        dat.write_bytes(struct.pack('<4096H', *range(4096)))
        try:
            proc = subprocess.Popen([str(args.exe.resolve()), '-pipe', pipe.base,
                                     '-f', str(dat), '-loop', '-log', str(Path(tmp) / 'emu.log')],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if not pipe.wait_for_pipe(5):
                raise RuntimeError('emulator endpoint did not appear')
            pipe.connect()
            dx.Host(pipe, 2.0, lambda _: None).bringup_and_run(1)
            time.sleep(0.5)
            start = time.perf_counter()
            time.sleep(6.1)
            with lock:
                captured = [s for s in samples if start <= s[0] < start + 6]
            for i in range(3):
                count = sum(start + 2*i <= s[0] < start + 2*(i+1) for s in captured)
                rate = count / 2
                check(f'window {i+1}: 2500 samples/s per sensor', 2375 <= rate <= 2625,
                      f'{rate:.1f} frames/s (5% timing tolerance)')
            check('continuous delivered frame counters', len(captured) > 1 and
                  all(((b[1] - a[1]) & 0x1fffffff) == 1 for a, b in zip(captured, captured[1:])),
                  f'{len(captured)} frames')
            check('valid telemetry headers and live reader', bool(captured) and
                  all(s[2] for s in captured) and not errors and pipe._thr.is_alive() and proc.poll() is None,
                  '; '.join(errors))
        except Exception as exc:
            check('capture completed', False, str(exc))
        finally:
            # Kill only this test's process; unblock its reader before closing.
            if proc is not None:
                proc.kill()
                proc.wait(timeout=5)
            pipe.close()
            if pipe._thr:
                pipe._thr.join(timeout=2)
    passed = sum(results)
    failed = len(results) - passed
    print(f'RESULTS: {passed} passed, {failed} failed')
    print('STATUS: ' + ('PASS' if failed == 0 else 'FAIL'))
    return int(failed != 0)


if __name__ == '__main__':
    raise SystemExit(main())
