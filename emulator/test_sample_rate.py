#!/usr/bin/env python3
"""Measure delivered normal telemetry: 2500 full-grid frames/s (per-sensor Hz).

Two steady 2 s windows must be within 5%; between them pause the actual reader
for 1.2 s and allow 2 s of catch-up. RTL permits buffered bursts and drops under
backpressure. Total delivery across the stall/recovery must not exceed elapsed
time's sample budget. Counter gaps are allowed; repeated/backward counters fail.
A unique endpoint isolates this from interactive emulators.
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
    pause = threading.Event()
    paused = threading.Event()
    resume = threading.Event()

    def reader():
        # Consume telemetry immediately, timestamp at the transport boundary,
        # and keep only metadata. Commands still use the existing response queue.
        try:
            while not pipe._stop.is_set():
                if pause.is_set():
                    paused.set()
                    resume.wait(5)
                    pause.clear()
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
            time.sleep(2)
            pause.set()
            if not paused.wait(2):
                raise RuntimeError('reader did not acknowledge stall')
            time.sleep(1.2)
            recovery = time.perf_counter()
            resume.set()
            time.sleep(2)
            steady = time.perf_counter()
            time.sleep(2.1)
            end = steady + 2
            with lock:
                captured = [s for s in samples if start <= s[0] < end]
            for label, window in [('before stall', start), ('after recovery', steady)]:
                count = sum(window <= s[0] < window + 2 for s in captured)
                rate = count / 2
                check(f'{label}: 2500 samples/s per sensor', 2375 <= rate <= 2625,
                      f'{rate:.1f} frames/s (5% timing tolerance)')
            catchup = sum(recovery <= s[0] < steady for s in captured) / (steady - recovery)
            print(f'INFO: catch-up delivery {catchup:.1f} frames/s; buffered bursts are permitted')
            check('no overproduction across stall and recovery',
                  len(captured) <= 2500 * (end - start) * 1.05,
                  f'{len(captured)} frames in {end-start:.3f}s (includes stopped reader)')
            check('no repeated or backward frame counters', len(captured) > 1 and
                  all(0 < ((b[1] - a[1]) & 0x1fffffff) < 0x10000000
                      for a, b in zip(captured, captured[1:])),
                  f'{len(captured)} frames')
            check('valid telemetry headers and live reader', bool(captured) and
                  all(s[2] for s in captured) and not errors and pipe._thr.is_alive() and proc.poll() is None,
                  '; '.join(errors))
        except Exception as exc:
            check('capture completed', False, str(exc))
        finally:
            resume.set()
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
