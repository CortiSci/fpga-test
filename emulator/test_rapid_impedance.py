"""Exercise all 64 consecutive impedance positions without a 10 ms dwell delay.

Uses the protocol runner's injector/decoder; reports command RTT separately.
"""
import argparse
import os
from pathlib import Path
import statistics
import time
import csv
import diff_emulators as dx


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--exe', required=True)
    ap.add_argument('--fixture', type=Path, help='directory from make_emulator_impedance_example.py')
    args = ap.parse_args()
    base = f'IONM_RAPID_{os.getpid()}'
    # Isolate this run from interactive emulator sessions.
    original_init = dx.PipeClient.__init__
    def init(self):
        original_init(self, base)
    dx.PipeClient.__init__ = init
    dx.INJECT_ROWS = list(range(64))
    dx.INJECT_LANES = [0, 1]
    dx.INJECT_FRAMES = 4
    rtts = []
    original_cmd = dx.PipeClient.do_cmd
    def timed(self, *a, **kw):
        start = time.perf_counter()
        result = original_cmd(self, *a, **kw)
        rtts.append(1000 * (time.perf_counter() - start))
        return result
    dx.PipeClient.do_cmd = timed
    # Stop's acknowledged command consumes preceding telemetry. No extra wait.
    dx.Host.drain = lambda self, quiet_s: 0
    emu_args = ['-pipe', base]
    expected = {}
    if args.fixture:
        emu_args += ['-if', str((args.fixture/'impedance-example.dat').resolve()), '-loop']
        with (args.fixture/'expected.csv').open() as f:
            for r in csv.DictReader(f):
                expected[(int(r['leg'])-5, int(r['lane_0based']), int(r['asic_row_0based']))] = int(r['peak_to_peak_counts'])
    result, measurements = dx.run_inject('sw', Path(args.exe).resolve(),
        emu_args, 5, 5, 5, print)
    failures = []
    if result.error:
        failures.append(result.error)
    for (ch, lane, row), value in measurements.items():
        if value is None or value[0] != 63-row or value[1] <= 0:
            failures.append(f'leg{ch+5} lane{lane} row{row}: {value}')
        elif expected and value[1] != expected[(ch, lane, 63-row)]:
            failures.append(f'leg{ch+5} lane{lane} row{row}: swing {value[1]}, expected {expected[(ch, lane, 63-row)]}')
    if len(measurements) != 512:
        failures.append(f'only {len(measurements)}/512 measurements')
    if rtts:
        print(f'Command RTT: median {statistics.median(rtts):.3f} ms; '
              f'p95 {sorted(rtts)[int(.95*len(rtts))]:.3f} ms; max {max(rtts):.3f} ms')
    for failure in failures:
        print('FAIL:', failure)
    print(f'RESULTS: {len(measurements)-len(failures)} passed, {len(failures)} failed')
    return bool(failures)


if __name__ == '__main__':
    raise SystemExit(main())
