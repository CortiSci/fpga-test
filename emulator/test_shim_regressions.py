"""Bounded Windows DLL regressions: cancellation, deadlines, failed-open handles.

Each scenario runs in a child process so a broken historical DLL cannot hang
the suite. --exe/--shim allow the identical tests to run on release artifacts.
"""
import argparse
import ctypes as c
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get('IONM_DUT_ROOT', HERE.parents[1]))
RELEASE = ROOT / 'Software Emulator/build/test_app/Release'


def deadline_case(shim, scenario):
    # A controlled server supplies a partial packet then deliberately goes idle.
    base = f'IONM_DEADLINE_{os.getpid()}'
    os.environ['IONM_PIPE_NAME'] = base
    buffered = scenario.startswith('buffered')
    os.environ['IONM_FTD_ACCURATE'] = '1' if buffered else '0'
    k = c.WinDLL('kernel32', use_last_error=True)
    k.CreateNamedPipeW.argtypes = [c.c_wchar_p] + [c.c_ulong]*6 + [c.c_void_p]
    k.CreateNamedPipeW.restype = c.c_void_p
    k.ConnectNamedPipe.argtypes = [c.c_void_p, c.c_void_p]
    k.WriteFile.argtypes = [c.c_void_p, c.c_void_p, c.c_ulong, c.c_void_p, c.c_void_p]
    k.CloseHandle.argtypes = [c.c_void_p]
    handles = []
    for suffix, access in [('_CTRL', 1), ('_DATA', 2)]:
        h = k.CreateNamedPipeW('\\\\.\\pipe\\' + base + suffix, access, 0, 1, 8192, 8192, 0, None)
        if h in (None, c.c_void_p(-1).value):
            raise c.WinError(c.get_last_error())
        handles.append(h)
    ft = c.WinDLL(str(shim))
    handle = c.c_void_p()
    if ft.FT_Create(None, 0, c.byref(handle)):
        raise RuntimeError('Cannot connect to controlled server')
    for h in handles:
        if not k.ConnectNamedPipe(h, None) and c.get_last_error() != 535:
            raise c.WinError(c.get_last_error())
    packet = bytes([8, 0, 1, 2, 3, 4, 5, 6, 7, 8])
    prefix = {'idle': 0, 'prefix': 1, 'payload': 5}.get(scenario, 0)
    def write(data):
        buf = c.create_string_buffer(data)
        count = c.c_ulong()
        if not k.WriteFile(handles[1], buf, len(data), c.byref(count), None) or count.value != len(data):
            raise RuntimeError('Fixture write failed')
    if prefix:
        write(packet[:prefix])
    abort_case = scenario == 'buffered-abort'
    ft.FT_SetPipeTimeout(handle, 0x82, 2000 if abort_case else 40)
    finished = threading.Event()
    def guard():
        if not finished.wait(0.040 if abort_case else 0.400):
            ft.FT_AbortPipe(handle, 0x82)
    worker = threading.Thread(target=guard, daemon=True)
    worker.start()
    buf, count = (c.c_ubyte*8)(), c.c_ulong(99)
    start = time.perf_counter()
    status = ft.FT_ReadPipe(handle, 0x82, buf, 8, c.byref(count), None)
    elapsed = (time.perf_counter()-start)*1000
    finished.set()
    worker.join()
    passed = status == (4 if abort_case else 19) and count.value == 0 and 20 <= elapsed < 250
    recovery = False
    if passed:
        write(packet[prefix:])
        status2 = ft.FT_ReadPipe(handle, 0x82, buf, 8, c.byref(count), None)
        recovery = status2 == 0 and count.value == 8 and bytes(buf) == packet[2:]
    result = dict(scenario=scenario, status=status, elapsed_ms=elapsed,
                  recovery=recovery, passed=passed and recovery)
    print(json.dumps(result), flush=True)
    ft.FT_Close(handle)
    for h in handles:
        k.CloseHandle(h)
    return 0 if result['passed'] else 1


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case', choices=('race', 'timeouts', 'connection', 'all'), default='all')
    p.add_argument('--exe', type=Path, default=RELEASE/'ionm_emulator.exe')
    p.add_argument('--shim', type=Path, default=RELEASE/'FTD3XX.dll')
    p.add_argument('--worker')
    p.add_argument('--out', type=Path)
    a = p.parse_args()
    if sys.platform != 'win32':
        print('SKIP: Windows FTD3XX shim required')
        return 0
    a.exe, a.shim = a.exe.resolve(), a.shim.resolve()
    if a.worker:
        return deadline_case(a.shim, a.worker)
    results = []
    with tempfile.TemporaryDirectory(prefix='ionm-shim-regressions-') as tmp:
        commands = []
        if a.case in ('race', 'all'):
            for mode in ('direct', 'buffered'):
                for delay in (0, 50):
                    name = f'abort-{mode}-{delay}ms'
                    cmd = [str(HERE/'probe_shim_overlapped_abort.py'), '--exe', str(a.exe),
                           '--shim', str(a.shim), '--delay-ms', str(delay), '--out', str(Path(tmp)/(name+'.json'))]
                    if mode == 'buffered': cmd += ['--mode2']
                    commands.append((name, cmd))
        if a.case in ('timeouts', 'all'):
            for name in ('idle', 'prefix', 'payload', 'buffered-idle', 'buffered-abort'):
                commands.append((name, [str(Path(__file__).resolve()), '--worker', name, '--shim', str(a.shim)]))
        if a.case in ('connection', 'all'):
            commands.append(('failed-open-handle', [str(HERE/'test_shim_connection.py'), '--exe', str(a.exe), '--shim', str(a.shim)]))
        for name, cmd in commands:
            start = time.monotonic()
            try:
                env = dict(os.environ, IONM_FTD_ACCURATE='0')
                proc = subprocess.Popen([sys.executable, '-B', *cmd], stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, text=True, env=env)
                stdout, stderr = proc.communicate(timeout=15)
                result = dict(name=name, passed=proc.returncode == 0, returncode=proc.returncode,
                              stdout=stdout, stderr=stderr)
            except subprocess.TimeoutExpired:
                # The worker owns an emulator child. Retire only this process
                # tree, never an interactive emulator running on another pipe.
                subprocess.run(['taskkill', '/PID', str(proc.pid), '/T', '/F'],
                               capture_output=True, timeout=5)
                proc.communicate(timeout=5)
                result = dict(name=name, passed=False, returncode=None, error='child exceeded 15 seconds')
            result['seconds'] = time.monotonic()-start
            results.append(result)
            print(('PASS ' if result['passed'] else 'FAIL ') + name, flush=True)
            if not result['passed']:
                print(result.get('stdout', '') + result.get('stderr', '') + result.get('error', ''))
    if a.out:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(json.dumps(dict(exe=str(a.exe), shim=str(a.shim), results=results), indent=2))
    passed = sum(r['passed'] for r in results)
    failed = len(results)-passed
    print(f'RESULTS: {passed} passed, {failed} failed')
    print('STATUS: ' + ('FAIL' if failed else 'PASS'))
    return int(bool(failed))


if __name__ == '__main__':
    raise SystemExit(main())
