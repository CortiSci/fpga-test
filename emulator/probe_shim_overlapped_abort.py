"""Manual reproducer: abort an endpoint with two pending overlapped reads.

Uses a private emulator instance. Not registered in CI. No hardware access.
"""
import argparse
import ctypes as c
import json
import os
from pathlib import Path
import subprocess
import time


class Overlapped(c.Structure):
    _fields_ = [('Internal', c.c_size_t), ('InternalHigh', c.c_size_t),
                ('Pointer', c.c_void_p), ('hEvent', c.c_void_p)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exe', required=True, type=Path)
    parser.add_argument('--shim', required=True, type=Path)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--reads', type=int, choices=(1, 2), default=2)
    args = parser.parse_args()
    base = f'IONM_ABORT_RACE_{os.getpid()}'
    os.environ['IONM_PIPE_NAME'] = base
    os.environ.pop('IONM_FTD_ACCURATE', None)
    ft = c.WinDLL(str(args.shim.resolve()))
    kernel = c.WinDLL('kernel32', use_last_error=True)
    kernel.WaitForSingleObject.argtypes = [c.c_void_p, c.c_ulong]
    kernel.WaitForSingleObject.restype = c.c_ulong
    proc = subprocess.Popen([str(args.exe.resolve()), '-pipe', base],
        cwd=str(args.exe.resolve().parent), stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, creationflags=subprocess.CREATE_NO_WINDOW)
    handle = c.c_void_p()
    reads = []
    try:
        status = ft.FT_Create(None, 0, c.byref(handle))
        if status:
            raise RuntimeError(f'FT_Create={status}')
        ft.FT_SetPipeTimeout(handle, 0x82, 5000)
        for i in range(args.reads):
            ov, buf, count = Overlapped(), (c.c_ubyte*8)(), c.c_ulong()
            if ft.FT_InitializeOverlapped(handle, c.byref(ov)):
                raise RuntimeError('InitializeOverlapped failed')
            status = ft.FT_ReadPipe(handle, 0x82, buf, 8, c.byref(count), c.byref(ov))
            reads.append((ov, buf, count, status))
            # First worker blocks in ReadFile; the second reaches rx_mu.
            time.sleep(0.050)
        before = [kernel.WaitForSingleObject(ov.hEvent, 0) for ov, *_ in reads]
        start = time.perf_counter()
        abort_status = ft.FT_AbortPipe(handle, 0x82)
        waits = [kernel.WaitForSingleObject(ov.hEvent, 250) for ov, *_ in reads]
        result = dict(reads=args.reads, pending_before_abort=before,
                      read_return_statuses=[r[3] for r in reads], abort_status=abort_status,
                      completion_wait_results=waits, elapsed_ms=(time.perf_counter()-start)*1000,
                      passed=all(w == 258 for w in before) and abort_status == 0
                             and all(w == 0 for w in waits))
        args.out.write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)
        return 0 if result['passed'] else 1
    finally:
        # Disconnect our private server before releasing contexts: never free a
        # pending worker's OVERLAPPED or buffer merely because the test timed out.
        proc.kill()
        proc.wait(timeout=5)
        completed = all(kernel.WaitForSingleObject(ov.hEvent, 6000) == 0 for ov, *_ in reads)
        if completed:
            for ov, *_ in reads:
                ft.FT_ReleaseOverlapped(handle, c.byref(ov))
            if handle.value:
                ft.FT_Close(handle)
        else:
            # Retire this isolated probe process without unsafe DLL teardown.
            os._exit(2)


if __name__ == '__main__':
    raise SystemExit(main())
