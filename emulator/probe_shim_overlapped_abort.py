"""Regression helper: abort an endpoint with two pending overlapped reads.

Uses a private emulator instance. Called by test_shim_regressions.py in CI.
No hardware access.
"""
import argparse
import ctypes as c
import json
import os
from pathlib import Path
import subprocess
import struct
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
    parser.add_argument('--mode2', action='store_true')
    parser.add_argument('--delay-ms', type=float, default=50,
                        help='Delay between submissions; zero exercises abort before worker startup')
    args = parser.parse_args()
    base = f'IONM_ABORT_RACE_{os.getpid()}'
    os.environ['IONM_PIPE_NAME'] = base
    os.environ['IONM_FTD_ACCURATE'] = '1' if args.mode2 else '0'
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
            if args.delay_ms:
                time.sleep(args.delay_ms / 1000)
        before = [kernel.WaitForSingleObject(ov.hEvent, 0) for ov, *_ in reads]
        start = time.perf_counter()
        abort_status = ft.FT_AbortPipe(handle, 0x82)
        waits = [kernel.WaitForSingleObject(ov.hEvent, 250) for ov, *_ in reads]
        elapsed_ms = (time.perf_counter()-start)*1000
        cancelled = []
        recovery = False
        if all(w == 0 for w in waits):
            for ov, *_ in reads:
                count = c.c_ulong(99)
                ok = ft.FT_GetOverlappedResult(handle, c.byref(ov), c.byref(count), False)
                cancelled.append(not ok and count.value == 0)
            # A fresh read submitted after abort must still receive the next ACK.
            command = (c.c_ubyte*8).from_buffer_copy(struct.pack('<4H', 0xaa55, 0, 0, 0))
            count = c.c_ulong()
            write_status = ft.FT_WritePipe(handle, 2, command, 8, c.byref(count), None)
            reply = (c.c_ubyte*8)()
            ft.FT_SetPipeTimeout(handle, 0x82, 200)
            read_status = ft.FT_ReadPipe(handle, 0x82, reply, 8, c.byref(count), None)
            recovery = write_status == 0 and read_status == 0 and count.value == 8 and bytes(reply[:2]) == b'\xaa\x55'
        result = dict(reads=args.reads, mode2=args.mode2, delay_ms=args.delay_ms,
                      pending_before_abort=before, cancelled=cancelled, recovery=recovery,
                      read_return_statuses=[r[3] for r in reads], abort_status=abort_status,
                      completion_wait_results=waits, elapsed_ms=elapsed_ms,
                      passed=all(w == 258 for w in before) and abort_status == 0
                             and all(w == 0 for w in waits) and all(cancelled) and recovery)
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
