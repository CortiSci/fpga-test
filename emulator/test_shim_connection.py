"""Second-client rejection must not return a stale handle or break the owner."""
import argparse
import ctypes as c
import os
from pathlib import Path
import struct
import subprocess


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--exe', required=True)
    ap.add_argument('--shim', required=True)
    args = ap.parse_args()
    base = f'IONM_CONNECTION_{os.getpid()}'
    os.environ['IONM_PIPE_NAME'] = base
    ft = c.WinDLL(str(Path(args.shim).resolve()))
    proc = subprocess.Popen([str(Path(args.exe).resolve()), '-pipe', base],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        cwd=str(Path(args.exe).resolve().parent), creationflags=subprocess.CREATE_NO_WINDOW)
    first, second = c.c_void_p(), c.c_void_p(1234)
    second_opened = False
    failures = 0
    def check(name, ok):
        nonlocal failures
        print(('PASS ' if ok else 'FAIL ') + name, flush=True)
        failures += not ok
    def command():
        data = (c.c_ubyte*8).from_buffer_copy(struct.pack('<4H', 0xaa55, 0, 0, 0))
        count = c.c_ulong()
        ft.FT_SetPipeTimeout(first, 0x82, 200)
        if ft.FT_WritePipe(first, 2, data, 8, c.byref(count), None):
            return False
        return (ft.FT_ReadPipe(first, 0x82, data, 8, c.byref(count), None) == 0
                and count.value == 8 and struct.unpack('<4H', data)[0] == 0x55aa)
    try:
        check('first client opens', ft.FT_Create(None, 0, c.byref(first)) == 0)
        if not first.value:
            return 1
        status = ft.FT_Create(None, 0, c.byref(second))
        second_opened = status == 0
        check('second client rejected with null handle', status != 0 and second.value is None)
        check('owner still receives responses', command())
        ft.FT_Close(first)
        first = c.c_void_p()
        check('reconnect after close', ft.FT_Create(None, 0, c.byref(first)) == 0)
        check('reconnected client receives responses', bool(first.value) and command())
    finally:
        if first.value:
            ft.FT_Close(first)
        if second_opened and second.value:
            ft.FT_Close(second)
        proc.kill()
        proc.wait(timeout=5)
    return int(bool(failures))


if __name__ == '__main__':
    raise SystemExit(main())
