"""Manual impedance timing probe; not registered in CI. Uses private emulator pipes.
The fixed dwell and single-leg variants adapt the existing run_inject sequence.
No failure of the contractor symptom has been reproduced with this probe.
"""
import argparse, json, os, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import diff_emulators as dx
p=argparse.ArgumentParser()
p.add_argument('--exe',required=True)
p.add_argument('--delay-ms',type=float,default=0)
p.add_argument('--lanes',type=int,default=16)
p.add_argument('--out',required=True)
p.add_argument('--shim')
p.add_argument('--dwell-ms',type=float,default=0)
p.add_argument('--leg',type=int,choices=range(4))
a=p.parse_args()
base=f'IONM_RACE_{os.getpid()}'
init=dx.PipeClient.__init__
dx.PipeClient.__init__=lambda self:init(self,base)
if a.shim:
    import ctypes as c, struct
    from queue import Empty
    os.environ['IONM_PIPE_NAME']=base
    ft=c.WinDLL(str(Path(a.shim).resolve()))
    class Messages:
        def __init__(self, client): self.client=client; self.buf=bytearray()
        def get(self,timeout):
            end=time.monotonic()+timeout
            while True:
                size=8 if len(self.buf)<8 else (dx.TELEM_WORDS*2 if self.buf[:2]==b'\x01\x00' else 8)
                if len(self.buf)>=size:
                    msg=bytes(self.buf[:size]); del self.buf[:size]; return msg
                remaining=end-time.monotonic()
                if remaining<=0: raise Empty()
                ft.FT_SetPipeTimeout(self.client.handle,0x82,max(1,int(remaining*1000)))
                buf=(c.c_ubyte*(size-len(self.buf)))(); count=c.c_ulong()
                status=ft.FT_ReadPipe(self.client.handle,0x82,buf,len(buf),c.byref(count),None)
                self.buf.extend(bytes(buf[:count.value]))
                if status not in (0,19): raise RuntimeError(f'FT_ReadPipe status={status}')
    def connect(self):
        self.handle=c.c_void_p()
        rc=ft.FT_Create(None,0,c.byref(self.handle))
        if rc: raise RuntimeError(f'FT_Create={rc}')
        self.q=Messages(self)
    def send(self,data):
        payload=(c.c_ubyte*(len(data)-2)).from_buffer_copy(data[2:]); count=c.c_ulong()
        rc=ft.FT_WritePipe(self.handle,2,payload,len(payload),c.byref(count),None)
        if rc or count.value!=len(payload): raise RuntimeError(f'FT_WritePipe={rc}')
    def close(self):
        if getattr(self,'handle',None): ft.FT_Close(self.handle); self.handle=None
    dx.PipeClient.connect=connect; dx.PipeClient._send=send; dx.PipeClient.close=close
dx.INJECT_ROWS=list(range(64)); dx.INJECT_LANES=list(range(a.lanes)); dx.INJECT_FRAMES=4
dx.Host.drain=lambda self,quiet_s:0
orig=dx.PipeClient.write_reg
def write(self,addr,data,timeout_s,stray=None):
    if a.leg is not None and addr in (dx.REG_ACQ_ALL_RUN,dx.REG_SPI_ENABLE_MASK) and data: data=1<<a.leg
    orig(self,addr,data,timeout_s,stray)
    if addr==dx.REG_ACQ_ALL_RUN and data:
        time.sleep(a.delay_ms/1000)
dx.PipeClient.write_reg=write
if a.dwell_ms or a.leg is not None:
    import inspect
    source=inspect.getsource(dx.run_inject)
    if a.dwell_ms:
        source=source.replace('while len(payloads) < INJECT_FRAMES:', 'deadline = time.monotonic() + ' + str(a.dwell_ms/1000) + '\n                while time.monotonic() < deadline:')
        source=source.replace('m = pipe.next_frame(tmo)', 'm = pipe.next_frame(max(0.001, deadline-time.monotonic()))')
    if a.leg is not None: source=source.replace('for ch in range(4)',f'for ch in [{a.leg}]')
    exec(source,dx.__dict__)
result, measurements=dx.run_inject('sw',Path(a.exe).resolve(),['-pipe',base],5,5,5,lambda s:None)
failures=[]
for (ch,lane,row),m in measurements.items():
    expected=dx.model_swing_counts(ch,63-row,lane) if hasattr(dx,'model_swing_counts') else 2*round(0.065*dx.model_z_ohms(ch,63-row,lane)/1.0493)
    if m is None or m[0]!=63-row or abs(m[1]-expected)>1:
        failures.append(dict(ch=ch,lane=lane,row=row,actual=m,expected=expected))
expected_count=a.lanes*(64 if a.leg is not None else 256)
report=dict(exe=str(Path(a.exe).resolve()),shim=a.shim,leg=a.leg,dwell_ms=a.dwell_ms,delay_ms=a.delay_ms,measurements=len(measurements),expected_measurements=expected_count,error=result.error,failures=failures,seconds=result.seconds,frames=len(result.frames),bad_crc=sum(not f.crc_ok for f in result.frames))
Path(a.out).write_text(json.dumps(report,indent=2))
print(json.dumps(report))
sys.exit(bool(result.error or failures or len(measurements)!=expected_count or report['bad_crc']))


