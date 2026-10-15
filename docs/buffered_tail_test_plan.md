# Current FPGA verification

History & versioning is for version control. Test the current implementation;
port useful fault scenarios when replacing an architecture, then remove obsolete
DUT configurations and their documentation.

The design repository's `tools/mcp_fpga_tools/simulation.py` owns the executable
inventory. Board scenarios instantiate the current consolidator and four tail
cores with complete-bank pull transport. Unit tests isolate public boundaries.

Coverage includes capture ownership/overflow/reset, both impedance parities,
2500-sweep/s throughput, bad parity/header recovery, source gaps, watchdog and
clock bootstrap, local ASIC routing/abort, repeated acquisition, delayed links,
dead/paused legs, USB backpressure/jitter, packetization, CRC and timestamp
coherency. Bench scoreboards judge payload identity and flags independently.

Run `python tools/ci/run_suite.py --suite rtl --jobs 4 --fail-on-any-failure`
from the design checkout. `--simulator mixed` accelerates supported board benches
with cached Verilator executables; scenarios still execute on every run. Icarus
retains four-state smoke and unit checks. `--suite quick` is a subset, while the
default full suite includes separately built host emulators. Logs and JSON go
under the requested output directory. Never count skipped targets as passes.

No simulator establishes pin electrical timing, physical fault response or
clinical safety. Routed timing and identified-image hardware tests are separate.
