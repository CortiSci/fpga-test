# FPGA tests

This submodule contains current IONM-A RTL benches, protocol models and host
emulator checks. The design repository owns RTL and the executable target
registry in `tools/mcp_fpga_tools/simulation.py`.

History & versioning is for version control. Keep useful behavioral regressions
on the current DUT; remove retired architectures and historical-only benches.

From the parent checkout:

```powershell
python tools/ci/run_suite.py --suite rtl --jobs 4 --fail-on-any-failure
python tools/ci/run_suite.py --suite rtl --simulator mixed --jobs 8 --fail-on-any-failure
```

There are 55 RTL targets and nine separate host targets. The default full suite
includes both. Host tests require rebuilt emulators; missing executables are
reported as skips. The quick profile is a subset for development.

Mixed mode caches compiled Verilator executables for board benches, not test
results. Cold builds take longer; every invocation reruns scenarios. Four-state
smoke and unit benches remain on Icarus. See the
[verification plan](docs/buffered_tail_test_plan.md) for scope and limitations.

Boundary tests prefer actual tail and consolidator RTL, `ucsd_asic_model.sv`
source stimulus and `ft600q_tlm.sv` USB observations. Unit tests isolate public
protocol boundaries. `consolidator/tb_top.sv` is the common board assembly.
Questa GUI launchers under `consolidator/scripts/` select the same registry.

The runner resolves RTL against `IONM_DUT_ROOT` and tests against
`IONM_TEST_ROOT`; defaults are the parent checkout and this submodule.
Git revisions are the source of earlier designs when reproducing a regression.
