# fpga-test

Testbenches, behavioural models and simulation scripts for the IONM-A FPGA
designs. Mounted as a submodule of `IONM-A-FPGA`, which holds the RTL.

The split exists so a bench can be run against **any revision of the design**
without disturbing either checkout — the thing that was painful when both lived
in one repo.

## The contract with the design repo

Two roots, resolved separately by the runner (`tools/mcp_fpga_tools/simulation.py`
in the design repo):

| root | owns | override |
|---|---|---|
| `DUT_ROOT` | the design: any path under a `<target>/src/rtl/` directory | `IONM_DUT_ROOT` |
| `TEST_ROOT` | this repo: benches, models, stubs, task packages, `.do` scripts | `IONM_TEST_ROOT` |

Source paths in target definitions stay written **repo-relative** — e.g.
`consolidator/tb_board.sv` and
`consolidator_v2/src/rtl/spi_ch_stream.v` — and are routed to the owning root
by `_is_dut_path()`. The layout here is FLAT and version-neutral — `consolidator/`, `tail/`,
`models/`, `tasks/` — not `consolidator_v2/src/sim/`.  The benches are meant to
run against every implementation of a module regardless of its version, so the
repo is named after the module, not the revision.  The runner maps its
repo-relative test paths onto this layout.

### The manifest travels with the design

`rtl_files.f` lives in the **design** repo and is the canonical module list.
It must be read from the same revision as the RTL it describes: geoff-green
compiles `small_fifo.v` and `watchdog_con.v`, con_phase does not. Expanding a
target's RTL list from one revision while taking sources from another silently
drops or adds modules, and the compile then fails for a reason that has nothing
to do with the revision under test. `_rebuild_dut_sources()` handles this; do
not re-introduce a copy of the module list here.

## Running a bench against another branch

```python
run_simulation("consolidator_board_unique", rtl_ref="72606e4")
```

The RTL is materialised from that ref; the benches always come from this
working tree. It only works where the bench and that revision still share an
interface — where they do not the compile fails, and **that is the result**.

Measured example (2026-09-09), same bench, same tail RTL, consolidator swapped:

```
con_phase (adopted)     9216 pass / 11 fail
geoff-green 72606e4    11264 pass /  1 fail
```

## Running the suite locally

The design repo carries the runner; this repo carries the benches.  From the
design repo root, in a bash shell:

```bash
export PATH="$PATH:/c/oss-cad-suite/bin"          # APPEND — the suite ships its own python.exe
PYTHONIOENCODING=utf-8 /c/Python314/python.exe tools/ci/run_suite.py --out out/sim_results.json
```

All 19 targets take about 90 seconds.  You get a per-target table,
`sim_results.json` with every target's failure lines, and `out/logs/<target>.log`
with each vvp transcript.  This is byte-for-byte what GitHub CI runs
(`.github/workflows/sim.yml`), so a local run is an exact preview.

Two Windows traps worth knowing, both fixed in the driver but easy to reintroduce
in ad-hoc scripts: `subprocess` with `text=True` decodes vvp output in the
**locale** codec (cp1252), which throws on these benches' UTF-8 glyphs — always
pass `encoding="utf-8"`; and the OSS CAD Suite's `bin/` must be *appended* to
PATH, or its bundled `python.exe` shadows the system interpreter.

Verdict is regression-vs-baseline (`.github/workflows/baseline.json` in the design repo), not
all-green — see that file's `_comment` and the CLAUDE.md section "Test suite".

## SW emulator vs RTL emulator — `emulator/`

`emulator/diff_emulators.py` is a different kind of test: it streams one sample
file through both `ionm_emulator.exe` (C++ model) and `ionm_emu_rtl.exe`
(Verilator RTL), drives both with the same host commands at the named pipe, and
reports where the decoded results differ — structurally, and as the electrode
image a host recovers.  Impedance is exercised mode-based (`TELEM_EN=0x02/0x03`).
`emulator/known_divergences.json` names the understood gaps (CRC-pacing bug in
the RTL, 2-bit `telem_en` missing from the SW tail, …) so only an *unexplained*
difference turns the run red.  See `emulator/README.md`.

## Test levels — and which to prefer

**Boundary tests** drive one protocol edge and observe another, with production
RTL in between. `tb_board.sv` / `test_stream_aced.sv` with `RUN_UNIQUE` set the
ASIC models to a transformed `{frame, sensor, leg}` identity and check
reconstruction at the USB boundary. These are portable across designs — swap
the consolidator and rerun unchanged. **Prefer these.**

The system has three modules with defined protocols between them, and there is
a model for each edge:

| edge | model |
|---|---|
| ASIC ↔ Tail | `models/ucsd_asic_model.sv` (`ro1_clk`, `ro1_sd[15:0]`, `ro1_frame`, SPI) |
| Tail ↔ Consolidator | real RTL both sides in the full-stack benches |
| Consolidator ↔ Host | `models/ft600q_tlm.sv` |

**Unit benches** poke internal signals (`tb_rotation.sv` reads `dut.wcnt`,
`w0_dist`, `resync_arm`). They are useful for isolating a defect but are
**not portable** — they fail to elaborate against a design with a different
interface. Keep them, but do not treat them as the system guard.

## Toolchain constraints

See `docs/toolchain_compat.md` in the design repo (C-01..C-10). Two that bite
often here:

* **C-10** — Icarus requires declarations before first *textual* use. Diamond
  and QuestaSim accept forward references, so RTL verified only through Diamond
  can fail to elaborate under Icarus.
* **C-08** — a polling loop `while (dut.sig !== val) @(posedge clk)` exits
  seeing state[N-1]. Use discard+check for serial-capture tasks.

## Status

The suite is not currently green: both consolidator revisions fail
`consolidator_board_unique`, and geoff's own release receipt records
`single_leg`, `board_unique`, `board_unique_skew` and `tail_reset` as blocking.
Establish a green baseline before treating a failure here as a regression.
