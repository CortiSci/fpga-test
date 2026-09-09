# emulator/ — SW emulator vs RTL emulator differential test

`diff_emulators.py` streams the **same sample file** through both emulators,
drives both with the **same host command sequence**, decodes both with the
**same parser**, and reports where the results differ.  Its job is to verify
that the software emulator (`ionm_emulator.exe`, pure C++) behaves like the RTL
(`ionm_emu_rtl.exe`, Verilator-compiled `consolidator_v2` + 4× `tail_fpga_small`)
— not to verify the RTL itself.

## Why a host program, not an Icarus/Verilator bench

The two emulators share exactly one interface: the named-pipe transport that
carries the bytes host software sees through the FTD3XX shim.  The SW emulator
is C++, so no simulator can host it; the RTL side is already a Verilator build.
So the comparison is made **at the pipe, by a host**, which is where application
code would notice a difference.  Verilator is used only to build the RTL side:

```bash
# MSYS2 UCRT64 shell (design repo)
cd consolidator_v2/verilator && bash ./build_rtl_emulator.sh     # -> Software Emulator/build/test_app/Release/ionm_emu_rtl.exe
```

The runner speaks the protocol directly (stdlib only, no shim, no pywin32):

```
CTRL (host -> emu):  [u16 LE byte_len][payload]   8-byte command  {0xAA55, flags(1=write), addr, data}
DATA (emu  -> host): [u16 LE byte_len][payload]   8-byte response {0x55AA, flags, addr, rdata}
                                                  8210-byte telemetry frame (4105 words)
```

## Running it

```bash
# from the design repo root, so DUT_ROOT resolves to it (or set IONM_DUT_ROOT)
PYTHONIOENCODING=utf-8 /c/Python314/python.exe fpga-test/emulator/diff_emulators.py --imp-file --out out/diff.json
```

Both binaries must be current builds in `Software Emulator/build/test_app/Release/`
(`cmake --build build --config Release` for the SW side).  The whole run is
about a minute; the RTL emulator produces 6 frames in ~1 s in normal mode.

Flags: `--legs normal,imp_even,imp_odd` (default all three), `--imp-file` adds
the informational `-if` leg, `--frames N`, `--strict` (ignore the known list),
`--sw-only` / `--rtl-only` for smoke-testing one side, `--out report.json`.

The last two lines are the same sentinels the SV benches emit
(`RESULTS: N passed, M failed` / `STATUS: PASS|FAIL`), so the result contract
tooling can score it.

## What is compared — three layers

| layer | what | how it is judged |
|---|---|---|
| **structural** | frame length 4105, tag `0x0001`, counter words, phase-word shape, CRC-32 validity, counter monotonic, phase-reporting style | per side; a DIFF is one side passing a check the other fails |
| **recovered** | the 64×64 electrode image the host recovers from each side after that side's *documented* mapping (contract §7 transpose + de-rotation, then the SW emulator's SubQv3 electrode map) | equal images = the host cannot tell the emulators apart — **this is the test** |
| **raw** | the un-remapped sensor array | informational; shows the remap difference directly |

The sample pattern is invertible — `value = file_frame << 12 | sensor` — so a
decoded frame names the source-file frame it came from, and frames are matched
by content rather than by arrival order (each emulator starts at an arbitrary
point in the looped file).

### Impedance

Impedance is exercised **mode-based**, the way the bringup tool does it: same
file, tails commanded to `TELEM_EN=0x02` (even SD lanes) then `0x03` (odd) over
the real SPI_CFG path.  An `-if`-driven comparison would not compare the two
models — the RTL harness opens `-if` but never reads it
(`RtlFileManager::impedance_value()` has no caller in
`Consolidator FPGA/verilator/sim_main.cpp`).  `--imp-file` still runs that leg,
labelled informational, so the gap stays visible.

## Known divergences

`known_divergences.json` names differences that are understood and accepted for
now — the same idea as the sim suite's `.github/workflows/baseline.json`.  A key
there turns a red check into a `KNOWN` row; `--strict` ignores the file.
**Remove a key when its gap is closed** so the check goes back to guarding.

State on 2026-09-09 (SW `557ab30` source; RTL from `consolidator_v2 @ 0f7c8c0`
con_phase + `tail_fpga_small`): **30 checks pass, 9 known-divergent, 0
unexplained.**

| observed | classification | where |
|---|---|---|
| Recovered electrode images identical, both 4096/4096 vs the file, all 4 source frames | **PASS** — the SW emulator IS the RTL as far as a host can tell, in normal streaming | — |
| RTL CRC invalid on every frame; SW valid | **RTL bug, SW correct** — the unmerged CRC-pacing fix `3f1d334` | `telem_engine_v3.v` |
| SW reports per-leg phases 7/19/41/58; RTL reports 0 | model difference, both contract-legal; images identical after de-rotation | `tail_fpga.cpp` vs `spi_ch_stream.v` |
| `TELEM_EN=0x02`: SW streams nothing, RTL streams even lanes | **SW gap** — `tail_fpga.cpp:59` stores `telem_en` as a bool (`& 0x01`) | CLAUDE.md SW track "2-bit telem_en" |
| `TELEM_EN=0x03`: SW streams normal data, RTL streams T0/T1-interleaved impedance planes | same SW gap | `asic_stream_tx.v` imp_phase |
| `TELEM_EN=0x03`: RTL `frame_cnt` steps ≈62/frame (61,123,186,…) | observed alongside the gap, **not diagnosed** — far more than 2:1 decimation predicts | re-examine when the SW tail models impedance |
| `-if`: SW streams the impedance file (auto-arms `SCFG_I` for all sensors), RTL streams the sample file | harness gap — different *files*, not different models | `asic_bfm.cpp` / `sim_main.cpp` |

When the SW tail grows 2-bit `telem_en` and the T0/T1 packing, delete the three
`imp_*` keys and the impedance legs become real equivalence checks.
