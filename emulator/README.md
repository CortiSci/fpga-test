# emulator/ — SW emulator vs RTL emulator differential test

`diff_emulators.py` streams the **same sample file** through both emulators,
drives both with the **same host command sequence**, decodes both with the
**same parser**, and reports where the results differ.  Its job is to verify
that the software emulator (`ionm_emulator.exe`, pure C++) behaves like the RTL
(`ionm_emu_rtl.exe`, Verilator-compiled `consolidator_v2` + 4× `tail_fpga_small`)
— not to verify the RTL itself.

## Why a host program, not an Icarus/Verilator bench

The two emulators share exactly one interface: the host transport that
carries the bytes host software sees through the FTD3XX shim - Windows named
pipes, or `AF_UNIX` sockets under `$IONM_PIPE_DIR` (default `/tmp`) on Linux,
same framing either way (`Software Emulator/emulator/src/ipc_stream.h`).  The SW emulator
is C++, so no simulator can host it; the RTL side is already a Verilator build.
So the comparison is made **at the pipe, by a host**, which is where application
code would notice a difference.  Verilator is used only to build the RTL side:

```bash
# design repo; MSYS2 UCRT64 on Windows, or Linux with verilator on PATH
bash consolidator_v2/verilator/build_rtl_emulator.sh     # -> Software Emulator/build/test_app/Release/ionm_emu_rtl[.exe]
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
PYTHONIOENCODING=utf-8 /c/Python314/python.exe fpga-test/emulator/diff_emulators.py --strict --out out/diff.json
```

Both binaries must be current builds in `Software Emulator/build/test_app/Release/`
(`cmake --build build --config Release` for the SW side).  The whole run is
about a minute; the RTL emulator produces 6 frames in ~1 s in normal mode.

Flags: `--legs normal,imp_even,imp_odd,inject` (default all four), `--frames N`, `--strict` (ignore the known list),
`--sw-only` / `--rtl-only` for smoke-testing one side, `--out report.json`.

The last lines are the same sentinels the SV benches emit
(`RESULTS: N passed, M failed` / `STATUS: PASS|FAIL`), so the result contract
tooling can score it, preceded by a `THROUGHPUT:` line - wall-clock frames/s each
emulator delivered while the host read, per leg and averaged, and the sw/rtl
ratio (the SW model paces itself to real time; the RTL runs at simulation speed).
Also in the JSON report as `throughput` and per side `frames_per_s`.

**On GitHub:** `.github/workflows/emulator.yml` builds both emulators on Ubuntu
(g++/cmake, apt verilator) and runs this test `--strict`; the check summary shows
the 55-check table (42 + the 13 `inject` checks) and the throughput line.  Locally on Windows it is also the
suite host target `emulator_differential`.

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

Impedance is exercised **mode-based**, the way the bringup tool does it: tails
commanded to `TELEM_EN=0x02` (even SD lanes) then `0x03` (odd) over the real
SPI_CFG path.  Both emulators always receive both `-f` and `-if`; in the
impedance legs each side must deliver the **`-if` file** on the selected lanes,
unpacked with the spec's T0/T1 rule (`decode_impedance()`), exactly as the normal
leg must deliver `-f`.  A side showing the wrong file is reported as such
("different FILES: …") rather than as a value mismatch.

### The impedance sweep itself (leg `inject`, 2026-09-11)

The bring-up tool's impedance measurement is not the tail's impedance mode: it is
normal-mode streaming with **one pixel at a time given `EN_IM`** through the
tail's CS passthrough — a Global write selecting the pixel slice (lane), 64
`PIX_OFF` words to clear the 64-row Pixel shift chain, one `PIX_INJECT` word, then
one `PIX_OFF` per row advance — and the ±65 nA square wave at Fs/4 recovered from
the stream as the peak-to-peak of the four `frame_cnt & 3` bin means
(`CannedFunctions::measureCurrent`).  Leg `inject` runs exactly that on lane 5,
chain rows 0/1/3, all four legs, against both emulators, with a `-f` file whose
frames are identical so the peak-to-peak isolates the injection.  Both emulators
share the ASIC pixel model (`Software Emulator/emulator/src/asic_pixel_model.h`:
PSLICE select, per-slice chains committed at the frame boundary, EN_PIXEL → 0x8000,
EN_IM → ±A with A = 65 nA × Z / 1.0493 µV per count and the deterministic
Z = 1000 + 100·(row·16 + lane) + 25·asic Ω), so per pixel the **group** (63 − row)
and the **swing** (2A, e.g. 186 counts for row 0 lane 5 of ASIC 0) must agree between the
sides and with the model; 13 checks.  The RTL side reaches the model through the
tails' real `spi_passthrough.v` and a 24-bit SPI decoder on the exported
`SPI_RO_*` pins; the SW side through `TailFpga`'s CS1/CS2 arming into `AsicBfm`.

## Known divergences

`known_divergences.json` names differences that are understood and accepted for
now — the same idea as the sim suite's `.github/workflows/baseline.json`.  A key
there turns a red check into a `KNOWN` row; `--strict` ignores the file.
**Remove a key when its gap is closed** so the check goes back to guarding.

**State on 2026-09-09 (end of day): the list is EMPTY — 42 checks pass in
`--strict` mode.**  Normal-mode recovered images identical (16384/16384 over 4
source frames); both impedance legs deliver the `-if` file on the selected lanes
from both emulators (2048/2048 each); CRC valid on both; phase 0 on both.

### Closed divergences (the morning's 9 rows, and where each was fixed)

| observed | cause | fix |
|---|---|---|
| RTL CRC invalid on every frame | CRC-pacing bug: `tx_gap` register lagged the FSM's emit by a cycle, so free-running streams emitted word pairs on consecutive cycles | `telem_engine_v3.v`: `tx_ready_g = telem_tx_ready & ~telem_tx_valid` (ported from `edge-fix-test` `3f1d334`). Sim suite 84 → 53 failing assertions. **Note:** `main` did not fit Diamond before this change either (1076/1056); the fix costs +3 SLICEs (1079). |
| SW phases 7/19/41/58, RTL 0 | SW modelled a rotating per-leg phase to exercise host de-rotation; con_phase anchors legs on the w0 marker and reports 0 | `consolidator.cpp`: phase 0 / unrotated by default (RTL truth); `-leg_phase` restores the rotating model for host testing |
| `TELEM_EN=0x02` → SW streams nothing; `0x03` → SW streams normal data | `tail_fpga.cpp` stored `telem_en` as a bool (`& 0x01`) | `tail_fpga.cpp`/`.h`: 2-bit `telem_mode_`; impedance frames = two 512-word sweeps with the spec's T0/T1 packing; `asic_bfm.cpp` seeds `SCFG_I` from the MODE, no longer from whether `-if` is open |
| `-if` given: SW switched a normal stream to impedance data | `asic_bfm.cpp` auto-armed `SCFG_I` whenever an impedance file was open | same as above — the mode decides, the file is just the source |
| RTL ignored `-if` | `RtlFileManager::impedance_value()` had no caller | `sim_main.cpp` (guarded `IONM_V2_HARNESS`): at each ASIC frame start read the tails' `imp_en` (new `sim_top` outputs via XMR) and source the frame from the `-if` file |
| RTL `frame_cnt` stepped ≈62/frame in impedance mode | harness drove a fixed 2.56 MHz `RO1_CLK`; the real ASIC doubles it in impedance mode, so the decimated words came 40 SCLK apart and `spi_ch_stream`'s 12-cycle `TIMEOUT_SHORT` re-hunted after every word | `sim_main.cpp`: `RO1_CLK` doubles (97/98-tick half-periods) while any tail is in impedance mode |
| **new, found by the fixed test:** one RTL leg per run decoded one bit right-shifted in impedance mode | `asic_stream_tx.v` `imp_phase` free-ran from whenever `TELEM_EN` landed; a leg armed on an odd tick paired (2p−1, 2p) | `asic_stream_tx.v`: the frame-start tick (`ro1_frame` high) is always T0. `tb_bist` 112/112; tail fits 107/128 SLICEs, 0 timing errors. Answers `impedance_mode_spec.md` §10(b). |

The last row is the payoff: it is an RTL bug that nothing else caught, visible
only once the two emulators were made to agree on everything around it.
