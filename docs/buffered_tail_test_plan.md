# Rev 2 FPGA verification plan

Draft, 2026-10-14. Branch `rev2`, starting test revision `d4c86b0`. These are
planned tests, not registered targets, simulation results or hardware evidence.
The design branch owns the [architecture proposal](../../docs/design-arch-details/buffered_tail_architecture.md)
when this repository is mounted at `IONM-A-FPGA/fpga-test`.

## Verification boundaries

Drive the ASIC source pins, command pins and host USB interface; check the emitted
samples, metadata and physical-control outputs. Prefer complete tail-to-consolidator
paths with real RTL over tests that recreate its internal algorithm. Keep the
existing small-tail and consolidator regression: Rev 2 tests are additive, and
passing the old protocol suite does not validate a new buffered protocol.

Use an independent expected identity per `{epoch, acquisition sweep, leg, lane,
pixel}`. A sequence skip is allowed only where the chosen loss policy explicitly
requires one and reports it. Never accept duplicates, reordered samples, mixed
sweeps, silently substituted zeros or loss reported as successful acquisition.
Randomize initial clock phase and use fixed, logged seeds for bounded variations.

## Planned targets

Names below are reserved proposals. Register them in the design repository runner
only alongside the corresponding RTL and executable bench. Each result must use
the existing RESULTS/STATUS contract and fail on missing completion or assertions.

| Proposed target | Step | Stimulus | Required observation |
| --- | --- | --- | --- |
| `buffered_tail_capture_identity` | R2-02 | Distinct signed samples on all 16 lanes over complete sweeps; frame at several bit offsets | Every captured sample has correct lane/pixel/sweep identity; partial startup sweep excluded |
| `tail_buffer_ownership` | R2-02 | Fill both banks; read slowly; start a third sweep; stop at every burst boundary | No overwrite of a bank being read; whole incoming sweep dropped when full; exact overrun count; oldest complete data retained |
| `buffered_tail_buffer_cdc` | R2-02 | Independent clocks, phase offsets, wraps, reset of each domain and clock pauses | Coherent payload/metadata ownership; no phantom full/empty release or stale epoch after reset |
| `tail_local_controls` | R2-03 | Toggle each control and combinations with unrelated bits set; reset mid-command | Independent amplifier/test controls; preserved bits; documented physical levels and atomic CDC; safe reset values |
| `tail_local_watchdog` | R2-03 | Valid pet, truncated/invalid pet, no pet, stopped SPI, stalled ASIC clock, repeated expiry | Expiry independent of SPI/ASIC activity; output safe-state bound; retained cause; deterministic restart; link traffic alone cannot mask missing host health |
| `buffered_capability_mismatch` | R2-03 | Current small-tail, supported Rev 2, unknown versions and mixed legs | Explicit compatibility decision before run; no accidental legacy opcode execution |
| `buffered_tail_burst_protocol` | R2-04 | Empty/busy reply, minimum/maximum/oversized length, interrupted read, retry, sequence wrap | No OOB memory access; ownership changes only at defined commit; bounded errors; retry does not duplicate published data |
| `buffered_board_stream_identity` | R2-04 | Four real tails, independent skew and source identity, host stalls | Correct USB reconstruction and capture association at 2,500 samples/s/channel; no false validity on absent data |
| `buffered_board_pause_recovery` | R2-04 | Pause below/at/above free buffer capacity at multiple word positions | Loss-free within demonstrated capacity; explicit whole-sweep drops beyond it; return to clean aligned acquisition |
| `buffered_board_dead_leg` | R2-04 | Remove one leg mid-burst, halt source clock, reset it, rejoin with new epoch | Healthy legs make bounded progress; missing leg marked invalid; no cross-leg epoch mixing or indefinite wait |
| `buffered_board_restart` | R2-04 | Start/stop/reset during capture, partial command and host backpressure | No stale sample publication; commands complete/fail within bound; repeatable next start |
| `buffered_tail_asic_transactions` | R2-05 | Both chip selects, supported lengths/rates, delayed completion, stuck busy and abort | Correct ASIC payload/readback; bounded status; no interference with buffer ownership |
| `buffered_impedance_transition` | R2-05 | Baseline telemetry to impedance and back, even/odd lanes, retries and repeated pixel selections | Correct T0/T1 and sensor identity; explicit data-valid boundary; dwell excludes stale/pre-switch samples; stop always bounded |
| `buffered_tail_adc` | R2-06 | Known positive/negative/extreme conversion values, DRDY, NACK and stuck bus | Correct agreed width/sign/scaling; valid/age flags; bounded timeout and recovery; no stale result presented as fresh |
| `buffered_tail_i2c_arbitration` | R2-06 | Concurrent DRDY auto-read and diagnostic command | One bus owner, bounded service, no mixed conversions; disabled/unsupported host access fails explicitly |
| `buffered_fault_accounting` | R2-07 | Overruns, invalid lengths, dead legs, resets and USB admission loss | Per-source counters exact and saturating/wrapping as specified; atomic snapshots; event survives until defined acknowledgement |

If tail transpose is selected, add an identity bench that checks sample ordering
against an independent ASIC model, including negative values and impedance packing;
update host compatibility tests for the new payload version. If tail-link CRC is
separately authorized, add single/multiple-bit corruption, truncation and late-error
tests proving invalid data cannot be published as valid. Neither is silently added
to the currently agreed implementation scope.

## Fast regression policy

Target **20 seconds or less per new test** on the reference CI runner, reporting
compile and simulation time separately as well as combined wall time. This is a
design goal until measured, not a current pass claim. Use short deterministic
scenarios and a small documented seed matrix. Exercise all lane/pixel identities
over a few sweeps rather than running a long burn campaign to find mapping errors.

Shrink buffer depth only for additional ownership corner tests; retain tests at
production depth for inference-sensitive behavior, overflow capacity and identity.
For watchdog logic, accelerated counters can explore state transitions, but retain
a production-parameter expiry check and clock-tolerance calculation. A test with a
shortened deadline cannot establish the 100 ms requirement by itself.

Keep bounded malformed-command/fuzz regressions in the regular suite when they
exercise a defined failure/recovery path. Record seed, delivered transactions and
minimal reproducer. Longer randomized campaigns, burn-in and physical fault
injection run separately and retain full logs; they do not replace quick regressions.
Never turn a timeout into a skip or increase it to hide a hung protocol state.

## Preventing false passes

1. Run each regression against the corresponding known-bad revision when one exists.
   Preserve the failure log, fix revision and passing retest. Do not claim new-feature
   tests reproduced an old bug if the old image simply lacks the interface.
2. For new features, demonstrate test sensitivity with controlled mutations such as
   swapping two lanes, releasing a bank early, removing the watchdog action or hiding
   the drop counter. A compile failure is not a behavioral failure reproduction.
3. Assert exact expected outputs and no unexpected outputs. Include per-test
   cycle/time bounds, nonzero traffic and observed recovery, not just zero errors.
4. Check capture rate using source-clock elapsed time over multiple sweeps, separate
   from burst delivery rate. Verify queued samples sent rapidly after a pause do not
   get counted as newly acquired 4,000-sample/s data.
5. Exercise active masks and unavailable LEG7/other legs explicitly. A physical run
   with a disconnected leg is not a four-leg pass.

## Integration and release evidence

The design repo owns `rtl_files.f`, target registration and image builds. This repo
owns benches, pin-level ASIC/ADC models, deterministic scenarios and expected results.
Commit both repos on `rev2`, then record the exact test commit in the design submodule
pointer. Do not silently point old targets at incompatible Rev 2 modules; keep a
small-tail baseline configuration and explicit new configuration.

At each implementation step, save exact design/test hashes, part/package/speed grade,
tool versions, clock/constraint files, test command, seeds, assertion counts, duration
and logs. Attach per-device LUT/FF/EBR and routed setup/hold reports. Report net area
delta, including added command/descriptor receivers, before claiming the consolidator
has gained headroom. Synthesis success alone is not fit/timing signoff.

Physical qualification additionally needs the fitted-part/BOM match, actual ASIC
frame shape, normal/impedance lane timing, four-leg sustained capture, buffer pause
boundaries, independent clock-loss/watchdog safe-state timing, ADC calibration and
host/USB recovery. The device-to-host Pi substitute cannot establish real-tail or
real-consolidator robustness. Previously deferred instrumentation work remains open.

## Current status

R2-01 through R2-08 are planned. No new RTL targets are registered by this document;
no Rev 2 simulation, fit, timing or hardware pass is claimed. Initial implementation
requires the as-built parts/pinout and the buffered-link contract described in the
architecture proposal.

## Implemented component tests

Registered in the design runner: `tail_buffer_ownership`, `tail_local_controls`,
and `tail_local_watchdog`. These cover production-depth buffer ownership with
asynchronous clocks, partial/full-buffer loss and restart, independent outputs,
watchdog command priority and the exact production timeout cycle. They pass
18,450 assertions. Three deliberate defects (occupied-bank overwrite, one-cycle
late timeout and coupled amplifier/test controls) each fail the corresponding
bench. Integration and board-level acceptance remain open.

The `consolidator_connected_legs` target compiles the common end-to-end smoke
bench with only the PCB's connected interfaces exposed. The same bench remains
compatible with the original top-level ports. Both configurations pass 16
assertions each. Together with eight transport, ownership and watchdog targets,
the board-porting regression passes 26,253 assertions across ten targets.
This does not establish routed timing or completion of the buffered protocol.
