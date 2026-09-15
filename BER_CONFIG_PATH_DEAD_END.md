# ber-config-path — the config-path loopback BER bench (dead end, 2026-09-14)

`consolidator/tests/test_ber_loopback.sv` (+define+RUN_BER_LOOPBACK, BER-01..06),
`emulator/test_ber.py` (host target `emulator_ber`) and the 1536-bit Pixel-chain
echo in `models/ucsd_asic_model.sv` test the design repo's `ber-config-path`
branch: a PRBS-23 looped from the host through the ASIC's Pixel chain and back.

The bench proved the path bit-exact at a stride of 27 bits per read-mode
transaction, as on hardware. But the config path can only compare ~3.7 kbit/s
(USB-command-bound: ~10 register round trips per 24-bit word), so demonstrating
BER ≤ 1e-7 (3×10⁷ bits) takes hours — impractical at every connect. The startup
BER test on `main` runs over the acquisition stream instead (41 Mbit/s per leg).
See `docs/ber_config_path_dead_end.md` in the design repo. The Pixel-chain echo in
the ASIC model is still the right model of the chip and may be worth carrying
forward on its own.
