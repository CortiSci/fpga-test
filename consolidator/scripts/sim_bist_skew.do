# Deterministic four-Tail BIST phase-skew mode.  The underlying runner retains
# the same FT600Q-only stimulus and adds RUN_BIST_SKEW result assertions.
set BIST_SKEW 1
do "C:/cortisci/IONM-A/IONM-A-FPGA/consolidator_v2/scripts/sim_bist_system.do"
