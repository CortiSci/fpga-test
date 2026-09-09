# Explicit Tail-unit BIST entry point.
# Scope: one Tail and its built-in BIST source; no Consolidator or USB model.

set REPO_ROOT  "C:/cortisci/IONM-A/IONM-A-FPGA"
# Benches, models and tasks live in the fpga-test submodule (mirrors the
# design repo layout).  RTL stays under REPO_ROOT.
set TEST_ROOT  "$REPO_ROOT/fpga-test"
set TAIL_ROOT  "$REPO_ROOT/tail_fpga_small"
set RTL_DIR    "$TAIL_ROOT/src/rtl"
set SIM_DIR    "$TEST_ROOT/tail_fpga_small/src/sim"
set SCRIPT_DIR "$TAIL_ROOT/scripts"
set SIM_MODELS "$TEST_ROOT/common/src/sim/models"

catch {quit -sim}
cd "$SIM_DIR"
catch {vdel -lib work -all}
if {[file exists work]} { file delete -force work }
vlib work
vmap work work

vlog -work work "$SIM_MODELS/gsr_stub.v"
vlog -work work "$SIM_MODELS/oddrx1f_stub.v"
vlog -work work "$RTL_DIR/user_spi_slave.v"
vlog -work work "$RTL_DIR/simple_ping_decoder.v"
vlog -work work "$RTL_DIR/spi_slave_small.v"
vlog -work work "$RTL_DIR/spi_passthrough.v"
vlog -work work "$RTL_DIR/asic_stream_tx.v"
vlog -work work "$RTL_DIR/asic_self_test.v"
vlog -work work "$RTL_DIR/tail_fpga_small.v"
vlog -work work -sv "$SIM_DIR/tb_bist.sv"
vlog -work work -sv "$SIM_DIR/tb_tail_unit.sv"

vsim -t 100ps -voptargs="+acc" -suppress 12110 -lib work \
     -l "$SCRIPT_DIR/sim_tail_unit.log" tb_tail_unit
run -all
