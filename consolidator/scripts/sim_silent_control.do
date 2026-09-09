# Full-board all-leg-silent control-arbitration test.
# Legal stimulus is FT600Q USB only: links are enabled, but no Tail telemetry
# configuration is issued.  No DUT or inter-FPGA signal is forced.

set USE_PLL_STUB 1
set DEFINES "+define+RUN_SILENT_CONTROL+USE_PLL_STUB"
if {!$USE_PLL_STUB} { set DEFINES "+define+RUN_SILENT_CONTROL" }

set REPO_ROOT  "C:/cortisci/IONM-A/IONM-A-FPGA"
# Benches, models and tasks live in the fpga-test submodule (mirrors the
# design repo layout).  RTL stays under REPO_ROOT.
set TEST_ROOT  "$REPO_ROOT/fpga-test"
set PROJ_ROOT  "$REPO_ROOT/consolidator_v2"
set SCRIPT_DIR "$TEST_ROOT/consolidator/scripts"
set SIM_DIR    "$TEST_ROOT/consolidator"
set RTL_CON    "$PROJ_ROOT/src/rtl"
set SIM_MODELS "$TEST_ROOT/models"
set SIM_TASKS  "$TEST_ROOT/tasks"
set RTL_TAIL   "$REPO_ROOT/tail_fpga_small/src/rtl"

cd "$SIM_DIR"
catch {quit -sim}
catch {vdel -lib work -all}
if {[file exists work]} { file delete -force work }
vlib work
vmap work work

foreach f {bb_stub.v efb_stub.v gsr_stub.v oddrx1f_stub.v} {
    vlog -work work $DEFINES "$SIM_MODELS/$f"
}
foreach f {user_spi_slave.v simple_ping_decoder.v spi_slave_small.v spi_passthrough.v asic_stream_tx.v asic_self_test.v tail_fpga_small.v} {
    vlog -work work $DEFINES "$RTL_TAIL/$f"
}
if {$USE_PLL_STUB} {
    vlog -work work $DEFINES "$SIM_MODELS/pll_stub.v"
} else {
    vlog -work work $DEFINES "$RTL_CON/pll_48m.v"
}
set _fh [open "$RTL_CON/rtl_files.f" r]
while {[gets $_fh _line] >= 0} {
    set _line [string trim $_line]
    if {$_line eq "" || [string match "//*" $_line] || [string match "#*" $_line]} continue
    vlog -work work $DEFINES "$RTL_CON/$_line"
}
close $_fh
foreach f {checker_tasks.sv usb_cmd_tasks.sv spi_tasks.sv} {
    vlog -work work -sv $DEFINES "$SIM_TASKS/$f"
}
foreach f {ft600q_tlm.sv ucsd_asic_model.sv ads122c14_i2c_model.sv spi_master_bfm.sv} {
    vlog -work work -sv $DEFINES "$SIM_MODELS/$f"
}
vlog -work work -sv $DEFINES "$SIM_DIR/tb_top.sv"

vsim -t 1ns -suppress 12110 -voptargs="+acc" -lib work -l "$SCRIPT_DIR/sim_silent_control.log" tb_top
run -all
