# ============================================================
# QuestaSim Deferred V2 Tests — Consolidator V2
# Location : consolidator_v2/scripts/sim_deferred.do
# Invoked from QuestaSim GUI:
#   do "C:/cortisci/IONM-A/IONM-A-FPGA/consolidator_v2/scripts/sim_deferred.do"
#   run -all
#
# Compiles with +define+RUN_DEFERRED — runs the deferred robustness suite:
#   SA-04-V2 : Leg FIFO overflow detection + FR-02 local_rst recovery
#   UC-03-V2 : Ctrl response deferred to 4105-word frame boundary
#   UC-04-V2 : Fault packet deferred to 4105-word frame boundary
#   FR-03-V2 : PLL lock-loss fault packet + post-recovery register read
#
# 50 ms timeout. force/release on dut_con.pll_locked (FR-03) is an approved
# exception documented in docs/sim_guidelines.md §3.
#
# Path strategy: all paths use the C:/cortisci junction (no spaces).
# See docs/simulation_standards.md for the simulation standards guide.
# ============================================================

set USE_PLL_STUB 1

set DEFINES "+define+RUN_DEFERRED+USE_PLL_STUB"
if {!$USE_PLL_STUB} { set DEFINES "+define+RUN_DEFERRED" }

set REPO_ROOT    "C:/cortisci/IONM-A/IONM-A-FPGA"
# Benches, models and tasks live in the fpga-test submodule (mirrors the
# design repo layout).  RTL stays under REPO_ROOT.
set TEST_ROOT    "$REPO_ROOT/fpga-test"
set PROJ_ROOT    "$REPO_ROOT/consolidator_v2"
set SCRIPT_DIR   "$TEST_ROOT/consolidator/scripts"
set SIM_DIR      "$TEST_ROOT/consolidator"
set RTL_CON      "$PROJ_ROOT/src/rtl"
set SIM_MODELS   "$TEST_ROOT/models"
set SIM_TASKS    "$TEST_ROOT/tasks"
set RTL_TAIL     "$REPO_ROOT/tail_fpga_small/src/rtl"

puts "REPO_ROOT    : $REPO_ROOT"
puts "PROJ_ROOT    : $PROJ_ROOT"
puts "RTL_CON      : $RTL_CON"
puts "RTL_TAIL     : $RTL_TAIL"
puts "SIM_MODELS   : $SIM_MODELS"

cd "$SIM_DIR"

catch {quit -sim}
catch {vdel -lib work -all}
if {[file exists work]} { file delete -force work }
vlib work
vmap work work

# Primitive stubs
vlog -work work          $DEFINES "$SIM_MODELS/bb_stub.v"
vlog -work work          $DEFINES "$SIM_MODELS/efb_stub.v"
vlog -work work          $DEFINES "$SIM_MODELS/gsr_stub.v"
vlog -work work          $DEFINES "$SIM_MODELS/oddrx1f_stub.v"

# Tail FPGA RTL
vlog -work work $DEFINES "$RTL_TAIL/user_spi_slave.v"
vlog -work work $DEFINES "$RTL_TAIL/simple_ping_decoder.v"
vlog -work work $DEFINES "$RTL_TAIL/spi_slave_small.v"
vlog -work work $DEFINES "$RTL_TAIL/spi_passthrough.v"
vlog -work work $DEFINES "$RTL_TAIL/asic_stream_tx.v"
vlog -work work $DEFINES "$RTL_TAIL/asic_self_test.v"
vlog -work work $DEFINES "$RTL_TAIL/tail_fpga_small.v"

# Consolidator V2 RTL — read from rtl_files.f (single source of truth).
# pll_48m.v is excluded from rtl_files.f; substitute stub when USE_PLL_STUB.
# To add a new module: edit src/rtl/rtl_files.f only.
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
unset _fh _line

# Testbench packages, models, top
vlog -work work -sv $DEFINES "$SIM_TASKS/checker_tasks.sv"
vlog -work work -sv $DEFINES "$SIM_TASKS/usb_cmd_tasks.sv"
vlog -work work -sv $DEFINES "$SIM_TASKS/spi_tasks.sv"
vlog -work work -sv $DEFINES "$SIM_MODELS/ft600q_tlm.sv"
vlog -work work -sv $DEFINES "$SIM_MODELS/ucsd_asic_model.sv"
vlog -work work -sv $DEFINES "$SIM_MODELS/ads122c14_i2c_model.sv"
vlog -work work -sv $DEFINES "$SIM_MODELS/spi_master_bfm.sv"
vlog -work work -sv $DEFINES "$SIM_DIR/tb_top.sv"

vsim -t 1ns -suppress 12110 -voptargs="+acc" -lib work -l "$SCRIPT_DIR/sim_deferred.log" tb_top

if {[file exists "$SCRIPT_DIR/wave.do"]} {
    do "$SCRIPT_DIR/wave.do"
}

run -all
