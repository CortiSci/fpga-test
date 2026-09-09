# ============================================================
# QuestaSim Simulation Script — Consolidator V2
# Location : consolidator_v2/scripts/sim.do
# Invoked from QuestaSim GUI:
#   do "C:/cortisci/IONM-A/IONM-A-FPGA/consolidator_v2/scripts/sim.do"
#   run -all
#
# Path strategy: all paths use the C:/cortisci junction which points
# to the corporate OneDrive root and contains no spaces.  No 8.3
# short-names are needed for any path in this script.
# One-time junction setup (cmd.exe, no admin):
#   mklink /J C:\cortisci "D:\Users\<you>\OneDrive - cortisci.com"
# See docs/simulation_standards.md for the simulation standards guide.
# ============================================================

set USE_PLL_STUB 1

set DEFINES ""
if {$USE_PLL_STUB} { append DEFINES "+define+USE_PLL_STUB" }

# ============================================================
# 1. Declare paths.
#    REPO_ROOT : IONM-A-FPGA repo root via C:/cortisci junction
#    PROJ_ROOT : consolidator_v2 directory
#    SIM_DIR   : consolidator_v2 testbench sources
#    SIM_MODELS: shared models — models (no spaces)
#    SIM_TASKS : shared tasks  — tasks  (no spaces)
#    RTL_TAIL  : tail_fpga_small RTL (no spaces via REPO_ROOT)
# ============================================================
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
puts "SIM_TASKS    : $SIM_TASKS"

# ============================================================
# 2. CWD → src/sim/ so the work library lands there
# ============================================================
cd "$SIM_DIR"

# ============================================================
# 3. Create work library
# ============================================================
catch {quit -sim}
catch {vdel -lib work -all}
if {[file exists work]} { file delete -force work }
vlib work
vmap work work

# ============================================================
# 4. Primitive stubs — shared from production models dir
# ============================================================
vlog -work work          $DEFINES "$SIM_MODELS/bb_stub.v"
vlog -work work          $DEFINES "$SIM_MODELS/efb_stub.v"
vlog -work work          $DEFINES "$SIM_MODELS/gsr_stub.v"
vlog -work work          $DEFINES "$SIM_MODELS/oddrx1f_stub.v"

if {$USE_PLL_STUB} {
    vlog -work work      $DEFINES "$SIM_MODELS/pll_stub.v"
}

# ============================================================
# 5. Tail FPGA RTL (tail_fpga_small)
#    Compile leaves first, then top.
# ============================================================
vlog -work work $DEFINES "$RTL_TAIL/user_spi_slave.v"
vlog -work work $DEFINES "$RTL_TAIL/simple_ping_decoder.v"
vlog -work work $DEFINES "$RTL_TAIL/spi_slave_small.v"
vlog -work work $DEFINES "$RTL_TAIL/spi_passthrough.v"
vlog -work work $DEFINES "$RTL_TAIL/asic_stream_tx.v"
vlog -work work $DEFINES "$RTL_TAIL/asic_self_test.v"
vlog -work work $DEFINES "$RTL_TAIL/tail_fpga_small.v"

# ============================================================
# 6. Consolidator V2 RTL — read from rtl_files.f (single source of truth).
#    pll_48m.v is excluded from rtl_files.f; substitute stub when USE_PLL_STUB.
#    To add a new module: edit src/rtl/rtl_files.f only.
# ============================================================
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

# ============================================================
# 7. Testbench packages, models, testbench
#    Packages first so checker_pkg / spi_tasks_pkg / usb_cmd_pkg
#    resolve before any module that imports them.
# ============================================================
vlog -work work -sv $DEFINES "$SIM_TASKS/checker_tasks.sv"
vlog -work work -sv $DEFINES "$SIM_TASKS/usb_cmd_tasks.sv"
vlog -work work -sv $DEFINES "$SIM_TASKS/spi_tasks.sv"
vlog -work work -sv $DEFINES "$SIM_MODELS/ft600q_tlm.sv"
vlog -work work -sv $DEFINES "$SIM_MODELS/ucsd_asic_model.sv"
vlog -work work -sv $DEFINES "$SIM_MODELS/ads122c14_i2c_model.sv"
vlog -work work -sv $DEFINES "$SIM_MODELS/spi_master_bfm.sv"
vlog -work work -sv $DEFINES "$SIM_DIR/tb_top.sv"

# ============================================================
# 8. Simulate
# ============================================================
vsim -t 1ns -suppress 12110 -voptargs="+acc" -lib work -l "$SCRIPT_DIR/sim.log" tb_top

if {[file exists "$SCRIPT_DIR/wave.do"]} {
    do "$SCRIPT_DIR/wave.do"
}

run -all
