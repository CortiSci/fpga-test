# ============================================================
# QuestaSim Regression: V2 spi_master + Production Tail FPGA SPI Ping
# Location : consolidator_v2/scripts/test_spi_ping_prod_tail.do
# Invoked from QuestaSim GUI:
#   do "C:/.../consolidator_v2/scripts/test_spi_ping_prod_tail.do"
#   run -all
#
# Purpose:
#   Regression-verify that V2 spi_master timing/functionality changes
#   produce correct PING (0xAA) → PING_ACK (0x55) behaviour when driving
#   the REAL production Tail FPGA RTL (record_tail_fpga.v / spi_cmd_decoder.v /
#   user_spi_slave.v) rather than the simplified tail_fpga_small stub.
#
#   Only SP-01..SP-04 run (+define+RUN_SPI_PING).  Full test suite
#   (telemetry, ASIC acquisition) is not exercised because:
#     - tb_top may not generate MCLK_20_48M (sys_clk domain dormant)
#     - ASIC BFM provides no live data in ping-only mode
#   The SPI slave path (user_spi_slave → spi_cmd_decoder) operates entirely
#   in the FPGA_SPI_SCLK domain and is independent of sys_clk.
#
# frame_counter conflict resolved via frame_counter_combined.v:
#   Both consolidator_v2 and record_tail_fpga define module "frame_counter"
#   with different port lists.  frame_counter_combined.v declares ALL ports
#   from both versions (names are disjoint) and implements both behaviours
#   in separate always blocks.  The dormant block's clock (unconnected → Z)
#   never generates posedge events, so only the connected block runs.
#   Do NOT compile either original frame_counter.v in this script.
#
# prod_tail_wrap.v exposes record_tail_fpga under the name tail_fpga_small
#   so tb_top.sv instantiation is unchanged.
# ============================================================

set USE_PLL_STUB 1

# Build DEFINES with separate +define+ switches (avoids vlog-13288 warning).
set DEFINES "+define+RUN_SPI_PING"
if {$USE_PLL_STUB} { append DEFINES " +define+USE_PLL_STUB" }

# ---- Path declarations (8.3 short-names to remove OneDrive spaces) ----------
set PROJ_ROOT    "C:/Users/geoff/OneDrive/DOCUME~1/python/FDA510~1/FPGADE~1/IONM-A~1/consolidator_v2"
set SCRIPT_DIR   "$TEST_ROOT/consolidator/scripts"
set SIM_DIR      "$PROJ_ROOT/src/sim"
set RTL_CON      "$PROJ_ROOT/src/rtl"
set V2_MODELS    "$SIM_DIR/models"

set PROD_CON_SIM "C:/Users/geoff/OneDrive/DOCUME~1/python/FDA510~1/FPGADE~1/IONM-A~1/CONSOL~1/src/sim"
set SIM_MODELS   "$PROD_CON_SIM/models"
set SIM_TASKS    "$PROD_CON_SIM/tasks"

# "Tail FPGA" has a space — resolve to 8.3 short-name
set RTL_TAIL [file normalize [file attributes [file normalize "$PROJ_ROOT/../Tail FPGA/src/rtl"] -shortname]]

puts "PROJ_ROOT : $PROJ_ROOT"
puts "RTL_TAIL  : $RTL_TAIL"
puts "RTL_CON   : $RTL_CON"

cd "$SIM_DIR"

# ---- Fresh work library ------------------------------------------------------
catch {quit -sim}
catch {vdel -lib work -all}
if {[file exists work]} { file delete -force work }
vlib work
vmap work work

# ---- Primitive stubs ---------------------------------------------------------
vlog -work work          $DEFINES "$SIM_MODELS/bb_stub.v"
vlog -work work          $DEFINES "$SIM_MODELS/efb_stub.v"
vlog -work work          $DEFINES "$SIM_MODELS/gsr_stub.v"
vlog -work work          $DEFINES "$SIM_MODELS/oddrx1f_stub.v"

if {$USE_PLL_STUB} {
    vlog -work work      $DEFINES "$SIM_MODELS/pll_stub.v"
}

# ---- Combined frame_counter stub (replaces BOTH RTL frame_counter.v files) ---
# Compiling either production Tail or V2 frame_counter.v here would collide
# (same module name, different port lists).  The combined stub handles both.
vlog -work work          $DEFINES "$V2_MODELS/frame_counter_combined.v"

# ---- Production Tail FPGA RTL -----------------------------------------------
# Compile order matches production Consolidator sim.do §5 (dependencies first).
# frame_counter.v is intentionally omitted — handled by frame_counter_combined.v above.
vlog -work work $DEFINES "$RTL_TAIL/async_cdc_fifo.v"
vlog -work work $DEFINES "$RTL_TAIL/ctrl_output_regs.v"
vlog -work work $DEFINES "$RTL_TAIL/asic_readout_if.v"
vlog -work work $DEFINES "$RTL_TAIL/asic_spi_master.v"
vlog -work work $DEFINES "$RTL_TAIL/asic_spi_ctrl.v"
vlog -work work $DEFINES "$RTL_TAIL/watchdog.v"
vlog -work work $DEFINES "$RTL_TAIL/user_spi_slave.v"
vlog -work work $DEFINES "$RTL_TAIL/spi_cmd_decoder.v"
vlog -work work $DEFINES "$RTL_TAIL/drdy_monitor.v"
vlog -work work $DEFINES "$RTL_TAIL/record_tail_fpga.v"

# Wrapper: exposes record_tail_fpga as tail_fpga_small (tb_top instantiation unchanged)
vlog -work work $DEFINES "$V2_MODELS/prod_tail_wrap.v"

# ---- V2 Consolidator RTL — read from rtl_files.f (single source of truth) ----
# frame_counter.v is intentionally SKIPPED — handled by frame_counter_combined.v above
# (both V2 and production Tail define module "frame_counter" with different port lists;
# the combined stub resolves the collision without two conflicting definitions).
# To add a new RTL module: edit src/rtl/rtl_files.f only.
if {!$USE_PLL_STUB} {
    vlog -work work $DEFINES "$RTL_CON/pll_48m.v"
}

set _fh [open "$RTL_CON/rtl_files.f" r]
while {[gets $_fh _line] >= 0} {
    set _line [string trim $_line]
    if {$_line eq "" || [string match "//*" $_line] || [string match "#*" $_line]} continue
    if {$_line eq "frame_counter.v"} continue  ;# omit — frame_counter_combined.v above
    vlog -work work $DEFINES "$RTL_CON/$_line"
}
close $_fh
unset _fh _line

# ---- Testbench (identical to sim_spi_ping.do — no modifications needed) ------
vlog -work work -sv $DEFINES "$SIM_TASKS/checker_tasks.sv"
vlog -work work -sv $DEFINES "$SIM_TASKS/usb_cmd_tasks.sv"
vlog -work work -sv $DEFINES "$SIM_TASKS/spi_tasks.sv"
vlog -work work -sv $DEFINES "$SIM_MODELS/ft600q_tlm.sv"
vlog -work work -sv $DEFINES "$SIM_MODELS/ucsd_asic_model.sv"
vlog -work work -sv $DEFINES "$SIM_MODELS/ads122c14_i2c_model.sv"
vlog -work work -sv $DEFINES "$SIM_MODELS/spi_master_bfm.sv"
vlog -work work -sv $DEFINES "$SIM_DIR/tb_top.sv"

vsim -t 1ns -suppress 12110 -voptargs="+acc" -lib work \
     -l "$SCRIPT_DIR/test_spi_ping_prod_tail.log" tb_top

if {[file exists "$SCRIPT_DIR/wave.do"]} {
    do "$SCRIPT_DIR/wave.do"
}

run -all
