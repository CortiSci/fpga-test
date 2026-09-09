# ============================================================
# sim_bist.do — QuestaSim BIST verification for tail_fpga_small
#
# Invoked from QuestaSim GUI:
#   do "C:/cortisci/IONM-A/IONM-A-FPGA/tail_fpga_small/scripts/sim_bist.do"
#   run -all
#
# What this tests:
#   TC-BIST-01  Post-reset state (test_en=0, st_frame=0, st_sd=0)
#   TC-BIST-02  SPI opcode 0x06 sets test_en; BIST runs immediately (no startup delay)
#   TC-BIST-03  SPI opcode 0x02 (data=0x01) sets telem_mode=normal; switch to telemetry-rate SCLK
#   TC-BIST-04  Send arm pulse (MOSI=1 while SS_N=1); verify arm=1
#   TC-BIST-05  Wait for frame_armed (first st_frame rising edge after arm)
#   TC-BIST-06  32 streaming words verified: each is prev+1 (monotonic counter) + parity
#   TC-BIST-07  Disable test_en resets BIST (st_clk=0, st_sd=0, st_frame=0); re-enable restarts
#
# Clock scaling: MCLK=100 MHz, SCLK_FAST=250 MHz (telemetry, ratio 2.5 = 51.2/20.48),
#                SCLK_SLOW=3.906 MHz (SPI commands, proportional to 800 kHz = 51.2/64).
# All asic_self_test and asic_stream_tx timing relationships are preserved.
#
# Path strategy: all paths use the C:/cortisci junction (no spaces).
# See docs/simulation_standards.md for the simulation standards guide.
# ============================================================

# ---- Path declarations -------------------------------------------------------
set REPO_ROOT  "C:/cortisci/IONM-A/IONM-A-FPGA"
# Benches, models and tasks live in the fpga-test submodule (mirrors the
# design repo layout).  RTL stays under REPO_ROOT.
set TEST_ROOT  "$REPO_ROOT/fpga-test"
set TAIL_ROOT  "$REPO_ROOT/tail_fpga_small"
set RTL_DIR    "$TAIL_ROOT/src/rtl"
set SIM_DIR    "$TEST_ROOT/tail_fpga_small/src/sim"
set SCRIPT_DIR "$TAIL_ROOT/scripts"

# Shared stubs and models from common/
set SIM_MODELS "$TEST_ROOT/common/src/sim/models"

puts "TAIL_ROOT  : $TAIL_ROOT"
puts "RTL_DIR    : $RTL_DIR"
puts "SIM_DIR    : $SIM_DIR"
puts "SIM_MODELS : $SIM_MODELS"

# ---- Drop any running simulation before changing directory -------------------
catch {quit -sim}

cd "$SIM_DIR"

# ---- Fresh work library ------------------------------------------------------
catch {vdel -lib work -all}
if {[file exists work]} { file delete -force work }
vlib work
vmap work work

# ---- Primitive stubs from common/src/sim/models/ ----------------------------
# gsr_stub.v  : defines GSR (and PUR); GSR is instantiated in tail_fpga_small.v
# oddrx1f_stub.v : defines ODDRXE; instantiated in spi_passthrough.v
vlog -work work "$SIM_MODELS/gsr_stub.v"
vlog -work work "$SIM_MODELS/oddrx1f_stub.v"

# ---- Tail FPGA RTL (leaves first, then top) ----------------------------------
# user_spi_slave.v and simple_ping_decoder.v are legacy modules still in src/rtl/;
# they are NOT instantiated by tail_fpga_small.v but are compiled to avoid
# "module not found" warnings if any testbench infrastructure references them.
vlog -work work "$RTL_DIR/user_spi_slave.v"
vlog -work work "$RTL_DIR/simple_ping_decoder.v"
vlog -work work "$RTL_DIR/spi_slave_small.v"
vlog -work work "$RTL_DIR/spi_passthrough.v"
vlog -work work "$RTL_DIR/asic_stream_tx.v"
vlog -work work "$RTL_DIR/asic_self_test.v"
vlog -work work "$RTL_DIR/tail_fpga_small.v"

# ---- Testbench ---------------------------------------------------------------
vlog -work work -sv "$SIM_DIR/tb_bist.sv"

# ---- Simulate ----------------------------------------------------------------
# +acc: required for hierarchical signal access (dut.u_self_test.st_frame etc.)
# -suppress 12110: suppresses "Some checker assertions were not checked" (benign)
vsim -t 100ps -voptargs="+acc" -suppress 12110 -lib work \
     -l "$SCRIPT_DIR/sim_bist.log" tb_bist

# ---- Waveform setup ----------------------------------------------------------
# Add the signals most useful for visual debugging in the Wave window.
add wave -divider {Clocks / Reset}
add wave -label "mclk"      sim:/tb_bist/mclk
add wave -label "sclk"      sim:/tb_bist/sclk
add wave -label "sclk_fast" sim:/tb_bist/sclk_fast
add wave -label "sclk_slow" sim:/tb_bist/sclk_slow
add wave -label "sclk_sel"  sim:/tb_bist/sclk_sel
add wave -label "devrst_n"  sim:/tb_bist/devrst_n

add wave -divider {SPI Interface}
add wave -label "MOSI"  sim:/tb_bist/mosi
add wave -label "MISO"  sim:/tb_bist/miso
add wave -label "SS_N"  sim:/tb_bist/ss_n

add wave -divider {spi_slave_small outputs}
add wave -label "test_en"    sim:/tb_bist/dut/test_en
add wave -label "telem_mode"  sim:/tb_bist/dut/telem_mode
add wave -label "imp_phase"   sim:/tb_bist/dut/u_stream_tx/imp_phase
add wave -label "ctrl_reg"    sim:/tb_bist/dut/ctrl_reg

add wave -divider {asic_self_test internals}
add wave -label "div_cnt"    sim:/tb_bist/dut/u_self_test/div_cnt
add wave -label "frame_pos"  -radix unsigned sim:/tb_bist/dut/u_self_test/frame_pos
add wave -label "data_cnt"   -radix unsigned sim:/tb_bist/dut/u_self_test/data_cnt
add wave -label "st_sd"      -radix unsigned sim:/tb_bist/dut/u_self_test/st_sd
add wave -label "st_clk"     sim:/tb_bist/dut/u_self_test/st_clk
add wave -label "st_frame"   sim:/tb_bist/dut/u_self_test/st_frame

add wave -divider {Input mux (after physical pads)}
add wave -label "ro1_clk_i"   sim:/tb_bist/dut/ro1_clk_i
add wave -label "ro1_sd_i"    -radix unsigned sim:/tb_bist/dut/ro1_sd_i
add wave -label "ro1_frame_i" sim:/tb_bist/dut/ro1_frame_i

add wave -divider {asic_stream_tx key state}
add wave -label "arm"          sim:/tb_bist/dut/u_stream_tx/arm
add wave -label "frame_s2"     sim:/tb_bist/dut/u_stream_tx/frame_s2
add wave -label "frame_armed"  sim:/tb_bist/dut/u_stream_tx/frame_armed
add wave -label "ro_edge_seen" sim:/tb_bist/dut/u_stream_tx/ro_edge_seen
add wave -label "tx_state"     sim:/tb_bist/dut/u_stream_tx/tx_state
add wave -label "sd_tx"  -radix unsigned sim:/tb_bist/dut/u_stream_tx/sd_tx
add wave -label "word_cnt"     sim:/tb_bist/dut/u_stream_tx/word_cnt
add wave -label "err_pending"  sim:/tb_bist/dut/u_stream_tx/err_pending
add wave -label "sclk_sel"     sim:/tb_bist/sclk_sel

configure wave -timelineunits ns
configure wave -namecolwidth 160
configure wave -valuecolwidth 80

run -all
