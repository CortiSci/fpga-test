# QuestaSim waveform configuration — Consolidator V2 SPI-ping sim
# Loaded automatically by test_spi_ping.do when this file exists.
# Covers the full USB→cmd_decoder→spi_cfg_ctrl→tail SPI path.

onerror {resume}
quietly WaveActivateNextPane {} 0

# ------------------------------------------------------------------
# Clocks & reset
# ------------------------------------------------------------------
add wave -divider "Clocks & Reset"
add wave -noupdate -label mclk_con      /tb_top/mclk_con
add wave -noupdate -label usb_clk       /tb_top/usb_fifo_clk
add wave -noupdate -label tail_mclk     /tb_top/tail_mclk
add wave -noupdate -label devrst_n      /tb_top/devrst_n

# ------------------------------------------------------------------
# FT600Q USB FIFO boundary (the 7 handshake signals)
# ------------------------------------------------------------------
add wave -divider "USB FT600Q handshake"
add wave -noupdate -label USB_D    -hex /tb_top/usb_fifo_d
add wave -noupdate -label USB_BE        /tb_top/usb_fifo_be
add wave -noupdate -label RXF_N         /tb_top/usb_fifo_rxf_n
add wave -noupdate -label TXE_N         /tb_top/usb_fifo_txe_n
add wave -noupdate -label OE_N          /tb_top/usb_fifo_oe_n
add wave -noupdate -label RD_N          /tb_top/usb_fifo_rd_n
add wave -noupdate -label WR_N          /tb_top/usb_fifo_wr_n

# ------------------------------------------------------------------
# FT600Q TLM internals — verify OE pre-drive and correct d_drive seq
# ------------------------------------------------------------------
add wave -divider "FT600Q TLM internals"
add wave -noupdate -label d_drive  -hex /tb_top/u_ft600q/d_drive
add wave -noupdate -label drive_d       /tb_top/u_ft600q/drive_d
add wave -noupdate -label rx_rd_ptr     /tb_top/u_ft600q/rx_rd_ptr
add wave -noupdate -label rx_wr_ptr     /tb_top/u_ft600q/rx_wr_ptr
add wave -noupdate -label ctrl_wr_ptr   /tb_top/u_ft600q/ctrl_wr_ptr
add wave -noupdate -label ctrl_rd_ptr   /tb_top/u_ft600q/ctrl_rd_ptr

# ------------------------------------------------------------------
# USB FIFO FSM (ft600_245_fifo_fsm inside consolidator_v2_top)
# Shows the 1-cycle pipeline: usb_d_pipe, rd_n_was_low, captures
# ------------------------------------------------------------------
add wave -divider "USB FSM (ft600_245_fifo_fsm)"
add wave -noupdate -label fsm_state     /tb_top/dut_con/usb_fifo/current_state
add wave -noupdate -label usb_d_pipe -hex /tb_top/dut_con/usb_fifo/usb_d_pipe
add wave -noupdate -label rd_n_was_low  /tb_top/dut_con/usb_fifo/rd_n_was_low
add wave -noupdate -label cmd_out_data -hex /tb_top/dut_con/usb_fifo/cmd_out_data
add wave -noupdate -label cmd_out_valid /tb_top/dut_con/usb_fifo/cmd_out_valid
add wave -noupdate -label cmd_out_ready /tb_top/dut_con/usb_fifo/cmd_out_ready

# ------------------------------------------------------------------
# CDC RX FIFO (USB→cmd_decoder domain crossing)
# ------------------------------------------------------------------
add wave -divider "CDC RX FIFO"
add wave -noupdate -label cdc_rx_empty  /tb_top/dut_con/cdc_rx_empty
add wave -noupdate -label cdc_rx_full   /tb_top/dut_con/cdc_rx_full
add wave -noupdate -label cmd_rx_valid  /tb_top/dut_con/cmd_rx_valid
add wave -noupdate -label cmd_rx_ready  /tb_top/dut_con/cmd_rx_ready

# ------------------------------------------------------------------
# cmd_decoder — the critical decode/discard logic
# magic_ok=0 at word_idx=3 silently discards the frame
# ------------------------------------------------------------------
add wave -divider "cmd_decoder"
add wave -noupdate -label cd_state      /tb_top/dut_con/cmd_dec/state
add wave -noupdate -label word_idx      /tb_top/dut_con/cmd_dec/word_idx
add wave -noupdate -label magic_ok      /tb_top/dut_con/cmd_dec/magic_ok
add wave -noupdate -label rx_data   -hex /tb_top/dut_con/cmd_dec/rx_data
add wave -noupdate -label reg_we        /tb_top/dut_con/cmd_dec/reg_we
add wave -noupdate -label reg_re        /tb_top/dut_con/cmd_dec/reg_re
add wave -noupdate -label reg_addr  -hex /tb_top/dut_con/cmd_dec/reg_addr
add wave -noupdate -label reg_wdata -hex /tb_top/dut_con/cmd_dec/reg_wdata

# ------------------------------------------------------------------
# CDC TX FIFO (cmd_decoder response→USB domain)
# ------------------------------------------------------------------
add wave -divider "CDC TX FIFO"
add wave -noupdate -label cdc_tx_empty  /tb_top/dut_con/cdc_tx_empty
add wave -noupdate -label ctrl_tx_valid /tb_top/dut_con/ctrl_tx_valid
add wave -noupdate -label ctrl_tx_ready /tb_top/dut_con/ctrl_tx_ready
add wave -noupdate -label framer_busy   /tb_top/dut_con/framer_busy

# ------------------------------------------------------------------
# SPI cfg path — Consolidator → Tail FPGA (4 channels, ch0 detail)
# ------------------------------------------------------------------
add wave -divider "SPI (Consolidator → Tail, 4 ch)"
add wave -noupdate -label SPI_SS        /tb_top/spi_ss_c2t
add wave -noupdate -label SPI_SCLK      /tb_top/spi_sclk_c2t
add wave -noupdate -label SPI_MOSI      /tb_top/spi_mosi_c2t
add wave -noupdate -label SPI_MISO      /tb_top/spi_miso_t2c

add wave -divider "spi_cfg_ctrl (instance: cfg_ctrl)"
add wave -noupdate -label cfg_go        /tb_top/dut_con/cfg_ctrl/cfg_go
add wave -noupdate -label cfg_busy      /tb_top/dut_con/cfg_ctrl/cfg_busy
add wave -noupdate -label cfg_ch        /tb_top/dut_con/cfg_ctrl/cfg_ch

add wave -divider "spi_master (instance: spi_cfg_m)"
add wave -noupdate -label spim_state    /tb_top/dut_con/spi_cfg_m/state
add wave -noupdate -label spim_bit_cnt  /tb_top/dut_con/spi_cfg_m/bit_cnt
add wave -noupdate -label spim_tx_data -hex /tb_top/dut_con/spi_cfg_m/tx_data
add wave -noupdate -label spim_rx_data -hex /tb_top/dut_con/spi_cfg_m/rx_data

# ------------------------------------------------------------------
# Tail FPGA #0 (ch0 / LEG5) SPI slave
# ------------------------------------------------------------------
add wave -divider "tail_fpga_small ch0 SPI slave (spi_slave_small)"
add wave -noupdate -label t0_ss_n         /tb_top/tail_ch[0]/dut_tail/u_spi_slave/ss_n
add wave -noupdate -label t0_bit_cnt      /tb_top/tail_ch[0]/dut_tail/u_spi_slave/bit_cnt
add wave -noupdate -label t0_byte_cnt     /tb_top/tail_ch[0]/dut_tail/u_spi_slave/byte_cnt
add wave -noupdate -label t0_opcode   -hex /tb_top/tail_ch[0]/dut_tail/u_spi_slave/opcode_r
add wave -noupdate -label t0_shift_tx -hex /tb_top/tail_ch[0]/dut_tail/u_spi_slave/shift_tx
add wave -noupdate -label t0_miso_ff      /tb_top/tail_ch[0]/dut_tail/u_spi_slave/spi_miso_ff

# ------------------------------------------------------------------
# Formatting
# ------------------------------------------------------------------
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {700 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 200
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {600 ns} {2 us}
