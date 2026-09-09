// All-leg-silent control-arbitration test.  Full board integration scope:
// commands traverse the FT600Q USB model; the four Tail inputs remain silent
// because no Tail CTRL/TELEM_EN transaction is issued.  No DUT or inter-FPGA
// signal is forced.

task automatic run_SA_SILENT_CONTROL();
    logic [15:0] m, f, a, d;
    integer telem_before;

    $display("[SA-SILENT-CTRL] four enabled but unconfigured Tails: control must remain live before V3 header");

    // Enable all physical links, but deliberately do not release Tail reset or
    // enable Tail telemetry.  ACQ_ALL_RUN therefore arms V2 while every leg
    // remains electrically silent through its normal FPGA boundary.
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, 16'h000F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d !== 16'h000F)
        $error("[SA-SILENT-CTRL] SPI_EN response 0x%04h, expected 0x000F", d);

    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_ACQ_ALL_RUN, 16'h000F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d !== 16'h000F)
        $error("[SA-SILENT-CTRL] start response 0x%04h, expected 0x000F", d);

    // The start acknowledgement proves the command arbiter did not wait for a
    // telemetry frame that cannot begin.  Confirm an ordinary read is live too.
    telem_before = tb_top.u_ft600q.telem_wr_ptr - tb_top.u_ft600q.telem_rd_ptr;
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_FW_VERSION, 16'h0000);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d !== 16'h0002)
        $error("[SA-SILENT-CTRL] FW_VERSION 0x%04h, expected 0x0002", d);
    if ((tb_top.u_ft600q.telem_wr_ptr - tb_top.u_ft600q.telem_rd_ptr) != telem_before)
        $error("[SA-SILENT-CTRL] telemetry began despite all Tail links being silent");

    // Stop must be live by the same pre-header arbitration path.  It must not
    // require a zero-padded final frame because no V3 header was ever emitted.
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_ACQ_ALL_RUN, 16'h0000);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d !== 16'h0000)
        $error("[SA-SILENT-CTRL] stop response 0x%04h, expected 0x0000", d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, 16'h0000);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);

    if (tb_top.u_ft600q.telem_wr_ptr != tb_top.u_ft600q.telem_rd_ptr)
        $error("[SA-SILENT-CTRL] unexpected V3 telemetry after stop");
    else
        $display("[SA-SILENT-CTRL] PASS: start, read, and stop responses completed with no V3 header");
endtask
