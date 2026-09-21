// Stop the actual PLL reference at the board boundary. No force or lock override.
`ifdef RUN_PLL_LOSS
initial begin : pll_loss_test
    logic [15:0] m, f, a, d;
    integer passed = 0;
    @(posedge devrst_n);
    #5000;
    if (u_ft600q.ctrl_wr_ptr != u_ft600q.ctrl_rd_ptr)
        $fatal(1, "PLL startup must not report a loss");
    passed++;
    repeat (2) begin
        @(negedge usb_fifo_clk);
        usb_reference_enabled = 0;
        #20000;
        if (dut_con.rst_n !== 0 || dut_con.pll_locked_raw !== 0)
            $fatal(1, "PLL reference removal did not assert core reset");
        passed++;
        if (u_ft600q.ctrl_wr_ptr != u_ft600q.ctrl_rd_ptr)
            $fatal(1, "Unexpected response while reference absent");
        passed++;
        usb_reference_enabled = 1;
        #20000;
        if (u_ft600q.ctrl_wr_ptr - u_ft600q.ctrl_rd_ptr != 4)
            $fatal(1, "PLL loss must produce exactly one retained fault after recovery");
        u_ft600q.wait_response_frame_typed(m, f, a, d);
        if (m !== 16'h55aa || f !== 16'hffff || a !== 16'h0002 || d !== 0)
            $fatal(1, "Wrong PLL recovery fault %h %h %h %h", m, f, a, d);
        passed++;
        u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), 16'h0006, 0);
        u_ft600q.wait_response_frame_typed(m, f, a, d);
        if (a !== 16'h0006 || d !== 16'h0002 || f !== 0)
            $fatal(1, "Control path failed after PLL recovery");
        passed++;
        #20000;
        if (u_ft600q.ctrl_wr_ptr != u_ft600q.ctrl_rd_ptr)
            $fatal(1, "Duplicate PLL fault after acknowledgement");
        passed++;
    end
    $display("RESULTS: %0d passed, 0 failed", passed);
    $display("STATUS: PASS");
    $finish;
end
initial begin
    #200000;
    $fatal(1, "PLL recovery diagnostic timed out");
end
`endif
