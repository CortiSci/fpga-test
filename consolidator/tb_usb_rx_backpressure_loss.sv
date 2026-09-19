`timescale 1ns/1ps
// Reproducer for a real receive-path defect: when cmd_out_ready falls while
// RD_N is already asserted, the FT600 consumes one final word but the mover
// suppresses cmd_out_valid. That word is permanently lost.
module tb_usb_rx_backpressure_loss;
    reg usb_clk=0, core_clk=0, rst_n=0;
    always #7.5 usb_clk=~usb_clk;          // approximately 66.7 MHz
    always #10.416667 core_clk=~core_clk; // 48 MHz

    wire [15:0] usb_d;
    wire [1:0] usb_be;
    wire rxf_n, txe_n, rd_n, wr_n, oe_n;
    wire [15:0] mover_data;
    wire mover_valid;
    wire fifo_full, fifo_almost_full, fifo_empty;
    wire [15:0] fifo_data;
    reg drain=1;
    wire mover_ready=~fifo_almost_full & ~fifo_full;
    wire fifo_read=drain & ~fifo_empty;
    integer delivered=0, forwarded=0, failures=0, sequence_errors=0;
    reg [15:0] expected_words[0:159];

    ft600q_tlm model(.clk_66m(usb_clk),.rst_n(rst_n),.usb_fifo_d(usb_d),
        .usb_fifo_be(usb_be),.rxf_n(rxf_n),.txe_n(txe_n),
        .rd_n(rd_n),.wr_n(wr_n),.oe_n(oe_n));
    ft600_245_fifo_fsm mover(.usb_fifo_clk(usb_clk),.devrst_n(rst_n),
        .usb_fifo_d(usb_d),.usb_fifo_rxf_n(rxf_n),.usb_fifo_txe_n(txe_n),
        .usb_fifo_rd_n(rd_n),.usb_fifo_wr_n(wr_n),.usb_fifo_oe_n(oe_n),
        .cmd_out_data(mover_data),.cmd_out_valid(mover_valid),.cmd_out_ready(mover_ready),
        .tx_in_data(16'h0),.tx_in_valid(1'b0),.tx_in_ready(),
        .tx_in_queued_words(10'h0),.telemetry_active(1'b0),.tx_source_idle(1'b1),
        .dbg_state());
    cdc_fifo #(.WIDTH(16),.DEPTH_LOG2(5)) fifo(.wr_clk(usb_clk),.wr_rst_n(rst_n),
        .wr_data(mover_data),.wr_en(mover_valid),.wr_full(fifo_full),
        .wr_almost_full(fifo_almost_full),.wr_half_full(),.rd_clk(core_clk),
        .rd_rst_n(rst_n),.rd_data(fifo_data),.rd_en(fifo_read),
        .rd_empty(fifo_empty),.rd_level());

    task fail(input string reason);
        begin
            failures=failures+1;
            $display("[RX-BP] FAIL %s",reason);
        end
    endtask

    always @(posedge usb_clk) if (mover_valid) forwarded=forwarded+1;

    // The FIFO is FWFT. Check the current head on the same edge that consumes it.
    always @(posedge core_clk) if (fifo_read) begin
        if (fifo_data !== expected_words[delivered]) begin
            if(sequence_errors==0)
                $display("[RX-BP] first mismatch expected[%0d]=%04h got=%04h",
                         delivered,expected_words[delivered],fifo_data);
            sequence_errors=sequence_errors+1;
        end
        delivered=delivered+1;
    end

    integer i, wait_cycles;
    initial begin
        repeat(5) @(negedge usb_clk); rst_n=1;
        // A finite host burst arrives at 66 MHz while the core drains normally
        // at 48 MHz. The rate difference alone fills the production 32-word CDC.
        for(i=0;i<160;i=i+1) expected_words[i]=16'h6000+i;
        // Put a valid USB_RX_WORDS probe on a real four-word boundary where the
        // current mover loses its address/data words under the rate mismatch.
        expected_words[92]=16'haa55; expected_words[93]=16'h0000;
        expected_words[94]=16'h005e; expected_words[95]=16'h0000;
        for(i=0;i<160;i=i+1) model.send_word(expected_words[i]);
        wait_cycles=0;
        while(!fifo_almost_full && wait_cycles<200) begin
            @(negedge usb_clk); wait_cycles=wait_cycles+1;
        end
        if(!fifo_almost_full) fail("receive FIFO never reached backpressure boundary");
        repeat(8) @(negedge usb_clk);

        wait_cycles=0;
        while((model.rx_count()!=0 || !fifo_empty) && wait_cycles<4000) begin
            @(negedge core_clk); wait_cycles=wait_cycles+1;
        end
        repeat(10) @(negedge core_clk);
        if(model.rx_rd_ptr!=forwarded) begin
            $display("[RX-BP] accepted by FT600=%0d forwarded to CDC=%0d",
                     model.rx_rd_ptr,forwarded);
            fail("accepted-word accounting mismatch");
        end
        if(delivered!=160 || sequence_errors!=0) begin
            $display("[RX-BP] delivered=%0d/160 sequence_errors=%0d host_remaining=%0d",
                     delivered,sequence_errors,model.rx_count());
            fail("end-to-end receive stream lost/reordered words");
        end
        if(failures==0) begin
            $display("RESULTS: 1 passed, 0 failed");
            $display("STATUS: PASS");
        end else begin
            $display("RESULTS: 0 passed, %0d failed",failures);
            $display("STATUS: FAIL");
            $fatal(1,"USB receive backpressure loses data");
        end
        $finish;
    end
    initial begin #100000; fail("simulation watchdog"); $fatal(1); end
endmodule
