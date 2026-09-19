`timescale 1ns/1ps
// Reproducer for a real receive-path defect in ft600_245_fifo_fsm.
//
// The mover's RD_N is a registered output.  When cmd_out_ready falls (the RX
// CDC's registered almost-full), the mover leaves READ_DATA at the next edge --
// but RD_N is still low AT that edge, so the FT600 dequeues one more word (the
// chip advances on every clock it samples RD#=0; the model does the same).  Two
// accepted words are then never written into the CDC:
//   * the word dequeued one edge earlier sits in usb_d_pipe and would be
//     captured by the READ_POST case, but that capture is guarded by
//     cmd_out_ready, which is exactly the signal that just fell;
//   * the word dequeued at the exit edge lands in usb_d_pipe after it and
//     nothing captures it at all (READ_POST -> IDLE has no capture).
// Every ready-low exit therefore loses two words.  A host OUT burst longer than
// the CDC can absorb at the 66.7 -> 51.2 MHz rate difference exits that way
// repeatedly.  Command framing downstream is a plain four-word count, so each
// lost word also leaves cmd_decoder misaligned until the next reset
// (tb_usb_rx_probe_after_burst shows that host-visible form).
//
// Fixture: production mover + production 32-word cdc_fifo + the top level's
// ready gate (~almost_full & ~full), the existing FT600 model, a normally
// draining core.  No artificial core stall.  Every accepted word is accounted
// for at four points -- FT600 dequeue, mover strobe, actual CDC write
// (wr_en & ~full), core read -- and every ready-low exit is logged with what it
// cost.  Contract: every word the FT600 dequeued reaches the core exactly once,
// in order.  Runs in well under a second.
module tb_usb_rx_backpressure_loss;
    reg usb_clk=0, core_clk=0, rst_n=0;
    always #7.5 usb_clk=~usb_clk;          // 66.7 MHz FT600 clock
    always #9.765625 core_clk=~core_clk;   // 51.2 MHz: pll_48m CLKOP (lpf FREQUENCY NET clk_48m)

    localparam integer N_WORDS = 240;      // 60 four-word host packets in one FT600 burst

    wire [15:0] usb_d;
    wire [1:0] usb_be;
    wire rxf_n, txe_n, rd_n, wr_n, oe_n;
    wire [15:0] mover_data;
    wire mover_valid;
    wire fifo_full, fifo_almost_full, fifo_empty;
    wire [15:0] fifo_data;
    wire mover_ready = ~fifo_almost_full & ~fifo_full;   // consolidator_v2_top fsm_rx_ready
    wire fifo_read   = ~fifo_empty;                       // cmd_decoder in ST_RX: rx_ready=1
    integer delivered=0, strobed=0, written=0, written_full=0, failures=0, sequence_errors=0;
    integer stall_exits=0, guarded_discards=0, uncaptured=0;
    reg [15:0] expected_words[0:N_WORDS-1];

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

    // ---- accounting at every stage -------------------------------------------
    always @(posedge usb_clk) if (rst_n) begin
        if (mover_valid)               strobed=strobed+1;
        if (mover_valid && !fifo_full) written=written+1;        // cdc_fifo.do_write
        if (mover_valid &&  fifo_full) written_full=written_full+1; // strobe silently dropped by the FIFO
    end

    // ---- the boundary: READ_DATA -> READ_POST because ready fell --------------
    // Evaluated just before the edge that performs the transition.  RD_N is the
    // registered value the FT600 samples at this edge; a low RD_N here means the
    // chip dequeues once more.  usb_d_pipe holds the word dequeued one edge ago.
    always @(posedge usb_clk) if (rst_n &&
            mover.current_state==mover.STATE_READ_DATA &&
            mover.next_state==mover.STATE_READ_POST && !mover_ready) begin
        stall_exits=stall_exits+1;
        $display("[RX-BP] ready-low exit #%0d at %0t: FT600 dequeued=%0d  strobed=%0d  written=%0d  RD_N=%b  RXF_N=%b  (word in pipe %04h, on bus %04h)",
                 stall_exits,$time,model.rx_rd_ptr,strobed,written,rd_n,rxf_n,mover.usb_d_pipe,usb_d);
        if (mover.rd_n_was_low) guarded_discards=guarded_discards+1;   // READ_POST capture is guarded by ready
        if (!rd_n && !rxf_n)   uncaptured=uncaptured+1;                // dequeued at this edge, no later capture
    end

    // The FIFO is FWFT: check the head on the edge that consumes it.
    always @(posedge core_clk) if (rst_n && fifo_read) begin
        if (delivered < N_WORDS && fifo_data !== expected_words[delivered]) begin
            if (sequence_errors==0)
                $display("[RX-BP] first mismatch: expected[%0d]=%04h got=%04h",
                         delivered,expected_words[delivered],fifo_data);
            sequence_errors=sequence_errors+1;
        end
        delivered=delivered+1;
    end

    integer i, wait_cycles;
    initial begin
        repeat(5) @(negedge usb_clk); rst_n=1;
        // One contiguous host OUT burst: 60 four-word packets.  Identity pattern
        // 0x6000+i so any lost/duplicated/reordered word is named; one of the
        // packets is a valid USB_RX_WORDS probe (AA55 0000 005E 0000) on a real
        // four-word boundary, the bytes the hardware fuzz probes send.
        for(i=0;i<N_WORDS;i=i+1) expected_words[i]=16'h6000+i;
        expected_words[92]=16'haa55; expected_words[93]=16'h0000;
        expected_words[94]=16'h005e; expected_words[95]=16'h0000;
        for(i=0;i<N_WORDS;i=i+1) model.send_word(expected_words[i]);

        // The burst must actually reach the backpressure boundary, else the
        // fixture proves nothing.
        wait_cycles=0;
        while(!fifo_almost_full && wait_cycles<400) begin
            @(negedge usb_clk); wait_cycles=wait_cycles+1;
        end
        if(!fifo_almost_full) fail("receive FIFO never reached the backpressure boundary");

        // Drain to quiescence: FT600 empty, CDC empty, mover idle.
        wait_cycles=0;
        while((model.rx_count()!=0 || !fifo_empty ||
               mover.current_state!=mover.STATE_IDLE) && wait_cycles<6000) begin
            @(negedge core_clk); wait_cycles=wait_cycles+1;
        end
        repeat(10) @(negedge core_clk);

        $display("[RX-BP] accepted by FT600=%0d  strobed by mover=%0d  written to CDC=%0d  (strobes into a full CDC=%0d)  read by core=%0d",
                 model.rx_rd_ptr,strobed,written,written_full,delivered);
        $display("[RX-BP] ready-low exits=%0d  guarded READ_POST discards=%0d  dequeued-at-exit never captured=%0d  -> %0d words lost in the mover, %0d in the FIFO",
                 stall_exits,guarded_discards,uncaptured,model.rx_rd_ptr-strobed,strobed-written);
        if(model.rx_rd_ptr!=N_WORDS)
            fail("FT600 did not deliver the whole burst (bench/model problem, not the DUT)");
        if(stall_exits==0)
            fail("no ready-low exit occurred: the burst never exercised the boundary");
        if(model.rx_rd_ptr!=written)
            fail("every word the FT600 dequeued must be written into the CDC");
        if(written_full!=0)
            fail("the mover strobed into a full CDC");
        if(delivered!=N_WORDS || sequence_errors!=0) begin
            $display("[RX-BP] delivered=%0d/%0d sequence_errors=%0d host_remaining=%0d",
                     delivered,N_WORDS,sequence_errors,model.rx_count());
            fail("end-to-end receive stream lost/reordered words");
        end
        if(failures==0) begin
            $display("RESULTS: 5 passed, 0 failed");
            $display("STATUS: PASS");
        end else begin
            $display("RESULTS: %0d passed, %0d failed",5-failures,failures);
            $display("STATUS: FAIL");
            $fatal(1,"USB receive backpressure loses accepted words");
        end
        $finish;
    end
    initial begin #200000; fail("simulation watchdog"); $display("STATUS: FAIL"); $fatal(1); end
endmodule
