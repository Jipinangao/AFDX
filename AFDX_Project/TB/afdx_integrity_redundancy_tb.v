//=====================================================================
// afdx_integrity_redundancy_tb
// XAPP1130 "Redundancy Management": per-network Integrity Checker followed by
// the Redundancy Manager. Statistics are produced only by real frame events
// (no direct MIB writes). Full-scale instance: 4096 receive VLs.
//=====================================================================
`timescale 1ns/1ps
module afdx_integrity_redundancy_tb;
    localparam W = 12;
    reg clk = 0, rst_n = 0;
    reg a_valid = 0, b_valid = 0;
    reg [W-1:0] a_vlid = 0, b_vlid = 0;
    reg [7:0] a_seq = 0, b_seq = 0;
    reg [10:0] a_len = 0, b_len = 0;
    reg a_error = 0, b_error = 0;
    reg cfg_we = 0; reg [15:0] cfg_addr = 0; reg [31:0] cfg_wdata = 0;
    reg mib_re = 0; reg [15:0] mib_addr = 0;

    wire out_valid, out_network, out_error, sequence_error, duplicate_drop, mib_rvalid, init_done;
    wire [W-1:0] out_vlid; wire [7:0] out_seq_num; wire [10:0] out_frame_len;
    wire [31:0] mib_rdata;
    wire event_valid; wire [W-1:0] event_vlid; wire [7:0] event_seq_num; wire event_network;

    integer pass = 0, fail = 0;
    reg [31:0] v;
    // sticky captures of the one-cycle result pulses
    reg got_out, got_dup, got_seqerr, got_err; reg [7:0] got_seq;

    afdx_integrity_redundancy #(.NUM_RX_VL(4096), .RX_IDX_W(W), .FIFO_DEPTH(8), .DEFAULT_SKEW_CYCLES(32'd8)) dut (
        .clk(clk), .rst_n(rst_n),
        .a_valid(a_valid), .a_vlid(a_vlid), .a_seq_num(a_seq), .a_frame_len(a_len), .a_error(a_error),
        .b_valid(b_valid), .b_vlid(b_vlid), .b_seq_num(b_seq), .b_frame_len(b_len), .b_error(b_error),
        .cfg_we(cfg_we), .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata),
        .out_valid(out_valid), .out_vlid(out_vlid), .out_seq_num(out_seq_num), .out_frame_len(out_frame_len),
        .out_network(out_network), .out_error(out_error), .sequence_error(sequence_error), .duplicate_drop(duplicate_drop),
        .event_valid(event_valid), .event_vlid(event_vlid), .event_seq_num(event_seq_num), .event_network(event_network),
        .mib_re(mib_re), .mib_addr(mib_addr), .mib_rvalid(mib_rvalid), .mib_rdata(mib_rdata), .init_done(init_done));

    always #4 clk = ~clk;

    always @(posedge clk) begin
        if (out_valid)      begin got_out = 1; got_seq = out_seq_num; end
        if (duplicate_drop) got_dup = 1;
        if (sequence_error) got_seqerr = 1;
        if (out_error)      got_err = 1;
    end

    task check; input c; input [8*64-1:0] n;
        begin if (c) begin pass = pass + 1; $display("[PASS] %0s", n); end
              else begin fail = fail + 1; $display("[FAIL] %0s", n); end end
    endtask

    // send one frame event and wait until the checker has finished with it
    task send; input net; input [W-1:0] vl; input [7:0] seq; input err;
        begin
            got_out = 0; got_dup = 0; got_seqerr = 0; got_err = 0;
            @(negedge clk);
            if (!net) begin a_valid = 1; a_vlid = vl; a_seq = seq; a_len = 64; a_error = err; end
            else      begin b_valid = 1; b_vlid = vl; b_seq = seq; b_len = 64; b_error = err; end
            @(negedge clk); a_valid = 0; b_valid = 0;
            repeat (6) @(posedge clk);
            #1;
        end
    endtask

    task mib; input [3:0] obj; input [W-1:0] vl;
        begin
            @(negedge clk); mib_addr = {obj, vl}; mib_re = 1; @(negedge clk); mib_re = 0;
            while (!mib_rvalid) @(posedge clk);
            #1 v = mib_rdata;
        end
    endtask

    task cfg; input [3:0] sel; input [W-1:0] vl; input [31:0] d;
        begin @(negedge clk); cfg_addr = {sel, vl}; cfg_wdata = d; cfg_we = 1; @(negedge clk); cfg_we = 0; end
    endtask

    initial begin
        repeat (4) @(posedge clk); #1 rst_n = 1;
        wait (init_done); repeat (2) @(posedge clk);
        cfg(4'd1, 12'd1, 32'd8);                       // VL1 SkewMAX = 8 cycles

        send(0, 12'd1, 8'd77, 0);  check(got_out && got_seq == 77,           "first frame after reset: any SN accepted");
        send(1, 12'd1, 8'd77, 0);  check(got_dup && !got_out,                "network B copy of SN77 inside SkewMAX is dropped");
        send(0, 12'd1, 8'd78, 0);  check(got_out && got_seq == 78,           "PSN+1 accepted");
        send(1, 12'd1, 8'd78, 0);  check(got_dup,                            "network B copy of SN78 dropped");
        send(0, 12'd1, 8'd80, 0);  check(got_out && got_seq == 80,           "PSN+2 accepted (single frame loss tolerated)");
        send(1, 12'd1, 8'd80, 0);  check(got_dup,                            "network B copy of SN80 dropped");
        send(0, 12'd1, 8'd81, 0);  check(got_out && got_seq == 81,           "sequence continues");
        send(0, 12'd1, 8'd84, 0);  check(!got_out && got_seqerr,             "SN84 after 81 rejected as sequence error");
        send(0, 12'd1, 8'd85, 0);  check(got_out && got_seq == 85,           "stream recovers: rejected SN stored as PSN");
        send(0, 12'd1, 8'd0, 0);   check(got_out && got_seq == 0,            "SN=0 (transmitter reset) always accepted");
        send(0, 12'd1, 8'd1, 0);   check(got_out && got_seq == 1,            "SN=1 accepted after SN=0");

        send(0, 12'd2, 8'd255, 0); check(got_out && got_seq == 255,          "VL2 first frame SN255");
        send(0, 12'd2, 8'd1, 0);   check(got_out && got_seq == 1,            "SN wraps 255 -> 1");

        mib(4'd0, 12'd1); check(v == 7, "VL1 rx_frames = 7 (77,78,80,81,85,0,1)");
        mib(4'd1, 12'd1); check(v == 1, "VL1 rx_errors = 1");
        mib(4'd2, 12'd1); check(v == 3, "VL1 duplicates = 3");
        mib(4'd3, 12'd1); check(v == 1, "VL1 seq_errors = 1");
        mib(4'd4, 12'd1); check(v == 1, "VL1 last SN seen = 1");

        // SkewMAX boundary
        send(0, 12'd3, 8'd1, 0);   check(got_out,                            "VL3 first frame");
        send(1, 12'd3, 8'd1, 0);   check(got_dup,                            "A/B copy inside SkewMAX window dropped");
        send(0, 12'd3, 8'd2, 0);   check(got_out && got_seq == 2,            "VL3 next SN accepted");
        repeat (40) @(posedge clk);
        send(1, 12'd3, 8'd2, 0);   check(got_out && got_seq == 2,            "same SN after SkewMAX expired is accepted as new frame");

        // frame errors are always dropped, also when integrity checking is disabled / first frame
        send(0, 12'd4, 8'd5, 1);   check(!got_out && got_err,                "errored first frame is dropped");
        cfg(4'd0, 12'd5, 32'd0);                       // VL5: integrity checking off
        send(0, 12'd5, 8'd10, 0);  check(got_out,                            "VL5 (integrity off) frame accepted");
        send(0, 12'd5, 8'd100, 0); check(got_out && got_seq == 100,          "VL5 (integrity off): SN jump 10->100 not checked");
        send(0, 12'd5, 8'd101, 1); check(!got_out && got_err,                "VL5 (integrity off): errored frame still dropped");

        // full-scale: last RX VL index
        send(0, 12'd4095, 8'd9, 0); check(got_out && out_vlid == 12'd4095,   "VL4095 (last of 4096) works");
        send(1, 12'd4095, 8'd9, 0); check(got_dup,                           "VL4095 redundancy works");
        mib(4'd0, 12'd4095); check(v == 1, "VL4095 rx_frames = 1");
        mib(4'd0, 12'd0);    check(v == 0, "VL0 untouched: rx_frames = 0 (VL state is independent)");

        $display("INTEGRITY RESULT: PASS=%0d FAIL=%0d", pass, fail);
        if (fail == 0) $display("========== INTEGRITY-REDUNDANCY ALL PASS =========="); else $display("========== INTEGRITY-REDUNDANCY FAIL ==========");
        $finish;
    end
endmodule
