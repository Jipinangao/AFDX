//=====================================================================
// afdx_rx_payload_deliver_tb
// The decap output and the integrity decisions are emulated by the testbench.
//  1) accepted frame is delivered with its meta data (VL, UDP ports)
//  2) redundant copy on the other network is not delivered a second time
//  3) BURST: a 1471-byte frame followed by 6 short accepted frames while the long
//     one is still being released -> all 7 delivered, in order, data intact
//     (a single-frame buffer would silently lose the short frames)
//  4) frame rejected by the integrity checker is dropped and its space reclaimed
//  5) event whose identity does not match the stored frame -> correlation_miss_count
//  6) ring overflow (no decisions for > ring size) is counted in overrun_count
//=====================================================================
`timescale 1ns/1ps
module afdx_rx_payload_deliver_tb;
    localparam W = 12;
    reg clk = 0, rst_n = 0;

    reg a_payload_valid = 0; reg [7:0] a_payload_data = 0; reg a_frame_done = 0;
    reg [W-1:0] a_frame_vlid = 0; reg [7:0] a_frame_seq = 0; reg [10:0] a_frame_plen = 0;
    reg b_payload_valid = 0; reg [7:0] b_payload_data = 0; reg b_frame_done = 0;
    reg [W-1:0] b_frame_vlid = 0; reg [7:0] b_frame_seq = 0; reg [10:0] b_frame_plen = 0;
    reg out_valid = 0, out_error = 0, out_network = 0, event_valid = 0, event_network = 0;
    reg [W-1:0] out_vlid = 0, event_vlid = 0; reg [7:0] out_seq = 0, event_seq = 0;
	reg [31:0] cm0, ov0;
    wire deliver_valid, deliver_last, deliver_mf; wire [7:0] deliver_data; wire [W-1:0] deliver_vlid;
    wire [15:0] dl_ip, dl_sp, dl_dp; wire [12:0] dl_off; wire [31:0] overrun_count, correlation_miss_count;

    afdx_rx_payload_deliver #(.RX_IDX_W(W), .RING_AW(12), .DESC_AW(5), .REL_AW(6)) dut (
        .clk(clk), .rst_n(rst_n),
        .a_payload_valid(a_payload_valid), .a_payload_data(a_payload_data), .a_payload_last(1'b0),
        .a_frame_done(a_frame_done), .a_frame_vlid(a_frame_vlid), .a_frame_seq_num(a_frame_seq),
        .a_frame_payload_len(a_frame_plen), .a_frame_error(1'b0), .a_frame_ip_id(16'd7), .a_frame_mf(1'b0),
        .a_frame_offset_8b(13'd0), .a_frame_udp_src_port(16'd40000), .a_frame_udp_dst_port(16'd161),
        .b_payload_valid(b_payload_valid), .b_payload_data(b_payload_data), .b_payload_last(1'b0),
        .b_frame_done(b_frame_done), .b_frame_vlid(b_frame_vlid), .b_frame_seq_num(b_frame_seq),
        .b_frame_payload_len(b_frame_plen), .b_frame_error(1'b0), .b_frame_ip_id(16'd8), .b_frame_mf(1'b0),
        .b_frame_offset_8b(13'd0), .b_frame_udp_src_port(16'd40001), .b_frame_udp_dst_port(16'd6150),
        .out_valid(out_valid), .out_vlid(out_vlid), .out_seq_num(out_seq), .out_frame_len(11'd0),
        .out_network(out_network), .out_error(out_error),
        .event_valid(event_valid), .event_vlid(event_vlid), .event_seq_num(event_seq), .event_network(event_network),
        .deliver_valid(deliver_valid), .deliver_data(deliver_data), .deliver_last(deliver_last), .deliver_vlid(deliver_vlid),
        .deliver_ip_id(dl_ip), .deliver_mf(deliver_mf), .deliver_frag_offset_8b(dl_off),
        .deliver_udp_src_port(dl_sp), .deliver_udp_dst_port(dl_dp),
        .overrun_count(overrun_count), .correlation_miss_count(correlation_miss_count));

    always #4 clk = ~clk;
    integer pass = 0, fail = 0;
    task check; input c; input [8*80-1:0] n;
        begin if (c) begin pass = pass + 1; $display("[PASS] %0s", n); end
              else begin fail = fail + 1; $display("[FAIL] %0s", n); end end
    endtask

    // ---- delivered message log ----
    reg [7:0] msg [0:63][0:1599]; integer mlen [0:63]; reg [W-1:0] mvl [0:63]; reg [15:0] mdp [0:63]; reg [15:0] msp [0:63];
    integer mn = 0, mpos = 0;
    always @(posedge clk) if (deliver_valid) begin
        msg[mn][mpos] = deliver_data; mpos = mpos + 1;
        if (deliver_last) begin mlen[mn] = mpos; mvl[mn] = deliver_vlid; mdp[mn] = dl_dp; msp[mn] = dl_sp; mn = mn + 1; mpos = 0; end
    end

    // ---- stimulus helpers ----
    task frame_a; input [W-1:0] vl; input [7:0] sn; input integer len; input [7:0] base;
        integer k;
        begin
            for (k = 0; k < len; k = k + 1) begin
                @(negedge clk); a_payload_valid = 1; a_payload_data = base + k[7:0];
            end
            @(negedge clk); a_payload_valid = 0; a_frame_done = 1; a_frame_vlid = vl; a_frame_seq = sn; a_frame_plen = len;
            @(negedge clk); a_frame_done = 0;
        end endtask
    task frame_b; input [W-1:0] vl; input [7:0] sn; input integer len; input [7:0] base;
        integer k;
        begin
            for (k = 0; k < len; k = k + 1) begin
                @(negedge clk); b_payload_valid = 1; b_payload_data = base + k[7:0];
            end
            @(negedge clk); b_payload_valid = 0; b_frame_done = 1; b_frame_vlid = vl; b_frame_seq = sn; b_frame_plen = len;
            @(negedge clk); b_frame_done = 0;
        end endtask
    // integrity decision for the oldest frame of a network
    task decide; input net; input [W-1:0] vl; input [7:0] sn; input accept;
        begin
            @(negedge clk); event_valid = 1; event_network = net; event_vlid = vl; event_seq = sn;
            out_valid = accept; out_network = net; out_vlid = vl; out_seq = sn; out_error = 0;
            @(negedge clk); event_valid = 0; out_valid = 0;
        end endtask
    function ok_data; input integer idx; input integer len; input [7:0] base; integer k;
        begin ok_data = (mlen[idx] == len); for (k = 0; k < len; k = k + 1) if (msg[idx][k] !== (base + k[7:0])) ok_data = 0; end
    endfunction

    integer i, base_n; reg allok;
    initial begin
        repeat (4) @(posedge clk); rst_n = 1; repeat (3) @(posedge clk);

        // 1) single accepted frame
        frame_a(12'd10, 8'd1, 20, 8'h10); repeat (4) @(posedge clk); decide(0, 12'd10, 8'd1, 1);
        repeat (60) @(posedge clk);
        check(mn == 1 && ok_data(0, 20, 8'h10) && mvl[0] == 12'd10, "accepted frame delivered, data and VL correct");
        check(mdp[0] == 16'd161 && msp[0] == 16'd40000, "UDP ports of the frame are delivered with it");

        // 2) A accepted, B duplicate dropped
        frame_a(12'd11, 8'd1, 17, 8'h20); frame_b(12'd11, 8'd1, 17, 8'h20);
        decide(0, 12'd11, 8'd1, 1); decide(1, 12'd11, 8'd1, 0);
        repeat (60) @(posedge clk);
        check(mn == 2 && ok_data(1, 17, 8'h20), "redundant copy is not delivered twice");

        // 3) burst: long frame, then 6 short accepted frames during its release
        base_n = mn;
        frame_a(12'd20, 8'd1, 1471, 8'h30);
        decide(0, 12'd20, 8'd1, 1);                       // long release (~1.5k cycles) starts
        for (i = 0; i < 6; i = i + 1) begin
            frame_a(12'd21, i[7:0] + 8'd1, 17 + i, 8'h50 + i[7:0]);
            decide(0, 12'd21, i[7:0] + 8'd1, 1);          // accepted while the long frame is still streaming
        end
        repeat (4000) @(posedge clk);
        allok = (mn == base_n + 7) && ok_data(base_n, 1471, 8'h30);
        for (i = 0; i < 6; i = i + 1) allok = allok && ok_data(base_n + 1 + i, 17 + i, 8'h50 + i[7:0]) && (mvl[base_n+1+i] == 12'd21);
        check(allok, "burst: long frame + 6 short accepted frames: all 7 delivered in order, intact");
        check(overrun_count == 0, "burst: no overrun counted");

        // 4) rejected frame dropped, space reclaimed
        base_n = mn;
        frame_a(12'd30, 8'd5, 30, 8'h77); decide(0, 12'd30, 8'd5, 0);
        frame_a(12'd30, 8'd6, 18, 8'h88); decide(0, 12'd30, 8'd6, 1);
        repeat (100) @(posedge clk);
        check(mn == base_n + 1 && ok_data(base_n, 18, 8'h88), "rejected frame dropped, next frame delivered");

        // 5) identity mismatch
        c0_snapshot;
        frame_a(12'd40, 8'd1, 17, 8'h01);
        decide(0, 12'd41, 8'd1, 1);                       // wrong VL in the event
        repeat (60) @(posedge clk);
        check(correlation_miss_count == cm0 + 1 && mn == base_n + 1, "event with wrong identity: counted, nothing delivered");

        // 6) ring overflow (no decisions): 4 x 1471 bytes > 4096-byte ring
        ov0 = overrun_count;
        for (i = 0; i < 4; i = i + 1) frame_a(12'd50, i[7:0] + 8'd1, 1471, 8'h00);
        repeat (20) @(posedge clk);
        check(overrun_count > ov0, "ring overflow without decisions is counted in overrun_count");

        $display("RX-PAYLOAD-DELIVER RESULT: PASS=%0d FAIL=%0d", pass, fail);
        if (fail == 0) $display("========== RX-PAYLOAD-DELIVER ALL PASS =========="); else $display("========== RX-PAYLOAD-DELIVER FAIL ==========");
        $finish;
    end
    
    task c0_snapshot; begin cm0 = correlation_miss_count; end endtask
endmodule
