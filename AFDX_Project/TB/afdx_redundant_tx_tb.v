//=====================================================================
// afdx_redundant_tx_tb : A/B redundant transmit copy
//  - frames of 64 / 1514 bytes and a 1-byte frame are stored and sent
//  - the output (driving BOTH networks) equals the input byte for byte
//  - out_valid is continuous inside a frame, and there is a gap between frames
//  - back-to-back input frames are accepted (in_ready returns after the read-out started)
//=====================================================================
`timescale 1ns/1ps
module afdx_redundant_tx_tb;
    reg clk = 0, rst_n = 0;
    reg in_valid = 0; reg [7:0] in_data = 0; reg in_last = 0; wire in_ready;
    wire out_valid, out_last; wire [7:0] out_data, out_data_b; wire [31:0] frame_count;
    afdx_redundant_tx dut (.clk(clk), .rst_n(rst_n), .in_valid(in_valid), .in_data(in_data), .in_last(in_last), .in_ready(in_ready),
                           .out_valid(out_valid), .out_data(out_data), .out_data_b(out_data_b), .out_last(out_last), .frame_count(frame_count));
    always #4 clk = ~clk;
    integer pass = 0, fail = 0;
    task check; input c; input [8*80-1:0] n;
        begin if (c) begin pass = pass + 1; $display("[PASS] %0s", n); end
              else begin fail = fail + 1; $display("[FAIL] %0s", n); end end
    endtask

    // output monitor: one capture buffer per frame
    reg [7:0] cap [0:7][0:1599]; reg [7:0] capb [0:7][0:1599]; integer olen [0:7]; integer exp_len [0:7];
    integer on = 0, opos = 0; reg prev = 0;
    always @(posedge clk) begin
        prev <= out_valid;
        if (out_valid) begin cap[on][opos] = out_data; capb[on][opos] = out_data_b; opos = opos + 1; olen[on] = opos; end
        else if (prev) begin on = on + 1; opos = 0; end
    end

    task send; input integer n; input [7:0] base; integer k;
        begin
            for (k = 0; k < n; k = k + 1) begin
                @(negedge clk); while (!in_ready) @(negedge clk);
                in_valid = 1; in_data = base + k[7:0]; in_last = (k == n-1);
            end
            @(negedge clk); in_valid = 0; in_last = 0;
        end endtask

    integer i, f; reg ok;
    initial begin
        repeat (4) @(posedge clk); rst_n = 1; repeat (2) @(posedge clk);
        send(64, 8'h10); repeat (200) @(posedge clk);
        send(1514, 8'h40); repeat (1700) @(posedge clk);
        send(1, 8'h77); repeat (50) @(posedge clk);
        // back-to-back: second frame is presented while the first one is still being read out
        send(100, 8'h01); send(80, 8'h02); repeat (400) @(posedge clk);

        check(on == 5 && frame_count == 5, "5 frames transmitted, frame_count = 5");
        check(olen[0] == 64 && olen[1] == 1514 && olen[2] == 1, "frame lengths 64 / 1514 / 1 preserved");
        ok = 1; for (i = 0; i < 64; i = i + 1) if (i != 11 && cap[0][i] !== (8'h10 + i[7:0])) ok = 0;
        for (i = 0; i < 1514; i = i + 1) if (i != 11 && cap[1][i] !== (8'h40 + i[7:0])) ok = 0;
        check(ok, "network A bytes equal input bytes (except the interface-ID byte)");
        ok = 1; for (i = 0; i < 64; i = i + 1) if (capb[0][i] !== cap[0][i] && i != 11) ok = 0;
        for (i = 0; i < 1514; i = i + 1) if (capb[1][i] !== cap[1][i] && i != 11) ok = 0;
        check(ok, "network B copy equals network A copy except byte 11");
        check(cap[0][11] === 8'h20 && capb[0][11] === 8'h40 && cap[1][11] === 8'h20 && capb[1][11] === 8'h40,
              "source-MAC interface ID: A = 001 (0x20), B = 010 (0x40)");
        check(cap[2][0] === 8'h77 && capb[2][0] === 8'h77, "1-byte frame preserved on both networks");
        ok = (olen[3] == 100 && olen[4] == 80 && cap[3][99] === 8'h01 + 8'd99 && cap[4][0] === 8'h02);
        check(ok, "back-to-back frames: both delivered, in order, intact");
        $display("REDUNDANT-TX RESULT: PASS=%0d FAIL=%0d", pass, fail);
        if (fail == 0) $display("========== REDUNDANT-TX ALL PASS =========="); else $display("========== REDUNDANT-TX FAIL ==========");
        $finish;
    end
endmodule
