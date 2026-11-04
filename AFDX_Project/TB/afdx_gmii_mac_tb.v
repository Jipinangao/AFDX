//=====================================================================
// afdx_gmii_mac_tb : self-written GMII MAC (TX + RX) loopback test
//  1) frame bytes -> TX MAC -> GMII -> RX MAC -> identical frame bytes, FCS verified
//  2) GMII wire format: 7 x 0x55, 0xD5, frame, FCS(CRC-32) checked against an independent CRC
//  3) a corrupted bit on the wire is reported in o_err on the last byte
//  4) a 1514-byte and a 60-byte frame 30 cycles apart (the MAC needs >= 24 idle cycles between frames)
//=====================================================================
`timescale 1ns/1ps
module afdx_gmii_mac_tb;
    reg clk = 0, rst_n = 0;
    reg s_valid = 0; reg [7:0] s_data = 0;
    wire [7:0] txd; wire tx_en, tx_er;
    reg  flip = 0;
    integer cnt_en = 0;
    wire [7:0] rxd = (flip && tx_en && cnt_en == 40) ? (txd ^ 8'h10) : txd;
    wire [7:0] o_data; wire o_valid, o_err;
    afdx_gmii_mac_tx utx (.clk(clk), .rst_n(rst_n), .s_valid(s_valid), .s_data(s_data), .gmii_txd(txd), .gmii_tx_en(tx_en), .gmii_tx_er(tx_er));
    afdx_gmii_mac_rx urx (.clk(clk), .rst_n(rst_n), .gmii_rxd(rxd), .gmii_rx_dv(tx_en), .gmii_rx_er(1'b0), .o_data(o_data), .o_valid(o_valid), .o_err(o_err));
    always #4 clk = ~clk;
    integer pass = 0, fail = 0;
    task check; input c; input [8*80-1:0] n;
        begin if (c) begin pass = pass + 1; $display("[PASS] %0s", n); end
              else begin fail = fail + 1; $display("[FAIL] %0s", n); end end
    endtask
    always @(posedge clk) if (tx_en) cnt_en <= cnt_en + 1; else cnt_en <= 0;

    function [31:0] crc_of; input integer n; input [7:0] base; integer i, j; reg [31:0] c; reg [7:0] b;
        begin c = 32'hFFFFFFFF;
            for (i = 0; i < n; i = i + 1) begin b = base + i[7:0];
                for (j = 0; j < 8; j = j + 1) begin
                    if ((c[0] ^ b[j]) == 1'b1) c = (c >> 1) ^ 32'hEDB88320; else c = c >> 1;
                end
            end
            crc_of = ~c; end
    endfunction

    reg [7:0] wire_b [0:2047]; integer wn = 0;
    reg [7:0] rx_b [0:2047]; integer rn = 0, rx_frames = 0, rx_err_frames = 0, rx_len [0:7];
    always @(posedge clk) if (tx_en) begin wire_b[wn] = txd; wn = wn + 1; end
    reg rprev = 0;
    always @(posedge clk) begin
        rprev <= o_valid;
        if (o_valid) begin rx_b[rn] = o_data; rn = rn + 1; end
        if (o_err) rx_err_frames = rx_err_frames + 1;
        if (!o_valid && rprev) begin rx_len[rx_frames] = rn; rx_frames = rx_frames + 1; rn = 0; end
    end

    task send_frame; input integer n; input [7:0] base; integer k;
        begin
            for (k = 0; k < n; k = k + 1) begin @(negedge clk); s_valid = 1; s_data = base + k[7:0]; end
            @(negedge clk); s_valid = 0;
        end endtask

    integer i; reg ok; reg [31:0] f;
    initial begin
        repeat (4) @(posedge clk); rst_n = 1; repeat (3) @(posedge clk);
        wn = 0;
        send_frame(64, 8'h10); repeat (60) @(posedge clk);
        check(rx_frames == 1 && rx_len[0] == 64, "RX MAC output: 1 frame of 64 bytes (preamble/SFD/FCS stripped)");
        ok = 1; for (i = 0; i < 7; i = i + 1) if (wire_b[i] !== 8'h55) ok = 0;
        check(ok && wire_b[7] === 8'hD5, "GMII wire: 7 x 0x55 then SFD 0xD5");
        ok = 1; for (i = 0; i < 64; i = i + 1) if (wire_b[8+i] !== (8'h10 + i[7:0])) ok = 0;
        check(ok && wn == 8 + 64 + 4, "GMII wire: frame bytes unchanged, total = 8 + 64 + 4 (FCS)");
        f = crc_of(64, 8'h10);
        check({wire_b[75], wire_b[74], wire_b[73], wire_b[72]} === f, "GMII wire: FCS equals independent CRC-32 (low byte first)");
        check(rx_err_frames == 0, "good frame: no error flag");
        send_frame(1514, 8'h40); repeat (30) @(posedge clk); send_frame(60, 8'h70); repeat (200) @(posedge clk);
        check(rx_frames == 3 && rx_len[1] == 1514 && rx_len[2] == 60, "1514-byte and 60-byte frames both received intact");
        check(rx_err_frames == 0, "no error flags on good frames");
        flip = 1; send_frame(100, 8'h05); repeat (150) @(posedge clk); flip = 0;
        check(rx_frames == 4 && rx_err_frames == 1, "corrupted frame: error flag raised exactly once");
        send_frame(80, 8'h22); repeat (150) @(posedge clk);
        check(rx_frames == 5 && rx_err_frames == 1, "next good frame passes after the corrupted one");
        $display("GMII-MAC RESULT: PASS=%0d FAIL=%0d", pass, fail);
        if (fail == 0) $display("========== GMII-MAC ALL PASS =========="); else $display("========== GMII-MAC FAIL ==========");
        $finish;
    end
endmodule
