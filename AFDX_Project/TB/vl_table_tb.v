//=====================================================================
// vl_table_tb : VL configuration table test
//  1) init sweep: entries are invalid until init_done, config writes after
//  2) TX entry write/lookup, RX entry write/lookup (index 4095 = last entry)
//  3) unconfigured entry reads valid=0
//  4) config read-back (TX lo/hi word, RX word)
//  5) read-back issued in the same cycle as a lookup is deferred, lookup result intact
//=====================================================================
`timescale 1ns/1ps
module vl_table_tb;
    localparam TX_IDX_W = 7, RX_IDX_W = 12;
    reg clk = 0, rst_n = 0;
    reg  [15:0] reg_addr = 0; reg [31:0] reg_wdata = 0; reg reg_we = 0, reg_re = 0;
    wire [31:0] reg_rdata; wire reg_rdy;
    reg tx_lookup_valid = 0; reg [TX_IDX_W-1:0] tx_lookup_vlid = 0;
    wire tx_lookup_rdy, tx_entry_valid; wire [2:0] tx_bag_sel; wire [10:0] tx_max_len; wire [47:0] tx_dest_mac;
    reg rx_lookup_valid = 0; reg [RX_IDX_W-1:0] rx_lookup_vlid = 0;
    wire rx_lookup_rdy, rx_entry_valid; wire [10:0] rx_min_len, rx_max_len;
    wire init_done;
    integer pass = 0, fail = 0;

    vl_table #(.NUM_TX_VL(128), .NUM_RX_VL(4096), .TX_IDX_W(TX_IDX_W), .RX_IDX_W(RX_IDX_W)) dut (
        .clk(clk), .rst_n(rst_n), .reg_addr(reg_addr), .reg_wdata(reg_wdata), .reg_we(reg_we), .reg_re(reg_re),
        .reg_rdata(reg_rdata), .reg_rdy(reg_rdy),
        .tx_lookup_valid(tx_lookup_valid), .tx_lookup_vlid(tx_lookup_vlid), .tx_lookup_rdy(tx_lookup_rdy),
        .tx_bag_sel(tx_bag_sel), .tx_max_len(tx_max_len), .tx_dest_mac(tx_dest_mac), .tx_entry_valid(tx_entry_valid),
        .rx_lookup_valid(rx_lookup_valid), .rx_lookup_vlid(rx_lookup_vlid), .rx_lookup_rdy(rx_lookup_rdy),
        .rx_min_len(rx_min_len), .rx_max_len(rx_max_len), .rx_entry_valid(rx_entry_valid), .init_done(init_done));

    always #4 clk = ~clk;   // 125 MHz

    task check; input c; input [8*64-1:0] n;
        begin if (c) begin pass = pass + 1; $display("[PASS] %0s", n); end
              else begin fail = fail + 1; $display("[FAIL] %0s", n); end end
    endtask

    task wr; input [15:0] a; input [31:0] d;
        begin @(negedge clk); reg_addr = a; reg_wdata = d; reg_we = 1; @(negedge clk); reg_we = 0; end
    endtask

    // lookup helpers: result is valid in the cycle where *_lookup_rdy=1
    task tx_lookup; input [TX_IDX_W-1:0] v;
        begin @(negedge clk); tx_lookup_vlid = v; tx_lookup_valid = 1;
              @(negedge clk); tx_lookup_valid = 0; end   // sampled now: rdy cycle
    endtask
    task rx_lookup; input [RX_IDX_W-1:0] v;
        begin @(negedge clk); rx_lookup_vlid = v; rx_lookup_valid = 1;
              @(negedge clk); rx_lookup_valid = 0; end
    endtask

    integer w;
    initial begin
        repeat (4) @(posedge clk);
        #1 rst_n = 1;
        // during init: a lookup must report an invalid entry, and a config write is ignored
        @(negedge clk); rx_lookup_vlid = 5; rx_lookup_valid = 1; @(negedge clk); rx_lookup_valid = 0;
        check(rx_entry_valid === 1'b0 && init_done === 1'b0, "during init: lookup returns invalid entry");
        wait (init_done); repeat (2) @(posedge clk);

        // TX entry VL5: BAG sel 3, LMAX 1518, MAC 03:00:00:00:00:05
        wr({1'b0, 7'd5, 1'b0}, {17'h00005, 11'd1518, 1'b1, 3'd3});
        wr({1'b0, 7'd5, 1'b1}, {1'b0, 31'h0180_0000});     // dest_mac[47:17]
        // MAC = {hi[30:0], lo[31:15]} = {31'h01800000, 17'h00005}
        tx_lookup(7'd5);
        check(tx_lookup_rdy && tx_bag_sel === 3'd3 && tx_max_len === 11'd1518 && tx_entry_valid &&
              tx_dest_mac === {31'h0180_0000, 17'h00005}, "TX VL5 lookup returns the configured entry");

        // RX entry: first and last index
        wr({1'b1, 15'd0},    {9'd0, 1'b1, 11'd1518, 11'd64});
        wr({1'b1, 15'd4095}, {9'd0, 1'b1, 11'd800, 11'd100});
        rx_lookup(12'd0);
        check(rx_lookup_rdy && rx_min_len === 11'd64 && rx_max_len === 11'd1518 && rx_entry_valid, "RX VL0 lookup");
        rx_lookup(12'd4095);
        check(rx_lookup_rdy && rx_min_len === 11'd100 && rx_max_len === 11'd800 && rx_entry_valid, "RX VL4095 (last entry) lookup");

        tx_lookup(7'd10);
        check(tx_entry_valid === 1'b0, "unconfigured TX VL10 reads invalid");
        rx_lookup(12'd77);
        check(rx_entry_valid === 1'b0, "unconfigured RX VL77 reads invalid");

        // read-back
        @(negedge clk); reg_addr = {1'b1, 15'd4095}; reg_re = 1; @(negedge clk); reg_re = 0;
        w = 0; while (!reg_rdy && w < 10) begin @(posedge clk); #1; w = w + 1; end
        check(reg_rdy && reg_rdata === {9'd0, 1'b1, 11'd800, 11'd100}, "RX config read-back");
        @(negedge clk); reg_addr = {1'b0, 7'd5, 1'b1}; reg_re = 1; @(negedge clk); reg_re = 0;
        w = 0; while (!reg_rdy && w < 10) begin @(posedge clk); #1; w = w + 1; end
        check(reg_rdy && reg_rdata === {1'b0, 31'h0180_0000}, "TX config (hi word) read-back");

        // read-back request colliding with a lookup in the same cycle (RX table)
        @(negedge clk); reg_addr = {1'b1, 15'd0}; reg_re = 1; rx_lookup_vlid = 12'd4095; rx_lookup_valid = 1;
        @(negedge clk); reg_re = 0; rx_lookup_valid = 0;
        check(rx_lookup_rdy && rx_min_len === 11'd100 && rx_max_len === 11'd800, "lookup wins the read port; result intact");
        w = 0; while (!reg_rdy && w < 10) begin @(posedge clk); #1; w = w + 1; end
        check(reg_rdy && reg_rdata === {9'd0, 1'b1, 11'd1518, 11'd64}, "deferred read-back completes with correct data");

        $display("VL_TABLE RESULT: PASS=%0d FAIL=%0d", pass, fail);
        if (fail == 0) $display("========== VL-TABLE ALL PASS =========="); else $display("========== VL-TABLE FAIL ==========");
        $finish;
    end
endmodule
