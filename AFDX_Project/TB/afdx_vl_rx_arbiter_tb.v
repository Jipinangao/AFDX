//=====================================================================
// afdx_vl_rx_arbiter_tb : arbitration test against the real vl_table
//   1) only A requests -> A gets its entry, B sees nothing
//   2) only B requests -> B gets its entry, A sees nothing
//   3) A and B request in the same cycle -> A served first, B one cycle later,
//      each receives the entry of ITS OWN VLID (no cross-talk)
//   4) arbitration_defer_count increments on a conflict
//=====================================================================
`timescale 1ns/1ps
module afdx_vl_rx_arbiter_tb;
    localparam RX_IDX_W = 12;
    reg clk, rst_n;

    reg a_lookup_valid; reg [RX_IDX_W-1:0] a_lookup_vlid;
    wire a_lookup_rdy; wire [10:0] a_min_len, a_max_len; wire a_entry_valid;

    reg b_lookup_valid; reg [RX_IDX_W-1:0] b_lookup_vlid;
    wire b_lookup_rdy; wire [10:0] b_min_len, b_max_len; wire b_entry_valid;

    wire vlt_lookup_valid; wire [RX_IDX_W-1:0] vlt_lookup_vlid;
    wire vlt_lookup_rdy; wire [10:0] vlt_min_len, vlt_max_len; wire vlt_entry_valid;
    wire [31:0] arbitration_defer_count;
    wire init_done;

    // vl_table的其余接口(TX查表/配置总线)本测试不使用。
    reg [15:0] reg_addr; reg [31:0] reg_wdata; reg reg_we, reg_re;
    wire [31:0] reg_rdata; wire reg_rdy;
    reg tx_lookup_valid; reg [6:0] tx_lookup_vlid;
    wire tx_lookup_rdy; wire [2:0] tx_bag_sel; wire [10:0] tx_max_len; wire [47:0] tx_dest_mac; wire tx_entry_valid;

    integer pass_count, fail_count;

    afdx_vl_rx_arbiter #(.RX_IDX_W(RX_IDX_W)) arb (
        .clk(clk), .rst_n(rst_n),
        .a_lookup_valid(a_lookup_valid), .a_lookup_vlid(a_lookup_vlid),
        .a_lookup_rdy(a_lookup_rdy), .a_min_len(a_min_len), .a_max_len(a_max_len), .a_entry_valid(a_entry_valid),
        .b_lookup_valid(b_lookup_valid), .b_lookup_vlid(b_lookup_vlid),
        .b_lookup_rdy(b_lookup_rdy), .b_min_len(b_min_len), .b_max_len(b_max_len), .b_entry_valid(b_entry_valid),
        .vlt_lookup_valid(vlt_lookup_valid), .vlt_lookup_vlid(vlt_lookup_vlid),
        .vlt_lookup_rdy(vlt_lookup_rdy), .vlt_min_len(vlt_min_len), .vlt_max_len(vlt_max_len), .vlt_entry_valid(vlt_entry_valid),
        .arbitration_defer_count(arbitration_defer_count)
    );

    vl_table #(.NUM_TX_VL(128), .NUM_RX_VL(4096), .TX_IDX_W(7), .RX_IDX_W(RX_IDX_W)) vlt (
        .clk(clk), .rst_n(rst_n),
        .reg_addr(reg_addr), .reg_wdata(reg_wdata), .reg_we(reg_we), .reg_re(reg_re),
        .reg_rdata(reg_rdata), .reg_rdy(reg_rdy),
        .tx_lookup_valid(tx_lookup_valid), .tx_lookup_vlid(tx_lookup_vlid),
        .tx_lookup_rdy(tx_lookup_rdy), .tx_bag_sel(tx_bag_sel), .tx_max_len(tx_max_len),
        .tx_dest_mac(tx_dest_mac), .tx_entry_valid(tx_entry_valid),
        .rx_lookup_valid(vlt_lookup_valid), .rx_lookup_vlid(vlt_lookup_vlid),
        .rx_lookup_rdy(vlt_lookup_rdy), .rx_min_len(vlt_min_len), .rx_max_len(vlt_max_len),
        .rx_entry_valid(vlt_entry_valid), .init_done(init_done)
    );

    initial begin clk=0; forever #4 clk=~clk; end

    // capture the one-cycle rdy pulses with their data and arrival cycle
    integer cyc = 0;
    reg a_seen, b_seen; integer a_t, b_t;
    reg [10:0] a_min, a_max, b_min, b_max; reg a_val, b_val;
    always @(posedge clk) begin
        cyc <= cyc + 1;
        if (a_lookup_rdy) begin a_seen <= 1; a_t <= cyc; a_min <= a_min_len; a_max <= a_max_len; a_val <= a_entry_valid; end
        if (b_lookup_rdy) begin b_seen <= 1; b_t <= cyc; b_min <= b_min_len; b_max <= b_max_len; b_val <= b_entry_valid; end
    end

    task automatic check;
        input cond; input [8*72-1:0] name;
        begin
            if (cond) begin $display("[PASS] %0s", name); pass_count=pass_count+1; end
            else begin $display("[FAIL] %0s", name); fail_count=fail_count+1; end
        end
    endtask

    task automatic program_rx_vl;
        input [RX_IDX_W-1:0] vlid; input [10:0] lmin; input [10:0] lmax;
        begin
            @(negedge clk);
            reg_addr = {1'b1, {(16-RX_IDX_W-1){1'b0}}, vlid};
            reg_wdata = {9'd0, 1'b1, lmax, lmin};
            reg_we = 1; @(negedge clk); reg_we = 0;
        end
    endtask

    integer t0;
    initial begin
        pass_count=0; fail_count=0; a_seen=0; b_seen=0;
        rst_n=0;
        a_lookup_valid=0; a_lookup_vlid=0; b_lookup_valid=0; b_lookup_vlid=0;
        reg_addr=0; reg_wdata=0; reg_we=0; reg_re=0; tx_lookup_valid=0; tx_lookup_vlid=0;
        repeat(5) @(posedge clk); rst_n=1;
        wait (init_done); repeat (2) @(posedge clk);
        program_rx_vl(12'd100, 11'd64, 11'd200);
        program_rx_vl(12'd200, 11'd80, 11'd500);
        repeat(2) @(posedge clk);

        // 1) only A
        a_seen=0; b_seen=0;
        @(negedge clk); a_lookup_valid = 1; a_lookup_vlid = 12'd100; @(negedge clk); a_lookup_valid = 0;
        repeat (6) @(posedge clk);
        check(a_seen && a_val && a_min==11'd64 && a_max==11'd200, "A-only request: A receives the entry of VL100");
        check(!b_seen, "A-only request: no rdy on the B side");

        // 2) only B
        a_seen=0; b_seen=0;
        @(negedge clk); b_lookup_valid = 1; b_lookup_vlid = 12'd200; @(negedge clk); b_lookup_valid = 0;
        repeat (6) @(posedge clk);
        check(b_seen && b_val && b_min==11'd80 && b_max==11'd500, "B-only request: B receives the entry of VL200");
        check(!a_seen, "B-only request: no rdy on the A side");

        // 3) same-cycle conflict
        a_seen=0; b_seen=0;
        @(negedge clk); a_lookup_valid = 1; a_lookup_vlid = 12'd100; b_lookup_valid = 1; b_lookup_vlid = 12'd200;
        @(negedge clk); a_lookup_valid = 0; b_lookup_valid = 0;
        repeat (6) @(posedge clk);
        check(a_seen && a_min==11'd64 && a_max==11'd200, "conflict: A gets VL100 entry");
        check(b_seen && b_min==11'd80 && b_max==11'd500, "conflict: B gets VL200 entry (no cross-talk)");
        check(b_t == a_t + 1, "conflict: B is served exactly one cycle after A");
        check(arbitration_defer_count == 32'd1, "arbitration_defer_count == 1 after one conflict");

        $display("VL-RX-ARBITER RESULT: PASS=%0d FAIL=%0d", pass_count, fail_count);
        if (fail_count==0) $display("========== VL-RX-ARBITER ALL PASS ==========");
        else $display("========== VL-RX-ARBITER FAIL ==========");
        $finish;
    end
endmodule
