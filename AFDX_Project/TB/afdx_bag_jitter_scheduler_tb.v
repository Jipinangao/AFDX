`timescale 1ns/1ps
//=====================================================================
// TB: BAG + Jitter regulator
//
// 验证：
//   1) VL首次授权可正常发生，且按各自BAG周期持续调度。
//   2) 同VL连续授权间隔不低于BAG的保守约束。
//   3) 在正常(grant_ready恒为1)场景下，即便喂入一个很小的jitter_max_us，
//      也不应误报jitter_violation（因为调度器在BAG到期后立刻就能发出
//      授权，不会真的等到jitter窗口耗尽）。
//   4) 故意长时间不给grant_ready，人为制造"no grant within the jitter window"
//      的场景，验证jitter_violation确实会被置位。
//=====================================================================
module afdx_bag_jitter_scheduler_tb;
    localparam TX_IDX_W = 7;
    localparam CLK_FREQ_HZ = 1_000_000; // 1MHz: 1ms=1000 cycles

    reg clk, rst_n;
    reg req_valid; reg [6:0] req_vlid;
    wire tx_lookup_valid;
    wire [6:0] tx_lookup_vlid;
    reg tx_lookup_rdy;
    reg [2:0] tx_bag_sel;
    reg [10:0] tx_max_len;
    reg [47:0] tx_dest_mac;
    reg tx_entry_valid;
    wire grant_valid;
    wire [6:0] grant_vlid;
    wire [47:0] grant_dest_mac;
    wire [10:0] grant_max_len;
    reg grant_ready;
    reg [9:0] jitter_max_us_in;
    wire jitter_violation;

    integer cycles;
    integer last5, last10;
    integer grants5, grants10;
    integer errors;

    initial clk = 1'b0;
    always #5 clk = ~clk;

    // 用组合查表模型替代vl_table，让本TB只验证regulator算法。
    always @(*) begin
        tx_lookup_rdy = tx_lookup_valid;
        if (tx_lookup_vlid == 7'd5) begin
            tx_bag_sel = 3'd0;
            tx_max_len = 11'd64;
            tx_dest_mac = 48'h03_00_00_00_00_05;
            tx_entry_valid = 1'b1;
        end else if (tx_lookup_vlid == 7'd10) begin
            tx_bag_sel = 3'd1;
            tx_max_len = 11'd64;
            tx_dest_mac = 48'h03_00_00_00_00_0a;
            tx_entry_valid = 1'b1;
        end else begin
            tx_bag_sel = 3'd0;
            tx_max_len = 11'd0;
            tx_dest_mac = 48'd0;
            tx_entry_valid = 1'b0;
        end
    end

    afdx_bag_jitter_scheduler #(
        .NUM_TX_VL(128), .TX_IDX_W(TX_IDX_W),
        .CLK_FREQ_HZ(CLK_FREQ_HZ)
    ) dut (
        .clk(clk), .rst_n(rst_n), .req_valid(req_valid), .req_vlid(req_vlid),
        .tx_lookup_valid(tx_lookup_valid), .tx_lookup_vlid(tx_lookup_vlid),
        .tx_lookup_rdy(tx_lookup_rdy), .tx_bag_sel(tx_bag_sel), .tx_max_len(tx_max_len),
        .tx_dest_mac(tx_dest_mac), .tx_entry_valid(tx_entry_valid),
        .grant_valid(grant_valid), .grant_vlid(grant_vlid), .grant_dest_mac(grant_dest_mac),
        .grant_max_len(grant_max_len), .grant_ready(grant_ready),
        .jitter_max_us(jitter_max_us_in), .jitter_violation(jitter_violation)
    );

    // the consumed message is replaced by one for the other VL
    always @(posedge clk) if (rst_n && grant_valid && grant_ready) req_vlid <= (grant_vlid == 7'd5) ? 7'd10 : 7'd5;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) cycles <= 0;
        else cycles <= cycles + 1;
    end

    always @(posedge clk) begin
        if (rst_n && grant_valid && grant_ready) begin
            if (grant_vlid == 7'd5) begin
                if (last5 >= 0 && (cycles-last5) < 1000) begin
                    $display("[FAIL] VL5 BAG violation: %0d cycles", cycles-last5);
                    errors = errors + 1;
                end
                last5 = cycles;
                grants5 = grants5 + 1;
            end else if (grant_vlid == 7'd10) begin
                if (last10 >= 0 && (cycles-last10) < 2000) begin
                    $display("[FAIL] VL10 BAG violation: %0d cycles", cycles-last10);
                    errors = errors + 1;
                end
                last10 = cycles;
                grants10 = grants10 + 1;
            end
        end
    end

    initial begin
        errors=0; last5=-1; last10=-1; grants5=0; grants10=0; cycles=0;
        rst_n=0; req_valid=0; req_vlid=7'd5; grant_ready=1;
        // The scheduler serves ONE pending request at a time (single message buffer).
        // After every grant the 'message' is consumed and the next one is for the other VL, so
        // VL5 (BAG 1 ms) and VL10 (BAG 2 ms) alternate and are throttled by their own BAG.
        // 1 MHz clock: 1 us = 1 cycle, so jitter_max_us is expressed in cycles here.
        jitter_max_us_in=10'd200;
        repeat(4) @(posedge clk); #1 rst_n=1;
        req_valid=1; req_vlid=7'd5;
        repeat(5000) @(posedge clk);

        if (grants5 < 3 || grants10 < 2) begin
            $display("[FAIL] Grants too few: VL5=%0d VL10=%0d", grants5, grants10); errors=errors+1;
        end else $display("[PASS] continuous scheduling: VL5=%0d VL10=%0d", grants5, grants10);

        if (jitter_violation) begin
            $display("[FAIL] jitter_violation must not be raised in the normal case (grant_ready=1)"); errors=errors+1;
        end else $display("[PASS] jitter_violation=0 in the normal case (no false alarm)");

        // ---- 人为制造超过jitter窗口仍未获得授权的场景 ----
        req_valid = 0;
        grant_ready = 0;
        jitter_max_us_in = 10'd1; // 极小的jitter窗口(1us=1cycle@1MHz)，便于快速触发
        repeat(20) @(posedge clk);
        req_vlid = 7'd20; req_valid = 1'b1;   // unconfigured VL: lookup says invalid -> never granted
        repeat(200) @(posedge clk);
        if (!jitter_violation) begin
            $display("[FAIL] jitter_violation should be set after the forced timeout but is still 0"); errors=errors+1;
        end else $display("[PASS] jitter_violation correctly set after the forced timeout");

        if (errors==0) $display("==== afdx_bag_jitter_scheduler_tb: ALL PASS ====");
        else $display("==== afdx_bag_jitter_scheduler_tb: %0d ERROR(S) ====", errors);
        $finish;
    end
endmodule
