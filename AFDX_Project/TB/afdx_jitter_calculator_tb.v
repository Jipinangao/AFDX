//=====================================================================
// afdx_jitter_calculator_tb : 用真实vl_table + XAPP1130原文算例验证
//
// XAPP1130原文("Building an End System"一节)给出的算例：
//   端系统有3条VL: VLA LMAX=1518字节, VLB LMAX=1024字节, VLC LMAX=512字节，
//   介质速率100Mb/s，文档给出的结论是"allowed jitter is 289us"。
//   本测试用完全相同的3条VL配置驱动真实vl_table，核对本模块算出的
//   JitterMax是否与文档数值一致(本设计对"extra"用向上取整，因此
//   预期值是290us，比文档的289us保守1us，属于工程上"宁可估计得更宽松
//   /更安全，不可估计得比标准要求更紧"的合理取整方向，已在代码注释
//   中说明)。
//
// 覆盖：
//   1) 空表(全部entry_valid=0) -> JitterMax=40us(只有固定项)
//   2) XAPP1130原文3-VL算例 -> JitterMax=290us(即文档289us的保守取整)
//   3) 单条VL、LMAX=64字节(最小帧) -> 用公式手算比对
//   4) 极端情况(许多条大帧VL) -> 被500us上限钳制
//=====================================================================
`timescale 1ns/1ps
module afdx_jitter_calculator_tb;
    localparam TX_IDX_W = 7;
    localparam NUM_TX_VL = 128;

    reg clk, rst_n;

    wire lookup_valid; wire [TX_IDX_W-1:0] lookup_vlid;
    wire lookup_rdy; wire [10:0] max_len; wire entry_valid;
    wire [9:0] jitter_max_us;
    wire calc_valid;
    wire [31:0] scan_pass_count;

    reg [15:0] reg_addr; reg [31:0] reg_wdata; reg reg_we, reg_re;
    wire [31:0] reg_rdata; wire reg_rdy;
    reg [10:0] rx_dummy_min, rx_dummy_max;

    integer pass_count, fail_count;

    afdx_jitter_calculator #(.NUM_TX_VL(NUM_TX_VL), .TX_IDX_W(TX_IDX_W), .MEDIUM_BPS(100_000_000)) dut (
        .clk(clk), .rst_n(rst_n),
        .lookup_valid(lookup_valid), .lookup_vlid(lookup_vlid), .lookup_rdy(lookup_rdy),
        .max_len(max_len), .entry_valid(entry_valid),
        .jitter_max_us(jitter_max_us), .calc_valid(calc_valid), .scan_pass_count(scan_pass_count)
    );

    vl_table #(.NUM_TX_VL(NUM_TX_VL), .NUM_RX_VL(16), .TX_IDX_W(TX_IDX_W), .RX_IDX_W(4)) vlt (
        .clk(clk), .rst_n(rst_n),
        .reg_addr(reg_addr), .reg_wdata(reg_wdata), .reg_we(reg_we), .reg_re(reg_re),
        .reg_rdata(reg_rdata), .reg_rdy(reg_rdy),
        .tx_lookup_valid(lookup_valid), .tx_lookup_vlid(lookup_vlid),
        .tx_lookup_rdy(lookup_rdy), .tx_bag_sel(), .tx_max_len(max_len),
        .tx_dest_mac(), .tx_entry_valid(entry_valid),
        .rx_lookup_valid(1'b0), .rx_lookup_vlid(4'd0),
        .rx_lookup_rdy(), .rx_min_len(), .rx_max_len(), .rx_entry_valid()
    );

    initial begin clk=0; forever #5 clk=~clk; end

    task automatic check;
        input cond; input [8*128-1:0] name;
        begin
            if (cond) begin $display("[PASS] %0s", name); pass_count=pass_count+1; end
            else begin $display("[FAIL] %0s", name); fail_count=fail_count+1; end
        end
    endtask

    task automatic program_tx_vl;
        input [TX_IDX_W-1:0] vlid;
        input [10:0] lmax;
        begin
            @(negedge clk);
            reg_addr = {1'b0, {(14-TX_IDX_W){1'b0}}, vlid, 1'b0};
            reg_wdata = {17'd0, lmax, 1'b1, 3'd0}; // valid=1, bag_sel=0, dest_mac不影响jitter计算
            reg_we = 1;
            @(posedge clk);
            @(negedge clk);
            reg_we = 0;
        end
    endtask

    task automatic clear_all_vl;
        integer k;
        begin
            for (k=0;k<NUM_TX_VL;k=k+1) begin
                @(negedge clk);
                reg_addr = {1'b0, {(14-TX_IDX_W){1'b0}}, k[TX_IDX_W-1:0], 1'b0};
                reg_wdata = 32'd0; // valid=0
                reg_we = 1;
                @(posedge clk);
                @(negedge clk);
                reg_we = 0;
            end
        end
    endtask

    task automatic wait_scan_passes;
        input integer n;
        integer target;
        integer wdt;
        begin
            target = scan_pass_count + n;
            wdt = 0;
            while (scan_pass_count < target) begin
                @(posedge clk);
                wdt = wdt + 1;
                if (wdt > 20000) begin $display("WATCHDOG wait_scan_passes"); disable wait_scan_passes; end
            end
            @(posedge clk); #1;
        end
    endtask

    initial begin
        pass_count=0; fail_count=0;
        rst_n=0; reg_addr=0; reg_wdata=0; reg_we=0; reg_re=0;
        repeat(5) @(posedge clk); rst_n=1;

        // ---- 1) 空表 ----
        wait_scan_passes(2);
        check(calc_valid==1'b1, "calc_valid set after at least one scan pass");
        check(jitter_max_us==10'd40, "empty table (no VL configured): JitterMax = 40 us (fixed term only)");

        // ---- 2) XAPP1130原文3-VL算例 ----
        program_tx_vl(7'd0, 11'd1518);
        program_tx_vl(7'd1, 11'd1024);
        program_tx_vl(7'd2, 11'd512);
        wait_scan_passes(2);
        check(jitter_max_us==10'd290,
              "XAPP1130 3-VL example (1518/1024/512 B, 100 Mb/s): 290 us (document says 289 us, rounded conservatively)");

        // ---- 3) 清空后只留1条VL, LMAX=64字节(最小帧), 手算比对 ----
        clear_all_vl;
        program_tx_vl(7'd50, 11'd64);
        wait_scan_passes(2);
        // 手算: 8*(20+64)=672 bit; extra=ceil(672/100)=7us; 40+7=47us
        check(jitter_max_us==10'd47, "single VL (LMAX=64 B): hand-calculated JitterMax = 47 us, matches DUT");

        // ---- 4) 极端情况: 大量大帧VL被500us上限钳制 ----
        clear_all_vl;
        begin : many_vl
            integer k;
            for (k=0;k<NUM_TX_VL;k=k+1)
                program_tx_vl(k[TX_IDX_W-1:0], 11'd1518);
        end
        wait_scan_passes(2);
        check(jitter_max_us==10'd500, "128 VLs all LMAX=1518 B (extreme): correctly clamped to the 500 us limit");

        $display("============================================");
        $display("JITTER-CALC RESULT: PASS=%0d FAIL=%0d", pass_count, fail_count);
        if (fail_count==0) $display("========== JITTER-CALC ALL PASS ==========");
        else $display("========== JITTER-CALC FAIL ==========");
        $display("============================================");
        $finish;
    end
endmodule
