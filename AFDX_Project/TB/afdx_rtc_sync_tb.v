//=====================================================================
// afdx_rtc_sync_tb
// 覆盖：
//   1) 自由运行：ms/秒进位正确
//   2) 小偏差同步 -> 按SLEW_STEP_MS牵引，不发生跳变
//   3) 大偏差同步 -> 立即跳变，step_count递增
//   4) 长时间无同步 -> rtc_locked失步
//   5) 畸形报文（长度不对）不生效
//=====================================================================
`timescale 1ns/1ps
module afdx_rtc_sync_tb;
    localparam CLK_FREQ_HZ = 1_000_000; // 1MHz，1ms=1000 cycles，加速仿真
    localparam integer CYCLES_PER_MS = CLK_FREQ_HZ/1000;

    reg clk, rst_n;
    reg [7:0] udp_rx_data;
    reg udp_rx_valid, udp_rx_last;
    reg [15:0] udp_rx_dst_port;
    reg cfg_we; reg [15:0] cfg_addr; reg [31:0] cfg_wdata;

    wire [31:0] rtc_seconds;
    wire [9:0]  rtc_ms;
    wire [31:0] rtc_uptime_ms;
    wire rtc_locked;
    wire [31:0] sync_age_ms, sync_count, step_count, slew_count;

    integer pass_count, fail_count;
    reg [7:0] pkt[0:5];
    integer j;

    afdx_rtc_sync #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .UDP_SYNC_PORT(16'd8319),
        .STEP_THRESHOLD_MS(32'd1000), .SLEW_STEP_MS(32'd2), .DEFAULT_LOST_MS(32'd5000)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .udp_rx_data(udp_rx_data), .udp_rx_valid(udp_rx_valid), .udp_rx_last(udp_rx_last),
        .udp_rx_dst_port(udp_rx_dst_port),
        .cfg_we(cfg_we), .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata),
        .rtc_seconds(rtc_seconds), .rtc_ms(rtc_ms), .rtc_uptime_ms(rtc_uptime_ms),
        .rtc_locked(rtc_locked), .sync_age_ms(sync_age_ms), .sync_count(sync_count),
        .step_count(step_count), .slew_count(slew_count)
    );

    initial begin clk=0; forever #5 clk=~clk; end

    task automatic check;
        input cond; input [8*128-1:0] name;
        begin
            if (cond) begin $display("[PASS] %0s", name); pass_count=pass_count+1; end
            else begin $display("[FAIL] %0s", name); fail_count=fail_count+1; end
        end
    endtask

    task automatic send_sync;
        input [31:0] seconds;
        input [15:0] ms;
        input integer plen; // 6=正常, 其它=畸形(截断)测试
        begin
            pkt[0]=seconds[31:24]; pkt[1]=seconds[23:16]; pkt[2]=seconds[15:8]; pkt[3]=seconds[7:0];
            pkt[4]=ms[15:8]; pkt[5]=ms[7:0];
            j = 0;
            while (j < plen) begin
                @(negedge clk);
                udp_rx_data = pkt[j]; udp_rx_valid = 1; udp_rx_last = (j == plen-1);
                udp_rx_dst_port = 16'd8319;
                @(posedge clk);
                j = j + 1;
            end
            @(negedge clk); udp_rx_valid = 0; udp_rx_last = 0;
        end
    endtask

    initial begin
        pass_count=0; fail_count=0;
        rst_n=0; udp_rx_data=0; udp_rx_valid=0; udp_rx_last=0; udp_rx_dst_port=0;
        cfg_we=0; cfg_addr=0; cfg_wdata=0;
        repeat(5) @(posedge clk); rst_n=1;

        // ---- 1) 自由运行计时正确性 ----
        repeat(2500*1) @(posedge clk); // 约2.5ms
        check(rtc_ms >= 10'd2 && rtc_seconds == 32'd0, "free running: rtc_ms advanced after ~2.5 ms, no premature second carry");

        repeat(CYCLES_PER_MS*1000) @(posedge clk); // 再推进约1秒
        check(rtc_seconds == 32'd1, "free running: rtc_seconds carries to 1 after ~1 s");

        // ---- 2) 小偏差同步：牵引而非跳变 ----
        // 此时本地时间约为(1, ~500ms附近)，构造一个仅比本地快5ms的基准。
        send_sync(rtc_seconds, rtc_ms + 10'd5, 6);
        @(posedge clk); #1;
        check(step_count == 32'd0, "small offset sync: no step (step_count=0)");
        check(slew_count == 32'd1, "small offset sync: one slew (slew_count=1)");
        check(sync_count == 32'd1, "small offset sync: sync_count incremented");
        check(rtc_locked == 1'b1, "after small offset sync: rtc_locked=1");

        // ---- 3) 大偏差同步：立即跳变 ----
        send_sync(32'd100000, 16'd0, 6);
        @(posedge clk); #1;
        check(step_count == 32'd1, "large offset sync: one step (step_count=1)");
        check(rtc_seconds == 32'd100000 && rtc_ms == 10'd0, "large offset sync: RTC aligned to the reference at once");
        check(sync_age_ms == 32'd0, "sync_age_ms cleared after sync");

        // ---- 4) 畸形报文(长度不足6字节)不生效 ----
        send_sync(32'd1, 16'd1, 4);
        @(posedge clk); #1;
        check(rtc_seconds == 32'd100000, "malformed (truncated) message does not change the RTC");
        check(sync_count == 32'd2, "malformed message not counted in sync_count (still 2)");

        // ---- 5) 长时间无同步 -> 失步 ----
        repeat(CYCLES_PER_MS*5001) @(posedge clk);
        check(rtc_locked == 1'b0, "rtc_locked=0 after the lost-sync threshold");
        check(rtc_uptime_ms > 32'd0, "rtc_uptime_ms keeps increasing monotonically, independent of the RTC absolute value");

        $display("============================================");
        $display("RTC-SYNC RESULT: PASS=%0d FAIL=%0d", pass_count, fail_count);
        if (fail_count==0) $display("========== RTC-SYNC ALL PASS ==========");
        else $display("========== RTC-SYNC FAIL ==========");
        $display("============================================");
        $finish;
    end
endmodule
