//=====================================================================
// AFDX_top_tb : 系统级测试，仅使用默认参数，支持综合后仿真。
// 激励源仅在 tx_data/tx_valid 上注入“VLID + 应用数据”，并观察
// rx_data/rx_valid。两个 GMII 端口（A 和 B）在内部自环连接到自身
// （A 发送 -> A 接收，B 发送 -> B 接收），并带有故障注入（网络断开 / FCS 错）。
//
// 覆盖内容：自配置、消息格式、A/B 冗余、单网故障、
//   帧损坏、分片与重组（3000 字节和 8192 字节）、越界 VL、
//   tx_ready 反压、应用接口吞吐量、时延、抖动。
// “MEASURE ...”开头的行会被引用到测试报告中。
//=====================================================================
`timescale 1ns/1ps
module AFDX_top_tb;
    reg clk = 0, reset = 1;
    reg  [7:0] tx_data = 0; reg tx_valid = 0; wire tx_ready;
    wire [7:0] rx_data; wire rx_valid;
    wire [7:0] a_txd, b_txd; wire a_en, b_en, a_er, b_er;
    reg  kill_a = 0, kill_b = 0, corrupt_a = 0, corrupt_b = 0;
    integer a_cnt = 0, b_cnt = 0;
    always @(posedge clk) begin a_cnt <= a_en ? a_cnt + 1 : 0; b_cnt <= b_en ? b_cnt + 1 : 0; end
    wire [7:0] a_rxd = (corrupt_a && a_cnt == 60) ? (a_txd ^ 8'h04) : a_txd;
    wire [7:0] b_rxd = (corrupt_b && b_cnt == 60) ? (b_txd ^ 8'h04) : b_txd;
    wire [7:0] err_flags;

    AFDX_top dut (
        .clk(clk), .reset(reset),
        .tx_data(tx_data), .tx_valid(tx_valid), .tx_ready(tx_ready),
        .rx_data(rx_data), .rx_valid(rx_valid), .rx_ready(1'b0),
        .gmii_a_txd(a_txd), .gmii_a_tx_en(a_en), .gmii_a_tx_er(a_er),
        .gmii_a_rxd(a_rxd), .gmii_a_rx_dv(a_en & ~kill_a), .gmii_a_rx_er(1'b0),
        .gmii_b_txd(b_txd), .gmii_b_tx_en(b_en), .gmii_b_tx_er(b_er),
        .gmii_b_rxd(b_rxd), .gmii_b_rx_dv(b_en & ~kill_b), .gmii_b_rx_er(1'b0),
        .err_flags_o(err_flags));

    always #4 clk = ~clk;        // 125 MHz 主时钟

    integer pass = 0, fail = 0;
    task check; input c; input [8*90-1:0] n;
        begin if (c) begin pass = pass + 1; $display("[PASS] %0s", n); end
              else begin fail = fail + 1; $display("[FAIL] %0s", n); end end
    endtask

    // 有效载荷模式：取决于 VLID 和字节索引
    function [7:0] pat; input [15:0] vl; input integer k; begin pat = vl[7:0] * 8'd7 + k[7:0] + 8'h31; end endfunction

    // ---------------- 应用端发送任务 ----------------
    integer tx_msgs = 0; time tx_t0 [0:511];
    integer w;
    task send_msg; input [15:0] vl; input integer n;
        integer k;
        begin
            w = 0; while (!tx_ready && w < 3000000) begin @(posedge clk); w = w + 1; end
            @(negedge clk);
            tx_valid = 1; tx_data = vl[15:8]; tx_t0[tx_msgs] = $time; tx_msgs = tx_msgs + 1;
            @(negedge clk); tx_data = vl[7:0];
            for (k = 0; k < n; k = k + 1) begin @(negedge clk); tx_data = pat(vl, k); end
            @(negedge clk); tx_valid = 0;
        end endtask

    // ---------------- 应用端接收监视 ----------------
    reg [7:0] rbuf [0:8300]; integer rpos = 0; reg in_rx = 0; time rt0;
    integer rx_msgs = 0; integer m_vl [0:511]; integer m_len [0:511]; time m_t0 [0:511]; reg m_ok [0:511];
    integer q; reg okk;
    always @(posedge clk) begin
        if (rx_valid) begin
            if (!in_rx) begin in_rx = 1; rpos = 0; rt0 = $time; end
            rbuf[rpos] = rx_data; rpos = rpos + 1;
        end else if (in_rx) begin
            in_rx = 0;
            m_vl[rx_msgs] = {rbuf[0], rbuf[1]}; m_len[rx_msgs] = rpos - 2; m_t0[rx_msgs] = rt0;
            okk = 1; for (q = 2; q < rpos; q = q + 1) if (rbuf[q] !== pat({rbuf[0], rbuf[1]}, q - 2)) okk = 0;
            m_ok[rx_msgs] = okk; rx_msgs = rx_msgs + 1;
        end
    end

    integer cyc = 0, cyc0 = 0;
    always @(posedge clk) cyc <= cyc + 1;
    integer i, n0, c0, round, v; time t_first_tx, t_last_rx, d; real mbps;
    time lmin, lmax, worst; integer idx0;

    initial begin
        repeat (6) @(posedge clk); reset = 0;

        // ---------- 0) 自配置检查 ----------
        c0 = 0; while (!tx_ready && c0 < 60000) begin @(posedge clk); c0 = c0 + 1; end
        $display("MEASURE self-configuration (all 128 TX VLs and 4096 RX VLs) finished %0d cycles after reset", c0);
        check(tx_ready && c0 < 20000, "VL tables configured by the design itself; tx_ready rises without any configuration input");

        // ---------- 1) 基本消息测试 ----------
        n0 = rx_msgs; send_msg(16'd5, 10); repeat (1500) @(posedge clk);
        check(rx_msgs == n0 + 1 && m_vl[n0] == 5 && m_len[n0] == 10 && m_ok[n0], "10-byte message on VL5 delivered once with VLID and data intact");
        n0 = rx_msgs; send_msg(16'd9, 1471); repeat (6000) @(posedge clk);
        check(rx_msgs == n0 + 1 && m_vl[n0] == 9 && m_len[n0] == 1471 && m_ok[n0], "1471-byte message (maximum single frame, 1518-byte Ethernet frame) on VL9");

        // ---------- 2) tx_ready 流量反压测试 ----------
        send_msg(16'd7, 40);
        @(posedge clk); #1;
        check(!tx_ready, "tx_ready drops after a message while the end system is busy with it");
        w = 0; while (!tx_ready && w < 3000000) begin @(posedge clk); w = w + 1; end
        check(tx_ready, "tx_ready returns to 1");
        repeat (3000) @(posedge clk);

        // ---------- 3) IP 分片与重组测试 ----------
        n0 = rx_msgs; send_msg(16'd12, 3000); repeat (460000) @(posedge clk);
        check(rx_msgs == n0 + 1 && m_vl[n0] == 12 && m_len[n0] == 3000 && m_ok[n0], "3000-byte message is IP-fragmented, sent, and reassembled intact (VL12)");
        n0 = rx_msgs; send_msg(16'd13, 8192); repeat (1000000) @(posedge clk);
        check(rx_msgs == n0 + 1 && m_vl[n0] == 13 && m_len[n0] == 8192 && m_ok[n0], "8192-byte message (maximum) fragmented and reassembled intact (VL13)");

        // ---------- 4) 冗余机制：单网断开 / 帧损坏测试 ----------
        kill_a = 1; n0 = rx_msgs; send_msg(16'd20, 200); repeat (4000) @(posedge clk); kill_a = 0;
        check(rx_msgs == n0 + 1 && m_vl[n0] == 20 && m_len[n0] == 200 && m_ok[n0], "network A dead: message still delivered once (via B)");
        kill_b = 1; n0 = rx_msgs; send_msg(16'd21, 200); repeat (4000) @(posedge clk); kill_b = 0;
        check(rx_msgs == n0 + 1 && m_vl[n0] == 21 && m_ok[n0], "network B dead: message still delivered once (via A)");
        corrupt_a = 1; n0 = rx_msgs; send_msg(16'd22, 300); repeat (4000) @(posedge clk); corrupt_a = 0;
        check(rx_msgs == n0 + 1 && m_vl[n0] == 22 && m_ok[n0], "FCS error on network A: frame dropped by the MAC, message delivered once via B");
        corrupt_b = 1; n0 = rx_msgs; send_msg(16'd23, 300); repeat (4000) @(posedge clk); corrupt_b = 0;
        check(rx_msgs == n0 + 1 && m_vl[n0] == 23 && m_ok[n0], "FCS error on network B: message delivered once via A");
        kill_a = 1; kill_b = 1; n0 = rx_msgs; send_msg(16'd24, 100); repeat (4000) @(posedge clk); kill_a = 0; kill_b = 0;
        check(rx_msgs == n0, "both networks dead: nothing delivered (no phantom message)");
        n0 = rx_msgs; send_msg(16'd25, 64); repeat (3000) @(posedge clk);
        check(rx_msgs == n0 + 1 && m_ok[n0], "traffic resumes after the networks come back");

        // ---------- 5) 越界 VLID 检查 ----------
        n0 = rx_msgs; send_msg(16'd200, 50); repeat (3000) @(posedge clk);
        check(rx_msgs == n0 && err_flags[6], "VLID outside the 128 transmit VLs: message dropped and flagged (err_flags_o[6])");

        // ---------- 6) 吞吐量测试：12 个 VL 轮询，1471 字节消息，带 tx_ready 反压 ----------
        repeat (140000) @(posedge clk);            // 等待每个 VL 的 BAG（包间隔时间）过去
        n0 = rx_msgs; idx0 = tx_msgs; t_first_tx = $time;
        for (round = 0; round < 5; round = round + 1)
            for (v = 1; v <= 12; v = v + 1) send_msg(v, 1471);
        w = 0; while (rx_msgs < n0 + 60 && w < 4000000) begin @(posedge clk); w = w + 1; end
        repeat (4000) @(posedge clk);
        okk = 1; for (i = 0; i < 60; i = i + 1) if (!m_ok[n0 + i] || m_len[n0 + i] != 1471) okk = 0;
        check(rx_msgs == n0 + 60 && okk, "60 messages (5 rounds x 12 VLs x 1471 B): none lost, none corrupted, none duplicated");
        t_last_rx = m_t0[n0 + 59] + 1471*8;
        d = t_last_rx - t_first_tx;
        mbps = (60.0 * 1471 * 8) / (d / 1000.0);
        $display("MEASURE application throughput: 60 x 1471 B payload in %0d ns = %0f Mbit/s (12 VLs, BAG 1 ms, tx_ready flow control)", d, mbps);
        check(mbps >= 100.0, "application payload throughput >= 100 Mbit/s");

        // ---------- 7) 时延与抖动测试 ----------
        repeat (140000) @(posedge clk);
        n0 = rx_msgs; idx0 = tx_msgs;
        send_msg(16'd30, 17); repeat (3000) @(posedge clk);
        $display("MEASURE latency, 17-byte message : first byte on tx_data -> first byte on rx_data = %0d ns", m_t0[n0] - tx_t0[idx0]);
        check(m_t0[n0] - tx_t0[idx0] < 2000000, "latency (17-byte message) < 2 ms");
        n0 = rx_msgs; idx0 = tx_msgs;
        send_msg(16'd31, 1471); repeat (8000) @(posedge clk);
        $display("MEASURE latency, 1471-byte message: first byte on tx_data -> first byte on rx_data = %0d ns", m_t0[n0] - tx_t0[idx0]);
        check(m_t0[n0] - tx_t0[idx0] < 2000000, "latency (1471-byte message) < 2 ms");

        // 抖动测试：12 个 VL 并发，长度分别为 17/300/900/1471 字节（最小、两个中间、最大帧长），
        // 每个 VL 每隔 1.1 ms（>= BAG）发送一帧，VL 之间错开 80 us
        repeat (140000) @(posedge clk);
        n0 = rx_msgs; idx0 = tx_msgs; cyc0 = cyc;
        for (round = 0; round < 4; round = round + 1)
            for (v = 1; v <= 12; v = v + 1) begin
                // 绝对调度：周期 137500 个周期（1.1 ms >= BAG），各个 VL 间隔 10000 个周期（80 us）
                while (cyc < cyc0 + round * 137500 + (v - 1) * 10000) @(posedge clk);
                send_msg(40 + v, (round == 0) ? 17 : (round == 1) ? 300 : (round == 2) ? 900 : 1471);
            end
        repeat (8000) @(posedge clk);
        worst = 0;
        for (v = 0; v < 12; v = v + 1) begin
            lmin = 64'd999999999999; lmax = 0;
            for (round = 0; round < 4; round = round + 1) begin
                d = m_t0[n0 + round*12 + v] - tx_t0[idx0 + round*12 + v];
                if (d < lmin) lmin = d;
                if (d > lmax) lmax = d;
            end
            if (lmax - lmin > worst) worst = lmax - lmin;
        end
        check(rx_msgs == n0 + 48, "48 paced messages (12 VLs x 4 frame sizes) all delivered");
        $display("MEASURE jitter (same VL, 4 frame sizes 17/300/900/1471 B): max latency - min latency, worst of 12 VLs = %0d ns", worst);
        check(worst < 100000, "jitter below 100 us (no three-digit microsecond jitter)");

        $display("AFDX_TOP RESULT: PASS=%0d FAIL=%0d", pass, fail);
        if (fail == 0) $display("========== AFDX_TOP ALL PASS =========es"); else $display("========== AFDX_TOP FAIL ==========");
        $finish;
    end
    initial begin #3000_000_000; $display("[FAIL] global watchdog"); $finish; end
endmodule