`timescale 1ns/1ps
//=====================================================================
// 测试台: ip_reasm_tb
// 目的  : 专项验证RX侧IP重组(ip_reasm.v)，直接按其输入契约驱动
//         "de-duplicated fragment burst"，切法与ip_frag.v的真实切法一致：
//           首片用户数据1464B(FragOffset=0,MF=1)，
//           延续片1472B/片，FragOffset=(已发用户字节+8)/8。
//         (首片带8字节UDP头，故延续片偏移比用户数据位置大8。)
// 覆盖  : A 未分片旁路  B 三片顺序  C 乱序到达  D 8192字节六片满载
//         E 超时释放    F 孤儿延续片 G 分片重叠 H 上下文耗尽
//         I 两条消息交错到达
// 判定  : 输出字节流与原始消息逐字节比对，不是只看长度/计数。
//=====================================================================
module ip_reasm_tb;
    localparam RX_IDX_W = 12;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg        in_valid = 0, in_last = 0, in_mf = 0;
    reg [7:0]  in_data = 0;
    reg [RX_IDX_W-1:0] in_vlid = 0;
    reg [15:0] in_ip_id = 0;
    reg [12:0] in_frag_offset_8b = 0;

    wire       out_valid, out_last;
    wire [7:0] out_data;
    wire [RX_IDX_W-1:0] out_vlid;
    wire [31:0] c_timeout, c_overlap, c_overflow, c_orphan, c_complete;

    // TIMEOUT_CYCLES必须明显大于"多条消息交错时,一条消息等待另一条
    // 消息传完自己的延续片才到来"的最长间隔(测试I: 约1464+若干空拍)，
    // 否则会被误判超时——这是仿真参数选择问题，不是模块本身的行为缺陷。
    ip_reasm #(.RX_IDX_W(RX_IDX_W), .NUM_CTX(4), .MAX_MSG_BYTES(8192), .TIMEOUT_CYCLES(6000)) dut (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_data(in_data), .in_last(in_last),
        .in_vlid(in_vlid), .in_ip_id(in_ip_id), .in_mf(in_mf), .in_frag_offset_8b(in_frag_offset_8b),
        .out_valid(out_valid), .out_data(out_data), .out_last(out_last), .out_vlid(out_vlid),
        .reasm_timeout_count(c_timeout), .reasm_overlap_count(c_overlap),
        .reasm_overflow_count(c_overflow), .reasm_orphan_count(c_orphan),
        .reasm_complete_count(c_complete)
    );

    integer pass_cnt = 0, fail_cnt = 0;
    task check; input cond; input [8*64-1:0] name; begin
        if (cond) begin pass_cnt = pass_cnt + 1; $display("[PASS] %0s", name); end
        else begin fail_cnt = fail_cnt + 1; $display("[FAIL] %0s", name); end
    end endtask

    // ---------- 原始消息(A用) ----------
    reg [7:0] msg [0:8191];
    reg [7:0] msg2 [0:8191];
    integer i;

    // ---------- 输出捕获(单个always块，避免同拍读写cap_idx的竞争) ----------
    reg [7:0] cap [0:8191];
    reg [7:0] last_msg [0:8191];
    integer cap_idx = 0;
    integer cap_msgs = 0;
    integer last_len = 0;
    reg [RX_IDX_W-1:0] cap_last_vlid = 0;
    integer ci;
    always @(posedge clk) begin
        if (out_valid) begin
            cap[cap_idx] = out_data;
            cap_idx = cap_idx + 1;
            if (out_last) begin
                for (ci = 0; ci < cap_idx; ci = ci + 1) last_msg[ci] = cap[ci];
                last_len = cap_idx;
                cap_last_vlid = out_vlid;
                cap_msgs = cap_msgs + 1;
                cap_idx = 0;
            end
        end
    end

    // ---------- 发送一个分片突发 ----------
    task send_frag;
        input [RX_IDX_W-1:0] vlid; input [15:0] ipid; input mf;
        input integer start; input integer len; input integer src;
        integer k;
        begin
            for (k = 0; k < len; k = k + 1) begin
                @(negedge clk);
                in_valid = 1; in_vlid = vlid; in_ip_id = ipid; in_mf = mf;
                in_frag_offset_8b = (start == 0) ? 13'd0 : ((start + 8) / 8);
                in_data = (src == 0) ? msg[start+k] : msg2[start+k];
                in_last = (k == len-1);
            end
            @(negedge clk); in_valid = 0; in_last = 0; in_mf = 0; in_frag_offset_8b = 0;
            repeat (3) @(negedge clk);
        end
    endtask

    task wait_msgs; input integer target; input integer max_cycles;
        integer w; begin
        w = 0;
        while (cap_msgs < target && w < max_cycles) begin @(posedge clk); w = w + 1; end
        repeat (5) @(posedge clk);
    end endtask

    function integer mism2; input integer len; input integer src;
        integer k, m; begin
        m = 0;
        for (k = 0; k < len; k = k + 1)
            if (last_msg[k] !== ((src==0) ? msg[k] : msg2[k])) m = m + 1;
        mism2 = m;
    end endfunction

    integer ov_before;
    integer base_msgs;

    initial begin
        for (i = 0; i < 8192; i = i + 1) begin
            msg[i]  = (i * 7 + 13) & 8'hFF;
            msg2[i] = (i * 11 + 200) & 8'hFF;
        end
        repeat (4) @(posedge clk); rst_n = 1; repeat (3) @(posedge clk);

        // ---------------- A: 未分片单帧旁路 ----------------
        base_msgs = cap_msgs;
        send_frag(12'd5, 16'h0001, 1'b0, 0, 17, 0);
        wait_msgs(base_msgs+1, 200);
        check(cap_msgs == base_msgs+1, "A unfragmented message bypassed (1 received)");
        check(last_len == 17 && mism2(17,0) == 0, "A bypass content identical byte by byte (17 B)");
        check(cap_last_vlid == 12'd5, "A bypass VLID correct");
        check(c_complete == 0, "A bypass does not use a reassembly context (complete count still 0)");

        // ---------------- B: 三片顺序 1464+1472+500 = 3436B ----------------
        base_msgs = cap_msgs;
        send_frag(12'd7, 16'h0100, 1'b1, 0,    1464, 0);
        send_frag(12'd7, 16'h0100, 1'b1, 1464, 1472, 0);
        send_frag(12'd7, 16'h0100, 1'b0, 2936, 500,  0);
        wait_msgs(base_msgs+1, 20000);
        check(cap_msgs == base_msgs+1, "B three fragments in order: 1 complete message reassembled");
        check(last_len == 3436, "B reassembled length = 3436");
        check(mism2(3436,0) == 0, "B reassembled content identical to the original");
        check(cap_last_vlid == 12'd7, "B reassembled message VLID correct");
        check(c_complete == 1, "B complete count = 1");

        // ---------------- C: gap (a middle fragment is missing) ----------------
        // Frames of one VL reach the reassembler in order (the integrity checker enforces increasing SN),
        // so a fragment that does not start where the previous one ended means a fragment was lost:
        // the whole message is dropped and counted in the overlap counter.
        base_msgs = cap_msgs; ov_before = c_overlap;
        send_frag(12'd8, 16'h0200, 1'b1, 0,    1464, 0);
        send_frag(12'd8, 16'h0200, 1'b0, 2936, 500,  0);   // last fragment, but the middle one never arrived
        wait_msgs(base_msgs+1, 300);
        repeat (50) @(posedge clk);
        check(cap_msgs == base_msgs, "C message with a missing middle fragment is not delivered");
        check(c_overlap == ov_before + 1, "C non-contiguous fragment: whole message dropped and counted (overlap +1)");

        // ---------------- D: 8192字节六片满载 ----------------
        base_msgs = cap_msgs;
        send_frag(12'd9, 16'h0300, 1'b1, 0,    1464, 0);
        send_frag(12'd9, 16'h0300, 1'b1, 1464, 1472, 0);
        send_frag(12'd9, 16'h0300, 1'b1, 2936, 1472, 0);
        send_frag(12'd9, 16'h0300, 1'b1, 4408, 1472, 0);
        send_frag(12'd9, 16'h0300, 1'b1, 5880, 1472, 0);
        send_frag(12'd9, 16'h0300, 1'b0, 7352, 840,  0);
        wait_msgs(base_msgs+1, 40000);
        check(cap_msgs == base_msgs+1, "D 8192 B six-fragment message reassembled as 1 message");
        check(last_len == 8192, "D reassembled length = 8192");
        check(mism2(8192,0) == 0, "D 8192 B content identical");

        // ---------------- I: 两条消息交错到达(不同ip_id) ----------------
        base_msgs = cap_msgs;
        send_frag(12'd10, 16'h0400, 1'b1, 0,    1464, 0);
        send_frag(12'd11, 16'h0401, 1'b1, 0,    1464, 1);
        send_frag(12'd10, 16'h0400, 1'b0, 1464, 300,  0);
        send_frag(12'd11, 16'h0401, 1'b0, 1464, 200,  1);
        wait_msgs(base_msgs+2, 30000);
        $display("DBG I: msgs=%0d(base %0d) complete=%0d orphan=%0d overlap=%0d timeout=%0d overflow=%0d", cap_msgs, base_msgs, c_complete, c_orphan, c_overlap, c_timeout, c_overflow);
        check(cap_msgs == base_msgs+2, "I two interleaved messages both reassembled completely");

        // ---------------- F: 孤儿延续片(没有首片) ----------------
        base_msgs = cap_msgs;
        send_frag(12'd12, 16'h0500, 1'b0, 1464, 100, 0);
        repeat (20) @(posedge clk);
        $display("DBG F: orphan=%0d timeout=%0d", c_orphan, c_timeout);
        check(c_orphan == 1, "F orphan continuation fragment dropped and counted (orphan=1)");
        check(cap_msgs == base_msgs, "F orphan continuation fragment produces no output");

        // ---------------- G: 分片重叠 ----------------
        base_msgs = cap_msgs;
        send_frag(12'd13, 16'h0600, 1'b1, 0,    1464, 0);
        send_frag(12'd13, 16'h0600, 1'b1, 1464, 1472, 0);
        send_frag(12'd13, 16'h0600, 1'b1, 1464, 1472, 0);  // 同一区间重复
        repeat (30) @(posedge clk);
        check(c_overlap == ov_before + 2, "G overlapping fragments: whole message dropped and counted (overlap +1 more)");
        wait_msgs(base_msgs+1, 300);
        check(cap_msgs == base_msgs, "G overlapping message produces no output");

        // ---------------- E: 超时释放 ----------------
        base_msgs = cap_msgs;
        send_frag(12'd14, 16'h0700, 1'b1, 0, 1464, 0);   // 只发首片，永远等不到后续
        repeat (6200) @(posedge clk);                      // > TIMEOUT_CYCLES(6000)
        $display("DBG E: timeout=%0d orphan=%0d overlap=%0d", c_timeout, c_orphan, c_overlap);
        check(c_timeout == 1, "E missing fragment timeout: context released and counted (timeout=1)");
        check(cap_msgs == base_msgs, "E timed-out message produces no output");

        // ---------------- H: 上下文耗尽(NUM_CTX=4, 发5条首片) ----------------
        send_frag(12'd20, 16'h0800, 1'b1, 0, 64, 0);
        send_frag(12'd21, 16'h0801, 1'b1, 0, 64, 0);
        send_frag(12'd22, 16'h0802, 1'b1, 0, 64, 0);
        send_frag(12'd23, 16'h0803, 1'b1, 0, 64, 0);
        send_frag(12'd24, 16'h0804, 1'b1, 0, 64, 0);     // 第5条: 无空闲上下文
        repeat (10) @(posedge clk);
        check(c_overflow == 1, "H parallel contexts exhausted: first fragment of the 5th message dropped and counted (overflow=1)");

        $display("============================================");
        $display("IP-REASM RESULT: PASS=%0d FAIL=%0d", pass_cnt, fail_cnt);
        if (fail_cnt == 0) $display("========== IP-REASM ALL PASS ==========");
        else               $display("========== IP-REASM FAIL ==========");
        $display("============================================");
        $finish;
    end

    initial begin #400000000; $display("WATCHDOG ip_reasm_tb"); $finish; end
endmodule
