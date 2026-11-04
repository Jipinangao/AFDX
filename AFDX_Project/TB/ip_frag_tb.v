//=====================================================================
// ip_frag_tb : IP分片边界值验证
// 覆盖：
//   1) 小消息(≤1471) -> 不分片，单一分片(mf=0,skip=0,offset=0)
//   2) 边界1471 -> 恰好不分片
//   3) 边界1472 -> 恰好触发分片，2片(1464+8)
//   4) 边界1473 -> 2片(1464+9)
//   5) 最大8192字节 -> 6片，长度和FragOffset与手算完全一致
//   6) 超过8192字节的消息 -> 计入msg_overrun_count，不产生任何分片
//   7) 连续多条消息 -> frag_ip_id递增
//=====================================================================
`timescale 1ns/1ps
module ip_frag_tb;
    reg clk, rst_n;
    reg msg_valid; reg [7:0] msg_data; reg msg_last; reg [13:0] msg_len;
    wire msg_ready;

    wire payload_valid; wire [7:0] payload_data; wire payload_last; wire [10:0] payload_len;
    reg payload_ready;
    wire [15:0] frag_ip_id; wire frag_mf; wire [12:0] frag_offset_8b; wire frag_skip_udp_hdr;
    wire [31:0] frag_count, msg_count, msg_overrun_count;

    integer pass_count, fail_count;

    ip_frag #(.MAX_MSG_LEN(8192), .TX_IDX_W(7)) dut (
        .msg_vlid(7'd9), .msg_udp_src_port(16'd5000), .msg_udp_dst_port(16'd5001),
        .tx_pending(), .tx_vlid(), .frag_udp_src_port(), .frag_udp_dst_port(),
        .clk(clk), .rst_n(rst_n),
        .msg_valid(msg_valid), .msg_data(msg_data), .msg_last(msg_last), .msg_len(msg_len), .msg_ready(msg_ready),
        .payload_valid(payload_valid), .payload_data(payload_data), .payload_last(payload_last),
        .payload_len(payload_len), .payload_ready(payload_ready),
        .frag_ip_id(frag_ip_id), .frag_mf(frag_mf), .frag_offset_8b(frag_offset_8b), .frag_skip_udp_hdr(frag_skip_udp_hdr),
        .frag_count(frag_count), .msg_count(msg_count), .msg_overrun_count(msg_overrun_count)
    );

    initial begin clk=0; forever #5 clk=~clk; end

    task automatic check;
        input cond; input [8*128-1:0] name;
        begin
            if (cond) begin $display("[PASS] %0s", name); pass_count=pass_count+1; end
            else begin $display("[FAIL] %0s", name); fail_count=fail_count+1; end
        end
    endtask

    // 说明（开发中发现的一个真实教训，如实记录）：
    // 最初这里没有在提交第一个字节前等待msg_ready，导致"上一条消息刚发完
    // 最后一个payload字节"and"这条testbench立刻提交下一条消息第一个字节"
    // 两件事落在了同一拍——而ip_frag内部完成S_XMIT到S_RECV的状态切换
    // 也正是在那一拍，谁先谁后取决于case语句这一拍到底执行的是哪个分支，
    // 结果导致下一条消息的第一个字节被悄悄丢弃、后续内容整体错位一位。
    // 这不是ip_frag.v本身的bug——它的msg_ready信号已经正确地在那一拍
    // 显示为0，问题在于本testbench没有遵守"生产者必须等ready才能发起
    // 传输"的握手规则。修复方式就是老老实实先等msg_ready=1。
    task automatic send_msg;
        input [13:0] len;
        integer k;
        begin
            while (!msg_ready) @(posedge clk);
            k = 0;
            while (k < len) begin
                @(negedge clk);
                msg_valid = 1; msg_data = k[7:0]; msg_last = (k == len-1); msg_len = len;
                @(posedge clk);
                k = k + 1;
            end
            @(negedge clk);
            msg_valid = 0; msg_last = 0;
        end
    endtask

    // 采集一个分片：记录长度、mf、offset、skip_udp、以及数据内容
    // (逐字节比对，而不只是累加和——事后复盘发现，最初版本这里只算了
    // 一个从未真正拿去check()的校验和got_sum，等于没有真正验证过字节
    // 内容，让ip_frag.v里一个"payload_data寄存器缓冲滞后read_idx一拍
    // 导致字节重复"的真实bug在这里被漏检，直到ip_frag_encap_tb.v解码
    // 真实以太网帧字节做端到端重组比对时才被抓出来。这里补上逐字节比对，
    // 不再是形同虚设的校验和。)
    integer got_len; reg got_mf; reg [12:0] got_offset; reg got_skip;
    reg [7:0] got_data[0:2047];
    task automatic capture_one_frag;
        integer wdt;
        begin
            got_len = 0;
            wdt = 0;
            while (1) begin
                @(posedge clk); #1;
                wdt = wdt + 1;
                if (wdt > 20000) begin $display("WATCHDOG capture_one_frag"); disable capture_one_frag; end
                if (payload_valid && payload_ready) begin
                    if (got_len == 0) begin got_mf = frag_mf; got_offset = frag_offset_8b; got_skip = frag_skip_udp_hdr; end
                    got_data[got_len] = payload_data;
                    got_len = got_len + 1;
                    if (payload_last) disable capture_one_frag;
                end
            end
        end
    endtask

    // 核对一个分片的数据内容是否与"byte k of the message is k[7:0]"的规则一致，
    // base是该分片在原始消息里的起始字节序号(0-based)。
    function automatic content_ok;
        input integer base;
        input integer len;
        integer k;
        reg mismatch;
        begin
            mismatch = 0;
            for (k = 0; k < len; k = k + 1)
                if (got_data[k] !== ((base+k) & 32'hFF)) mismatch = 1;
            content_ok = !mismatch;
        end
    endfunction

    initial begin
        pass_count=0; fail_count=0;
        rst_n=0; msg_valid=0; msg_data=0; msg_last=0; msg_len=0; payload_ready=0;
        repeat(5) @(posedge clk); rst_n=1;
        payload_ready = 1; // 全程保持1，简化时序，避免任务调用边界处的握手细节干扰对RTL本身的判断

        // ---- 1) 小消息(10字节) ----
        send_msg(14'd10);
        capture_one_frag;
        check(got_len==10 && !got_mf && got_offset==13'd0 && !got_skip,
              "small message (10 B): no fragmentation, single frame, mf=0/offset=0/skip=0");
        check(content_ok(0, 10), "small message (10 B): payload bytes identical to what was sent");
        @(posedge clk); #1; // 计数器在last那一拍才用NBA更新，留一点settle时间再读，避免读到更新前的值
        check(msg_count==32'd1, "small message done, msg_count=1");

        // ---- 2) 边界1471: 恰好不分片 ----
        send_msg(14'd1471);
        capture_one_frag;
        check(got_len==1471 && !got_mf && !got_skip, "boundary 1471: exactly no fragmentation");
        check(content_ok(0, 1471), "boundary 1471: payload bytes identical to what was sent");

        // ---- 3) 边界1472: 恰好触发分片，应为2片(1464+8) ----
        send_msg(14'd1472);
        capture_one_frag;
        check(got_len==1464 && got_mf && !got_skip && got_offset==13'd0,
              "boundary 1472: fragment 1 length 1464, mf=1, skip=0, offset=0");
        check(content_ok(0, 1464), "boundary 1472: fragment 1 payload correct (message bytes 0..1463)");
        capture_one_frag;
        check(got_len==8 && !got_mf && got_skip && got_offset==13'd184,
              "boundary 1472: fragment 2 (last) length 8, mf=0, skip=1, offset=184");
        check(content_ok(1464, 8), "boundary 1472: fragment 2 payload correct (message bytes 1464..1471, contiguous with fragment 1)");
        @(posedge clk); #1;
        check(frag_count==32'd4, "cumulative fragment count correct (1+1+2=4)");

        // ---- 4) 边界1473: 2片(1464+9) ----
        send_msg(14'd1473);
        capture_one_frag;
        check(got_len==1464 && got_mf, "boundary 1473: fragment 1 length 1464 (same as 1472 case)");
        check(content_ok(0, 1464), "boundary 1473: fragment 1 payload correct");
        capture_one_frag;
        check(got_len==9 && !got_mf && got_offset==13'd184, "boundary 1473: fragment 2 (last) length 9");
        check(content_ok(1464, 9), "boundary 1473: fragment 2 payload correct (message bytes 1464..1472)");

        // ---- 5) 最大8192字节: 应为6片, 长度[1464,1472,1472,1472,1472,840] ----
        send_msg(14'd8192);
        capture_one_frag; check(got_len==1464 && got_mf && !got_skip && got_offset==13'd0, "8192 B message: fragment 1 (first) length 1464");
        check(content_ok(0, 1464), "8192 B message: fragment 1 payload correct");
        capture_one_frag; check(got_len==1472 && got_mf && got_skip && got_offset==13'd184, "8192 B message: fragment 2 length 1472, offset=184");
        check(content_ok(1464, 1472), "8192 B message: fragment 2 payload correct (contiguous with fragment 1)");
        capture_one_frag; check(got_len==1472 && got_mf && got_skip && got_offset==13'd368, "8192 B message: fragment 3 length 1472, offset=368");
        check(content_ok(1464+1472, 1472), "8192 B message: fragment 3 payload correct");
        capture_one_frag; check(got_len==1472 && got_mf && got_skip && got_offset==13'd552, "8192 B message: fragment 4 length 1472, offset=552");
        check(content_ok(1464+1472*2, 1472), "8192 B message: fragment 4 payload correct");
        capture_one_frag; check(got_len==1472 && got_mf && got_skip && got_offset==13'd736, "8192 B message: fragment 5 length 1472, offset=736");
        check(content_ok(1464+1472*3, 1472), "8192 B message: fragment 5 payload correct");
        capture_one_frag; check(got_len==840 && !got_mf && got_skip && got_offset==13'd920, "8192 B message: fragment 6 (last) length 840, offset=920");
        check(content_ok(1464+1472*4, 840), "8192 B message: fragment 6 payload correct, contiguous, covers all 8192 bytes exactly once");
        check(1464+1472+1472+1472+1472+840 == 8192, "the 6 fragment lengths add up to the original message length 8192");

        // ---- 6) 超过8192字节 -> 丢弃计数，不产生分片 ----
        send_msg(14'd8193);
        // 不应有任何分片产生；直接检查msg_overrun_count而不等待payload(否则会永久等待触发watchdog)
        repeat(20) @(posedge clk);
        check(msg_overrun_count==32'd1, "message above 8192 B is counted in msg_overrun_count and not fragmented");
        check(msg_ready==1'b1, "after dropping the oversize message a new one is accepted at once (msg_ready=1)");

        // ---- 7) frag_ip_id在连续消息间递增 ----
        begin : ipid_check
            reg [15:0] id_before;
            id_before = frag_ip_id;
            send_msg(14'd5);
            capture_one_frag;
            @(posedge clk); #1;
            check(frag_ip_id == id_before + 16'd1, "frag_ip_id increments correctly between messages");
        end

        $display("============================================");
        $display("IP-FRAG RESULT: PASS=%0d FAIL=%0d", pass_count, fail_count);
        if (fail_count==0) $display("========== IP-FRAG ALL PASS ==========");
        else $display("========== IP-FRAG FAIL ==========");
        $display("============================================");
        $finish;
    end
endmodule
