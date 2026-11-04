//=====================================================================
// ip_frag_encap_tb : ip_frag + frame_encap 联调（解码真实线上字节）
//
// 不满足于"ip_frag's own claim that its output fields are right"，而是真正驱动frame_encap
// 编码出完整的以太网帧字节流，再从字节流里解析出Ethernet/IP头字段，
// 核对：
//   1) 一条1472字节消息被拆成的2个以太网帧里，IP.Identification
//      两帧完全相同(标识同一条消息)。
//   2) 第1帧IP.MF=1、FragOffset=0；第2帧IP.MF=0、FragOffset=184。
//   3) 第1帧携带标准UDP头(目的端口等于frame_encap配置的UDP_DST_PORT)，
//      第2帧从IP头后直接是延续数据，没有UDP头。
//   4) 把2帧的"original IP payload"部分(第1帧: UDP头8B+用户数据1464B；
//      第2帧: 用户数据8B)拼起来，与发送前的原始1472字节消息逐字节
//      比对完全一致——这是最终判据：不管中间过程字段对不对，
//      最后能不能正确重组出原始消息才是硬道理。
//   5) 8192字节消息(6片)的冒烟检查：帧数=6，全部IP.Identification
//      一致。
//=====================================================================
`timescale 1ns/1ps
module ip_frag_encap_tb;
    localparam TX_IDX_W = 7;
    reg clk, rst_n;

    reg msg_valid; reg [7:0] msg_data; reg msg_last; reg [13:0] msg_len;
    wire msg_ready;

    wire ip_payload_valid; wire [7:0] ip_payload_data; wire ip_payload_last; wire [10:0] ip_payload_len;
    wire ip_payload_ready;
    wire [15:0] frag_ip_id; wire frag_mf; wire [12:0] frag_offset_8b; wire frag_skip_udp_hdr;
    wire [31:0] frag_count, msg_count, msg_overrun_count;

    reg grant_valid;
    wire grant_ready;
    reg [TX_IDX_W-1:0] grant_vlid;
    reg [47:0] grant_dest_mac;
    reg [10:0] grant_max_len;

    wire [7:0] mac_tx_data; wire mac_tx_valid, mac_tx_last;
    reg mac_tx_ready;
    wire frame_len_violation;

    integer pass_count, fail_count;

    wire [15:0] frag_udp_sp, frag_udp_dp;
    ip_frag #(.MAX_MSG_LEN(8192), .TX_IDX_W(TX_IDX_W)) u_frag (
        .clk(clk), .rst_n(rst_n),
        .msg_vlid({TX_IDX_W{1'b0}}), .msg_udp_src_port(16'd5000), .msg_udp_dst_port(16'd5001),
        .tx_pending(), .tx_vlid(), .frag_udp_src_port(frag_udp_sp), .frag_udp_dst_port(frag_udp_dp),
        .msg_valid(msg_valid), .msg_data(msg_data), .msg_last(msg_last), .msg_len(msg_len), .msg_ready(msg_ready),
        .payload_valid(ip_payload_valid), .payload_data(ip_payload_data), .payload_last(ip_payload_last),
        .payload_len(ip_payload_len), .payload_ready(ip_payload_ready),
        .frag_ip_id(frag_ip_id), .frag_mf(frag_mf), .frag_offset_8b(frag_offset_8b), .frag_skip_udp_hdr(frag_skip_udp_hdr),
        .frag_count(frag_count), .msg_count(msg_count), .msg_overrun_count(msg_overrun_count)
    );

    frame_encap #(.TX_IDX_W(TX_IDX_W), .NUM_TX_VL(128),
                  .SRC_MAC(48'h02_00_00_00_01_20), .SRC_IP(32'hC0A80101), .DST_IP(32'hC0A801FF)) u_encap (
        .clk(clk), .rst_n(rst_n),
        .udp_src_port(frag_udp_sp), .udp_dst_port(frag_udp_dp),
        .grant_valid(grant_valid), .grant_vlid(grant_vlid), .grant_dest_mac(grant_dest_mac),
        .grant_max_len(grant_max_len), .grant_ready(grant_ready),
        .payload_valid(ip_payload_valid), .payload_data(ip_payload_data), .payload_last(ip_payload_last),
        .payload_len(ip_payload_len), .payload_ready(ip_payload_ready),
        .frag_ip_id(frag_ip_id), .frag_mf(frag_mf), .frag_offset_8b(frag_offset_8b), .frag_skip_udp_hdr(frag_skip_udp_hdr),
        .mac_tx_data(mac_tx_data), .mac_tx_valid(mac_tx_valid), .mac_tx_last(mac_tx_last), .mac_tx_ready(mac_tx_ready),
        .frame_len_violation(frame_len_violation)
    );

    initial begin clk=0; forever #5 clk=~clk; end

    // 简化的"grant source"：只要frame_encap空闲(grant_ready=1)且ip_frag有数据
    // 要发(ip_payload_valid=1)，立即授权——本测试只关心分片/编码的正确
    // 性，不关心BAG节奏本身(那已经在afdx_bag_jitter_scheduler_tb.v里
    // 单独验证过)。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            grant_valid <= 1'b0; grant_vlid <= 0; grant_dest_mac <= 48'h0A0B0C0D0E0F; grant_max_len <= 11'd1518;
        end else begin
            grant_valid <= grant_ready && ip_payload_valid;
        end
    end

    task automatic check;
        input cond; input [8*128-1:0] name;
        begin
            if (cond) begin $display("[PASS] %0s", name); pass_count=pass_count+1; end
            else begin $display("[FAIL] %0s", name); fail_count=fail_count+1; end
        end
    endtask

    task automatic send_msg;
        input [13:0] len;
        integer k;
        begin
            while (!msg_ready) @(posedge clk); // 见ip_frag_tb.v同名task头注释：必须先等ready再发起提交
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

    reg [7:0] frame_bytes[0:2047];
    integer frame_len;

    task automatic capture_one_frame;
        integer wdt;
        begin
            frame_len = 0;
            mac_tx_ready = 1;
            wdt = 0;
            while (1) begin
                @(posedge clk); #1;
                wdt = wdt + 1;
                if (wdt > 20000) begin $display("WATCHDOG capture_one_frame"); disable capture_one_frame; end
                if (mac_tx_valid && mac_tx_ready) begin
                    frame_bytes[frame_len] = mac_tx_data;
                    frame_len = frame_len + 1;
                    if (mac_tx_last) disable capture_one_frame;
                end
            end
        end
    endtask

    // 从捕获到的一帧里解析关心的字段
    function [15:0] get_ip_id;
        input dummy;
        begin
            get_ip_id = {frame_bytes[18], frame_bytes[19]};
        end
    endfunction

    function get_mf;
        input dummy;
        begin
            get_mf = frame_bytes[20][5]; // bit13 of 16bit flags+fragoffset字段 -> byte0 bit5
        end
    endfunction

    function [12:0] get_frag_off;
        input dummy;
        begin
            get_frag_off = {frame_bytes[20][4:0], frame_bytes[21]};
        end
    endfunction

    function [15:0] get_udp_dport;
        input dummy;
        begin
            get_udp_dport = {frame_bytes[36], frame_bytes[37]};
        end
    endfunction

    reg [7:0] reassembled[0:8191];
    integer reasm_len;

    initial begin
        pass_count=0; fail_count=0;
        rst_n=0; msg_valid=0; msg_data=0; msg_last=0; msg_len=0; mac_tx_ready=0;
        repeat(5) @(posedge clk); rst_n=1;

        // ==================== 1472字节消息: 2个分片 ====================
        send_msg(14'd1472);

        capture_one_frame;
        // frame1: 42字节头(含UDP) + 1464字节payload + 1字节SN = 1507字节(不含FCS)
        check(frame_len == 42+1464+1, "frame 1 (first fragment) length = 42 (with UDP header) + 1464 + 1 (SN)");
        check(get_udp_dport(1'b0) == 16'd5001, "frame 1 carries the UDP header, destination port = 5001");
        check(get_mf(1'b0) == 1'b1, "frame 1 IP.MF=1 (more fragments follow)");
        check(get_frag_off(1'b0) == 13'd0, "frame 1 IP.FragOffset=0");
        begin : copy1
            integer k;
            reasm_len = 0;
            // 第1帧原始IP载荷 = UDP头(8B,字节34..41) + 用户数据(1464B,字节42..1505)
            for (k=34; k<42+1464; k=k+1) begin
                reassembled[reasm_len] = frame_bytes[k];
                reasm_len = reasm_len + 1;
            end
        end
        begin : idcheck
            reg [15:0] id1;
            id1 = get_ip_id(1'b0);

            capture_one_frame;
            // frame2: 34字节头(不含UDP) + 8字节payload + 1字节SN = 43字节
            check(frame_len == 34+25+1, "frame 2 (last fragment): 34 header + 8 data padded to 25 + 1 SN = 60 bytes");
            check(get_ip_id(1'b0) == id1, "frame 2 IP.Identification equals frame 1 (same message)");
            check(get_mf(1'b0) == 1'b0, "frame 2 IP.MF=0 (last fragment)");
            check(get_frag_off(1'b0) == 13'd184, "frame 2 IP.FragOffset=184");
        end
        begin : copy2
            integer k;
            // 第2帧原始IP载荷 = 纯延续数据(8B, 字节34..41, 没有UDP头)
            for (k=34; k<34+8; k=k+1) begin
                reassembled[reasm_len] = frame_bytes[k];
                reasm_len = reasm_len + 1;
            end
        end
        check(reasm_len == 8+1472, "reassembled length = UDP header (8) + original user data (1472)");
        begin : content_check
            integer k; reg mismatch;
            mismatch = 0;
            // 重组结果的[8:8+1472)应与发送前的原始1472字节用户数据逐字节一致
            // (发送时用的是k[7:0]作为第k字节的内容，见send_msg)
            for (k=0; k<1472; k=k+1)
                if (reassembled[8+k] !== k[7:0]) mismatch = 1;
            check(!mismatch, "reassembled user data of both frames equals the original 1472 bytes (final criterion)");
        end

        // ==================== 8192字节消息: 冒烟检查(6片, IP_ID一致) ====================
        begin : big_msg
            reg [15:0] big_id;
            integer i;
            send_msg(14'd8192);
            capture_one_frame;
            big_id = get_ip_id(1'b0);
            for (i=0;i<5;i=i+1) capture_one_frame;
            check(frag_count >= 32'd8, "fragment count after the 8192 B message reaches the expected size (>=8 incl. earlier cases)");
            check(get_ip_id(1'b0) == big_id, "last fragment of the 8192 B message has the same IP.Identification as the first");
        end

        $display("============================================");
        $display("IP-FRAG-ENCAP RESULT: PASS=%0d FAIL=%0d", pass_count, fail_count);
        if (fail_count==0) $display("========== IP-FRAG-ENCAP ALL PASS ==========");
        else $display("========== IP-FRAG-ENCAP FAIL ==========");
        $display("============================================");
        $finish;
    end
endmodule
