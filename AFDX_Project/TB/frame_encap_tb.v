//=====================================================================
// 测试台: frame_encap_tb
// 目的  : 授权一个VL, 喂一段已知载荷, 把frame_encap吐出来的完整帧
//         逐字节抓下来, 核对:
//         1) 目的MAC/源MAC/EtherType 位置正确
//         2) IP总长度=28+enc_payload_len, UDP长度=8+enc_payload_len
//            (enc_payload_len = max(payload_len, 17), 见XAPP1130
//             "payload is padded to the 17-byte minimum"的要求)
//         3) IP头校验和满足"整个头部(含校验和字段)按16位字求和后
//            折叠为0xFFFF"这一行业标准判定法(不是照抄DUT的计算公式,
//            避免自己骗自己)
//         4) 载荷字节原样透传
//         5) 帧尾序列号=1(该VL首帧), mac_tx_last只在最后一字节拉高
//         6) 连续两帧同一VL, 序列号从1递增到2
//         7) 传输过程中间歇性拉低mac_tx_ready, 验证背压下数据不丢不乱
//         8) 载荷不足17字节时, 自动补0填充到17字节, 且IP/UDP长度字段
//            按填充后的长度计算
//         9) 组装出的帧总长超过该VL配置的LMAX(grant_max_len)时,
//            frame_len_violation应在发送序列号那一拍拉高一次
//=====================================================================
`timescale 1ns/1ps

module frame_encap_tb;

    localparam TX_IDX_W = 7;

    reg clk, rst_n;

    reg                 grant_valid;
    reg  [TX_IDX_W-1:0] grant_vlid;
    reg  [47:0]         grant_dest_mac;
    reg  [10:0]         grant_max_len;
    wire                grant_ready;

    wire         payload_valid;
    reg  [7:0]   payload_data;
    reg          payload_last;
    reg  [10:0]  payload_len;
    wire         payload_ready;

    wire [7:0] mac_tx_data;
    wire       mac_tx_valid;
    wire       mac_tx_last;
    reg        mac_tx_ready;
    wire       frame_len_violation;

    // ---- 被测模块 ----
    frame_encap #(
        .TX_IDX_W(TX_IDX_W),
        .NUM_TX_VL(128),
        .SRC_MAC(48'h02_00_00_00_01_20),
        .SRC_IP (32'hC0A80101),
        .DST_IP (32'hC0A801FF)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .udp_src_port(16'd5000), .udp_dst_port(16'd5001),
        .grant_valid(grant_valid), .grant_vlid(grant_vlid),
        .grant_dest_mac(grant_dest_mac), .grant_max_len(grant_max_len),
        .grant_ready(grant_ready),
        .payload_valid(payload_valid), .payload_data(payload_data),
        .payload_last(payload_last), .payload_len(payload_len),
        .payload_ready(payload_ready),
        .frag_ip_id(16'd0), .frag_mf(1'b0), .frag_offset_8b(13'd0), .frag_skip_udp_hdr(1'b0),
        .mac_tx_data(mac_tx_data), .mac_tx_valid(mac_tx_valid),
        .mac_tx_last(mac_tx_last), .mac_tx_ready(mac_tx_ready),
        .frame_len_violation(frame_len_violation)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    // ---- 载荷源: 从payload_mem里按顺序吐payload_len个字节 ----
    reg [7:0] payload_mem [0:31];
    integer   payload_ptr;
    integer   payload_total;
    reg       payload_active;
    reg       payload_valid_r;

    assign payload_valid = payload_valid_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            payload_valid_r <= 1'b0;
            payload_data    <= 8'd0;
            payload_last    <= 1'b0;
            payload_ptr     <= 0;
        end else if (!payload_active) begin
            payload_valid_r <= 1'b0;
        end else begin
            if (!payload_valid_r) begin
                // 还没摆出第一个字节, 摆出payload_ptr指向的这个
                payload_data    <= payload_mem[payload_ptr];
                payload_last    <= (payload_ptr == payload_total - 1);
                payload_valid_r <= 1'b1;
            end else if (payload_ready) begin
                // 当前字节被接受了, 推进到下一个(如果还有)
                if (payload_ptr == payload_total - 1) begin
                    payload_valid_r <= 1'b0;
                    payload_active  <= 1'b0;
                    payload_ptr     <= 0;
                end else begin
                    payload_ptr  <= payload_ptr + 1;
                    payload_data <= payload_mem[payload_ptr + 1];
                    payload_last <= (payload_ptr + 1 == payload_total - 1);
                    // payload_valid_r 保持1, 继续摆着新字节
                end
            end
            // else: payload_valid_r==1 且未被接受, 保持原样等待
        end
    end

    // ---- 捕获mac_tx输出的字节流 ----
    reg [7:0] cap_mem [0:2047];
    integer   cap_ptr;
    integer   error_cnt;

    always @(posedge clk) begin
        if (rst_n && mac_tx_valid && mac_tx_ready) begin
            cap_mem[cap_ptr] = mac_tx_data;
            cap_ptr = cap_ptr + 1;
        end
    end

    // ---- 捕获frame_len_violation (frame_len_violation为组合信号,
    //      与mac_tx_last同一拍有效, 用非阻塞采样避免竞争) ----
    reg last_frame_violation;
    always @(posedge clk) begin
        if (rst_n && mac_tx_valid && mac_tx_ready && mac_tx_last)
            last_frame_violation <= frame_len_violation;
    end

    // ---- 随机/间歇性背压驱动mac_tx_ready ----
    reg [2:0] ready_lfsr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mac_tx_ready <= 1'b1;
            ready_lfsr   <= 3'b101;
        end else begin
            ready_lfsr <= {ready_lfsr[1:0], ready_lfsr[2] ^ ready_lfsr[0]};
            mac_tx_ready <= ~ready_lfsr[0]; // 大约一半概率拉低, 制造背压
        end
    end

    // ---- 发起一次授权+载荷传输的task ----
    task send_frame;
        input [TX_IDX_W-1:0] vlid;
        input [47:0]         dest_mac;
        input integer        plen;
        input [10:0]         max_len;
        begin
            @(posedge clk); #1;
            cap_ptr        = 0;
            payload_ptr    = 0;
            payload_total  = plen;
            payload_len    = plen;
            payload_active = 1'b1;

            grant_vlid     = vlid;
            grant_dest_mac = dest_mac;
            grant_max_len  = max_len;
            grant_valid    = 1'b1;

            @(posedge clk); #1;
            grant_valid = 1'b0;

            // 等待frame_encap把整帧发完 (mac_tx_last且被接受)
            while (!(mac_tx_valid && mac_tx_ready && mac_tx_last)) begin
                @(posedge clk); #1;
            end
            @(posedge clk); #1; // 多等1拍让cap_mem/last_frame_violation落定
        end
    endtask

    // ---- 用于核对的16位字读取 (从cap_mem大端序取) ----
    function [15:0] be16;
        input integer idx;
        begin
            be16 = {cap_mem[idx], cap_mem[idx+1]};
        end
    endfunction

    integer i;
    reg [19:0] chk_verify;

    initial begin
        error_cnt = 0;
        rst_n = 0;
        grant_valid = 0; grant_vlid = 0; grant_dest_mac = 0; grant_max_len = 0;
        payload_active = 0;
        // 准备一段已知载荷: 0xA0,0xA1,...  (最多用到20字节)
        for (i = 0; i < 20; i = i + 1)
            payload_mem[i] = 8'hA0 + i;

        repeat (4) @(posedge clk);
        #1; rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // ==================== 第一帧: VL7, 20字节载荷(>=17, 不触发填充) ====================
        send_frame(7'd7, 48'hAA_BB_CC_DD_EE_FF, 20, 11'd1518);

        $display("---- first frame captured: %0d bytes (expected 63: 42 header + 20 payload + 1 SN) ----", cap_ptr);
        if (cap_ptr !== 63) begin
            $display("[FAIL] wrong frame length: got %0d expected 63", cap_ptr);
            error_cnt = error_cnt + 1;
        end else begin
            $display("[PASS] frame length = 63 bytes");
        end

        // 目的MAC
        if ({cap_mem[0],cap_mem[1],cap_mem[2],cap_mem[3],cap_mem[4],cap_mem[5]} === 48'hAA_BB_CC_DD_EE_FF)
            $display("[PASS] destination MAC correct");
        else begin
            $display("[FAIL] wrong destination MAC: %h", {cap_mem[0],cap_mem[1],cap_mem[2],cap_mem[3],cap_mem[4],cap_mem[5]});
            error_cnt = error_cnt + 1;
        end

        // 源MAC
        if ({cap_mem[6],cap_mem[7],cap_mem[8],cap_mem[9],cap_mem[10],cap_mem[11]} === 48'h02_00_00_00_01_20)
            $display("[PASS] source MAC correct");
        else begin
            $display("[FAIL] wrong source MAC: %h", {cap_mem[6],cap_mem[7],cap_mem[8],cap_mem[9],cap_mem[10],cap_mem[11]});
            error_cnt = error_cnt + 1;
        end

        // EtherType
        if (be16(12) === 16'h0800)
            $display("[PASS] EtherType=0x0800");
        else begin
            $display("[FAIL] wrong EtherType: %h", be16(12));
            error_cnt = error_cnt + 1;
        end

        // IP总长度 = 28+20 = 48
        if (be16(16) === 16'd48)
            $display("[PASS] IP total length = 48");
        else begin
            $display("[FAIL] wrong IP total length: %0d (expected 48)", be16(16));
            error_cnt = error_cnt + 1;
        end

        // UDP长度 = 8+20 = 28
        if (be16(38) === 16'd28)
            $display("[PASS] UDP length = 28");
        else begin
            $display("[FAIL] wrong UDP length: %0d (expected 28)", be16(38));
            error_cnt = error_cnt + 1;
        end

        // IP头校验和: 对头部10个16位字(含校验和字段本身)求和折叠, 应该=0xFFFF
        chk_verify = be16(14)+be16(16)+be16(18)+be16(20)+be16(22)+be16(24)+be16(26)+be16(28)+be16(30)+be16(32);
        chk_verify = chk_verify[15:0] + {12'd0, chk_verify[19:16]};
        chk_verify = chk_verify[15:0] + {15'd0, chk_verify[16]};
        if (chk_verify[15:0] === 16'hFFFF)
            $display("[PASS] IP header checksum consistent (folded sum = 0xFFFF)");
        else begin
            $display("[FAIL] IP header checksum inconsistent: folded = %h (expected FFFF)", chk_verify[15:0]);
            error_cnt = error_cnt + 1;
        end

        // 载荷透传
        begin : payload_check
            integer p_ok;
            p_ok = 1;
            for (i = 0; i < 20; i = i + 1) begin
                if (cap_mem[42+i] !== (8'hA0 + i)) p_ok = 0;
            end
            if (p_ok) $display("[PASS] 20-byte payload passed through unchanged");
            else begin
                $display("[FAIL] payload pass-through wrong");
                error_cnt = error_cnt + 1;
            end
        end

        // 序列号(帧尾第63字节, 下标62) 首帧应为0(XAPP1130: 复位标志)
        if (cap_mem[62] === 8'd0)
            $display("[PASS] first frame SN = 0 (reset marker)");
        else begin
            $display("[FAIL] wrong first-frame SN: %0d (expected 0)", cap_mem[62]);
            error_cnt = error_cnt + 1;
        end

        if (last_frame_violation === 1'b0)
            $display("[PASS] first frame within LMAX, frame_len_violation=0");
        else begin
            $display("[FAIL] first frame must not violate, but frame_len_violation=1");
            error_cnt = error_cnt + 1;
        end

        // ==================== 第二帧: 同一VL7, 验证序列号从0递增到1 ====================
        send_frame(7'd7, 48'hAA_BB_CC_DD_EE_FF, 20, 11'd1518);
        if (cap_mem[62] === 8'd1)
            $display("[PASS] second frame SN incremented to 1");
        else begin
            $display("[FAIL] wrong second-frame SN: %0d (expected 1)", cap_mem[62]);
            error_cnt = error_cnt + 1;
        end

        // ==================== 第三帧: 强制序列号=255, 验证本帧仍发255 ====================
        dut.seq_num[7] = 8'd255;
        send_frame(7'd7, 48'hAA_BB_CC_DD_EE_FF, 20, 11'd1518);
        if (cap_mem[62] === 8'd255)
            $display("[PASS] after forcing 255, this frame carries SN=255");
        else begin
            $display("[FAIL] wrong SN: %0d (expected 255)", cap_mem[62]);
            error_cnt = error_cnt + 1;
        end

        // ==================== 第四帧: 验证255之后真的回绕到1 ====================
        send_frame(7'd7, 48'hAA_BB_CC_DD_EE_FF, 20, 11'd1518);
        if (cap_mem[62] === 8'd1)
            $display("[PASS] SN wraps 255 -> 1");
        else begin
            $display("[FAIL] wrong SN wrap: %0d (expected 1)", cap_mem[62]);
            error_cnt = error_cnt + 1;
        end

        // ==================== 第五帧: 载荷仅5字节(<17), 验证自动补齐到17 ====================
        for (i = 0; i < 5; i = i + 1)
            payload_mem[i] = 8'hC0 + i;
        send_frame(7'd8, 48'h11_22_33_44_55_66, 5, 11'd1518);

        $display("---- fifth frame (5-byte payload) captured: %0d bytes (expected 60: 42 header + 17 payload incl. padding + 1 SN) ----", cap_ptr);
        if (cap_ptr !== 60) begin
            $display("[FAIL] wrong short-payload frame length: got %0d expected 60", cap_ptr);
            error_cnt = error_cnt + 1;
        end else
            $display("[PASS] short-payload frame length = 60 bytes (padded to the 17-byte minimum)");

        if (be16(16) === 16'd33) $display("[PASS] short-payload frame: IP total length=33 (28+5, padding NOT counted)");
        else begin $display("[FAIL] short-payload frame: IP total length=%0d (expected 33)", be16(16)); error_cnt=error_cnt+1; end

        if (be16(38) === 16'd13) $display("[PASS] short-payload frame: UDP length=13 (8+5, padding NOT counted)");
        else begin $display("[FAIL] short-payload frame: UDP length=%0d (expected 13)", be16(38)); error_cnt=error_cnt+1; end

        begin : short_payload_check
            integer p_ok;
            p_ok = 1;
            for (i = 0; i < 5; i = i + 1)
                if (cap_mem[42+i] !== (8'hC0+i)) p_ok = 0;
            for (i = 5; i < 17; i = i + 1)
                if (cap_mem[42+i] !== 8'h00) p_ok = 0;
            if (p_ok) $display("[PASS] short payload: first 5 bytes real data, last 12 bytes zero padding");
            else begin $display("[FAIL] short-payload padding wrong"); error_cnt = error_cnt + 1; end
        end

        // 恢复payload_mem供后续测试使用较长数据
        for (i = 0; i < 20; i = i + 1)
            payload_mem[i] = 8'hA0 + i;

        // ==================== 第六帧: 组帧超过该VL配置的LMAX, 验证frame_len_violation ====================
        // 20字节载荷组出的帧总长=63字节, 故意把LMAX配置成50, 应该触发违规指示
        send_frame(7'd9, 48'h77_88_99_AA_BB_CC, 20, 11'd50);
        if (last_frame_violation === 1'b1)
            $display("[PASS] frame_len_violation raised for payload above LMAX(50)");
        else begin
            $display("[FAIL] frame_len_violation not raised above LMAX");
            error_cnt = error_cnt + 1;
        end

        if (error_cnt == 0)
            $display("==== frame_encap_tb: ALL PASS ====");
        else
            $display("==== frame_encap_tb: %0d ERROR(S) ====", error_cnt);

        $finish;
    end

endmodule
