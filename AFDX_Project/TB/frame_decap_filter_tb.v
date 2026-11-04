//=====================================================================
// 测试台: frame_decap_filter_tb
// 目的  : A) 用vl_table+bag_scheduler+frame_encap真实产生一帧, 直接环回
//            接到frame_decap_filter的输入, 验证VLID/载荷/序列号能被
//            正确解出, frame_error=0
//         B) 手工构造几个刻意做错的帧, 逐一戳中每种错误检测路径:
//            以太网类型错/IP校验和错/VL未配置/载荷长度超范围/帧截断
//=====================================================================
`timescale 1ns/1ps

module frame_decap_filter_tb;

    localparam TX_IDX_W = 7;
    localparam RX_IDX_W = 12;
    localparam [31:0] OUI = 32'h0300_0000;

    reg clk, rst_n;
    integer error_cnt;

    // ==================== A部分: 真实TX链路 ====================
    reg  [15:0] reg_addr;  reg [31:0] reg_wdata; reg reg_we, reg_re;
    wire [31:0] reg_rdata; wire reg_rdy;

    wire s_tx_lookup_valid; wire [TX_IDX_W-1:0] s_tx_lookup_vlid; wire s_tx_lookup_rdy;
    wire [2:0] s_tx_bag_sel; wire [10:0] s_tx_max_len; wire [47:0] s_tx_dest_mac; wire s_tx_entry_valid;

    wire s_rx_lookup_valid; wire [RX_IDX_W-1:0] s_rx_lookup_vlid; wire s_rx_lookup_rdy;
    wire [10:0] s_rx_min_len; wire [10:0] s_rx_max_len; wire s_rx_entry_valid;

    reg  vl_req_v; reg [6:0] vl_req_id;
    wire grant_valid; wire [TX_IDX_W-1:0] grant_vlid;
    wire [47:0] grant_dest_mac; wire [10:0] grant_max_len;
    wire grant_ready_w;

    wire p_valid, p_ready; wire [7:0] p_data; wire p_last; wire [10:0] p_len;
    wire [7:0] enc_tx_data; wire enc_tx_valid, enc_tx_last;
    wire enc_frame_len_violation;

    wire vlt_init_done;
    vl_table #(.NUM_TX_VL(128), .NUM_RX_VL(4096), .TX_IDX_W(TX_IDX_W), .RX_IDX_W(RX_IDX_W)) u_vl_table (
        .clk(clk), .rst_n(rst_n),
        .reg_addr(reg_addr), .reg_wdata(reg_wdata), .reg_we(reg_we), .reg_re(reg_re),
        .reg_rdata(reg_rdata), .reg_rdy(reg_rdy),
        .tx_lookup_valid(s_tx_lookup_valid), .tx_lookup_vlid(s_tx_lookup_vlid), .tx_lookup_rdy(s_tx_lookup_rdy),
        .tx_bag_sel(s_tx_bag_sel), .tx_max_len(s_tx_max_len), .tx_dest_mac(s_tx_dest_mac), .tx_entry_valid(s_tx_entry_valid),
        .rx_lookup_valid(s_rx_lookup_valid), .rx_lookup_vlid(s_rx_lookup_vlid), .rx_lookup_rdy(s_rx_lookup_rdy),
        .rx_min_len(s_rx_min_len), .rx_max_len(s_rx_max_len), .rx_entry_valid(s_rx_entry_valid), .init_done(vlt_init_done)
    );

    // CLK_FREQ_HZ=256000 compresses 1 ms to 256 cycles to keep this unit test fast
    afdx_bag_jitter_scheduler #(.NUM_TX_VL(128), .TX_IDX_W(TX_IDX_W), .CLK_FREQ_HZ(256000)) u_bag_scheduler (
        .clk(clk), .rst_n(rst_n), .req_valid(vl_req_v), .req_vlid(vl_req_id), .jitter_max_us(10'd500), .jitter_violation(),
        .tx_lookup_valid(s_tx_lookup_valid), .tx_lookup_vlid(s_tx_lookup_vlid), .tx_lookup_rdy(s_tx_lookup_rdy),
        .tx_bag_sel(s_tx_bag_sel), .tx_max_len(s_tx_max_len), .tx_dest_mac(s_tx_dest_mac), .tx_entry_valid(s_tx_entry_valid),
        .grant_valid(grant_valid), .grant_vlid(grant_vlid), .grant_dest_mac(grant_dest_mac),
        .grant_max_len(grant_max_len), .grant_ready(grant_ready_w)
    );

    frame_encap #(.TX_IDX_W(TX_IDX_W), .NUM_TX_VL(128),
                  .SRC_MAC(48'h02_00_00_00_00_01), .SRC_IP(32'hC0A80101), .DST_IP(32'hC0A801FF)) u_frame_encap (
        .clk(clk), .rst_n(rst_n), .udp_src_port(16'd5000), .udp_dst_port(16'd5001),
        .grant_valid(grant_valid), .grant_vlid(grant_vlid), .grant_dest_mac(grant_dest_mac),
        .grant_max_len(grant_max_len), .grant_ready(grant_ready_w),
        .payload_valid(p_valid), .payload_data(p_data), .payload_last(p_last), .payload_len(p_len),
        .payload_ready(p_ready),
        .frag_ip_id(16'd0), .frag_mf(1'b0), .frag_offset_8b(13'd0), .frag_skip_udp_hdr(1'b0),
        .mac_tx_data(enc_tx_data), .mac_tx_valid(enc_tx_valid), .mac_tx_last(enc_tx_last), .mac_tx_ready(1'b1),
        .frame_len_violation(enc_frame_len_violation)
    );

    // payload源 (跟frame_encap_tb同款写法)
    reg [7:0] payload_mem [0:15];
    integer   payload_ptr, payload_total;
    reg       payload_active, payload_valid_r;
    assign p_valid = payload_valid_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            payload_valid_r <= 0; payload_ptr <= 0;
        end else if (!payload_active) begin
            payload_valid_r <= 0;
        end else if (!payload_valid_r) begin
            payload_valid_r <= 1;
        end else if (p_ready) begin
            if (payload_ptr == payload_total-1) begin
                payload_valid_r <= 0; payload_active <= 0; payload_ptr <= 0;
            end else begin
                payload_ptr <= payload_ptr + 1;
            end
        end
    end
    assign p_data = payload_mem[payload_ptr];
    assign p_last = (payload_ptr == payload_total-1);
    assign p_len  = payload_total;

    // ==================== 被测模块: frame_decap_filter ====================
    reg  mac_rx_data_ovr_en;   // B部分手工注入时占用mac_rx总线
    reg  [7:0] mrx_data; reg mrx_valid, mrx_last, mrx_err;
    wire [7:0] dec_mac_rx_data  = mac_rx_data_ovr_en ? mrx_data  : enc_tx_data;
    wire       dec_mac_rx_valid = mac_rx_data_ovr_en ? mrx_valid : enc_tx_valid;
    wire       dec_mac_rx_last  = mac_rx_data_ovr_en ? mrx_last  : enc_tx_last;
    wire       dec_mac_rx_err   = mac_rx_data_ovr_en ? mrx_err   : 1'b0;

    wire dec_rx_lookup_valid; wire [RX_IDX_W-1:0] dec_rx_lookup_vlid; reg dec_rx_lookup_rdy_dummy;
    wire out_p_valid; wire [7:0] out_p_data; wire out_p_last;
    wire frame_done; wire [RX_IDX_W-1:0] frame_vlid; wire [7:0] frame_seq_num;
    wire [10:0] frame_payload_len; wire frame_error;

    // ---- C部分用: 给rx_lookup接口注入"configurable delay"的查表响应, 用来
    //      验证frame_decap_filter对查表延迟的鲁棒性(不会用陈旧数据)。
    //      平时(C部分之外)直接透传共用vl_table的真实结果。 ----
    reg         rx_lookup_ovr_en;
    integer     rx_lookup_ovr_latency;
    reg         ovr_pending;
    integer     ovr_cnt;
    reg         ovr_rdy; reg [10:0] ovr_min_len, ovr_max_len; reg ovr_entry_valid;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ovr_pending <= 1'b0; ovr_cnt <= 0; ovr_rdy <= 1'b0;
        end else begin
            ovr_rdy <= 1'b0;
            if (dec_rx_lookup_valid && rx_lookup_ovr_en && !ovr_pending) begin
                ovr_pending <= 1'b1;
                ovr_cnt     <= rx_lookup_ovr_latency;
            end else if (ovr_pending) begin
                if (ovr_cnt == 0) begin
                    ovr_rdy     <= 1'b1;
                    ovr_pending <= 1'b0;
                end else begin
                    ovr_cnt <= ovr_cnt - 1;
                end
            end
        end
    end
    wire dut_rx_lookup_rdy   = rx_lookup_ovr_en ? ovr_rdy        : s_rx_lookup_rdy;
    wire [10:0] dut_rx_min_len = rx_lookup_ovr_en ? ovr_min_len    : s_rx_min_len;
    wire [10:0] dut_rx_max_len = rx_lookup_ovr_en ? ovr_max_len    : s_rx_max_len;
    wire        dut_rx_entry_valid = rx_lookup_ovr_en ? ovr_entry_valid : s_rx_entry_valid;

    // decap的rx_lookup接口平时接到共用的vl_table, B部分注入期间也接同一张表(共用RX配置);
    // C部分则切到上面的可配置延迟响应器
    frame_decap_filter #(.RX_IDX_W(RX_IDX_W), .EXPECTED_OUI(OUI)) dut (
        .clk(clk), .rst_n(rst_n),
        .mac_rx_data(dec_mac_rx_data), .mac_rx_valid(dec_mac_rx_valid),
        .mac_rx_last(dec_mac_rx_last), .mac_rx_err(dec_mac_rx_err),
        .rx_lookup_valid(dec_rx_lookup_valid), .rx_lookup_vlid(dec_rx_lookup_vlid),
        .rx_lookup_rdy(dut_rx_lookup_rdy), .rx_min_len(dut_rx_min_len),
        .rx_max_len(dut_rx_max_len), .rx_entry_valid(dut_rx_entry_valid),
        .out_payload_valid(out_p_valid), .out_payload_data(out_p_data), .out_payload_last(out_p_last),
        .frame_done(frame_done), .frame_vlid(frame_vlid), .frame_seq_num(frame_seq_num),
        .frame_payload_len(frame_payload_len), .frame_error(frame_error),
        .frame_udp_src_port(), .frame_udp_dst_port()
    );
    assign s_rx_lookup_valid = dec_rx_lookup_valid;
    assign s_rx_lookup_vlid  = dec_rx_lookup_vlid;

    initial clk = 0; always #5 clk = ~clk;

    // 捕获decap输出的载荷
    reg [7:0] cap_out [0:63];
    integer   cap_out_ptr;
    always @(posedge clk) begin
        if (rst_n && out_p_valid) begin
            cap_out[cap_out_ptr] = out_p_data;
            cap_out_ptr = cap_out_ptr + 1;
        end
    end

    // ---- vl_table写配置任务 (跟之前一致的防竞争写法) ----
    task write_tx_entry;
        input [TX_IDX_W-1:0] vlid; input [2:0] bag_sel; input valid;
        input [10:0] max_len; input [47:0] dest_mac;
        reg [31:0] lo_word, hi_word;
        begin
            lo_word = {dest_mac[16:0], max_len, valid, bag_sel};
            hi_word = {1'b0, dest_mac[47:17]};
            @(posedge clk); #1; reg_addr={1'b0,vlid,1'b0}; reg_wdata=lo_word; reg_we=1;
            @(posedge clk); #1; reg_we=0;
            @(posedge clk); #1; reg_addr={1'b0,vlid,1'b1}; reg_wdata=hi_word; reg_we=1;
            @(posedge clk); #1; reg_we=0;
        end
    endtask

    task write_rx_entry;
        input [RX_IDX_W-1:0] vlid; input [10:0] min_len; input [10:0] max_len; input valid;
        begin
            @(posedge clk); #1;
            reg_addr = {1'b1, {(16-1-RX_IDX_W){1'b0}}, vlid};
            reg_wdata = {9'd0, valid, max_len, min_len};
            reg_we = 1;
            @(posedge clk); #1; reg_we = 0;
        end
    endtask

    // ---- B部分: 手工发送字节数组 ----
    reg [7:0] inj_mem [0:63];
    task send_bytes;
        input integer n;
        integer k;
        begin
            mac_rx_data_ovr_en = 1'b1;
            for (k = 0; k < n; k = k + 1) begin
                @(posedge clk); #1;
                mrx_data  = inj_mem[k];
                mrx_valid = 1'b1;
                mrx_last  = (k == n-1);
                mrx_err   = 1'b0;
            end
            @(posedge clk); #1;
            mrx_valid = 1'b0;
            mrx_last  = 1'b0;
            repeat (3) @(posedge clk);
        end
    endtask

    // ---- 构造一个"baseline valid frame"到inj_mem, 供B部分各测试改坏用 ----
    function [15:0] calc_ip_checksum;
        input [15:0] w0,w1,w2,w3,w4,w6,w7,w8,w9;
        reg [19:0] s; reg [16:0] f1; reg [15:0] f2;
        begin
            s  = w0+w1+w2+w3+w4+16'h0000+w6+w7+w8+w9;
            f1 = {1'b0,s[15:0]} + {13'd0,s[19:16]};
            f2 = f1[15:0] + {15'd0,f1[16]};
            calc_ip_checksum = ~f2;
        end
    endfunction

    task build_baseline_frame;
        input [11:0] vlid;
        input [10:0] plen;
        input [7:0]  seq;
        integer i;
        reg [15:0] iptot, udplen, ipchk;
        begin
            iptot  = 16'd28 + plen;
            udplen = 16'd8 + plen;
            ipchk  = calc_ip_checksum(16'h4500, iptot, 16'h0000, 16'h0000, 16'h4011,
                                       32'hC0A80101 >> 16, 32'hC0A80101 & 32'h0000FFFF,
                                       32'hC0A801FF >> 16, 32'hC0A801FF & 32'h0000FFFF);
            inj_mem[0]=OUI[31:24]; inj_mem[1]=OUI[23:16]; inj_mem[2]=OUI[15:8]; inj_mem[3]=OUI[7:0];
            inj_mem[4]={4'd0, vlid[11:8]}; inj_mem[5]=vlid[7:0];
            inj_mem[6]=8'h02; inj_mem[7]=8'h00; inj_mem[8]=8'h00; inj_mem[9]=8'h00; inj_mem[10]=8'h00; inj_mem[11]=8'h01;
            inj_mem[12]=8'h08; inj_mem[13]=8'h00;
            inj_mem[14]=8'h45; inj_mem[15]=8'h00;
            inj_mem[16]=iptot[15:8]; inj_mem[17]=iptot[7:0];
            inj_mem[18]=8'h00; inj_mem[19]=8'h00; inj_mem[20]=8'h00; inj_mem[21]=8'h00;
            inj_mem[22]=8'h40; inj_mem[23]=8'h11;
            inj_mem[24]=ipchk[15:8]; inj_mem[25]=ipchk[7:0];
            inj_mem[26]=8'hC0; inj_mem[27]=8'hA8; inj_mem[28]=8'h01; inj_mem[29]=8'h01;
            inj_mem[30]=8'hC0; inj_mem[31]=8'hA8; inj_mem[32]=8'h01; inj_mem[33]=8'hFF;
            inj_mem[34]=16'd5000>>8; inj_mem[35]=16'd5000&8'hFF;
            inj_mem[36]=16'd5001>>8; inj_mem[37]=16'd5001&8'hFF;
            inj_mem[38]=udplen[15:8]; inj_mem[39]=udplen[7:0];
            inj_mem[40]=8'h00; inj_mem[41]=8'h00;
            for (i = 0; i < plen; i = i + 1)
                inj_mem[42+i] = 8'hB0 + i;
            inj_mem[42+plen] = seq;
        end
    endtask

    integer i;
    initial begin
        error_cnt = 0;
        rst_n = 0; reg_addr=0; reg_wdata=0; reg_we=0; reg_re=0; vl_req_v=0; vl_req_id=7;
        mac_rx_data_ovr_en = 0; mrx_data=0; mrx_valid=0; mrx_last=0; mrx_err=0;
        rx_lookup_ovr_en = 0; rx_lookup_ovr_latency = 0;
        ovr_entry_valid = 0; ovr_min_len = 0; ovr_max_len = 0;
        payload_active = 0; payload_total = 0;
        for (i=0;i<10;i=i+1) payload_mem[i] = 8'hA0+i;

        repeat(4) @(posedge clk); #1; rst_n = 1; wait (vlt_init_done); repeat(2) @(posedge clk);

        // ==================== A: 真实链路环回测试 ====================
        write_tx_entry(7'd7, 3'd0, 1'b1, 11'd1518, {OUI, 16'd7});
        write_rx_entry(12'd7, 11'd64, 11'd1518, 1'b1);

        cap_out_ptr = 0;
        payload_ptr = 0; payload_total = 10; payload_active = 1;
        @(posedge clk); #1; vl_req_v = 1'b1;

        // 等frame_done
        while (!frame_done) @(posedge clk);
        #1;
        $display("---- A: real link loop-back ----");
        if (frame_error === 1'b0) $display("[PASS] frame_error=0");
        else begin $display("[FAIL] frame_error=%b (expected 0)", frame_error); error_cnt=error_cnt+1; end

        if (frame_vlid === 12'd7) $display("[PASS] frame_vlid=7");
        else begin $display("[FAIL] frame_vlid=%0d (expected 7)", frame_vlid); error_cnt=error_cnt+1; end

        if (frame_seq_num === 8'd0) $display("[PASS] frame_seq_num=0 (reset marker, first frame)");
        else begin $display("[FAIL] frame_seq_num=%0d (expected 0)", frame_seq_num); error_cnt=error_cnt+1; end

        if (frame_payload_len === 11'd10) $display("[PASS] frame_payload_len=10 (real data only; the 7 padding bytes are not part of the payload)");
        else begin $display("[FAIL] frame_payload_len=%0d (expected 10)", frame_payload_len); error_cnt=error_cnt+1; end

        begin: chkpay
            integer ok; ok=1;
            for (i=0;i<10;i=i+1) if (cap_out[i] !== (8'hA0+i)) ok=0;
            if (ok) $display("[PASS] 10-byte payload correct");
            else begin $display("[FAIL] payload content wrong"); error_cnt=error_cnt+1; end
        end

        vl_req_v = 1'b0;
        repeat(5) @(posedge clk);

        // ==================== B: 人工构造错误帧 ====================
        $display("---- B: error path tests ----");

        // B1: 以太网类型错
        build_baseline_frame(12'd7, 11'd17, 8'd1);
        inj_mem[12] = 8'h86; // 改坏ethertype
        send_bytes(60);
        #1;
        if (frame_error === 1'b1) $display("[PASS] B1 wrong EtherType detected");
        else begin $display("[FAIL] B1 wrong EtherType not detected"); error_cnt=error_cnt+1; end

        // B2: IP校验和错
        build_baseline_frame(12'd7, 11'd17, 8'd1);
        inj_mem[25] = inj_mem[25] ^ 8'hFF; // 改坏checksum低字节
        send_bytes(60);
        #1;
        if (frame_error === 1'b1) $display("[PASS] B2 bad IP checksum detected");
        else begin $display("[FAIL] B2 bad IP checksum not detected"); error_cnt=error_cnt+1; end

        // B3: VL未配置 (用VLID=99, 没在rx表里配过)
        build_baseline_frame(12'd99, 11'd17, 8'd1);
        send_bytes(60);
        #1;
        if (frame_error === 1'b1) $display("[PASS] B3 unconfigured VL detected");
        else begin $display("[FAIL] B3 unconfigured VL not detected"); error_cnt=error_cnt+1; end

        // B4: VL的LMAX收窄到63字节；合法最小帧为64字节，因此应被拒绝
        write_rx_entry(12'd7, 11'd64, 11'd63, 1'b1);
        build_baseline_frame(12'd7, 11'd17, 8'd1);
        send_bytes(60);
        #1;
        if (frame_error === 1'b1) $display("[PASS] B4 LMAX violation detected");
        else begin $display("[FAIL] B4 LMAX violation not detected"); error_cnt=error_cnt+1; end
        write_rx_entry(12'd7, 11'd64, 11'd1518, 1'b1); // 改回来

        // B5: 帧截断 (头还没收完就mac_rx_last)
        build_baseline_frame(12'd7, 11'd17, 8'd1);
        mac_rx_data_ovr_en = 1'b1;
        for (i = 0; i < 20; i = i + 1) begin
            @(posedge clk); #1;
            mrx_data  = inj_mem[i];
            mrx_valid = 1'b1;
            mrx_last  = (i == 19); // 提前在第20字节结束
        end
        @(posedge clk); #1;  // DUT在这一拍处理i=19那个mac_rx_last, frame_done应在此刻脉冲
        if (frame_done && frame_error)
            $display("[PASS] B5 truncated frame detected");
        else begin
            $display("[FAIL] B5 truncated frame not detected: frame_done=%b frame_error=%b", frame_done, frame_error);
            error_cnt = error_cnt + 1;
        end
        mrx_valid = 1'b0; mrx_last = 1'b0;
        repeat(3) @(posedge clk);

        // ==================== C: 健壮性专项测试 ====================
        $display("---- C: robustness tests ----");
        rx_lookup_ovr_en = 1'b0;
        mac_rx_data_ovr_en = 1'b1;
        ovr_entry_valid = 1'b1; ovr_min_len = 11'd64; ovr_max_len = 11'd1518;

        // C1: 查表延迟压力测试——最小合法17字节编码载荷+人为拉长查表延迟,
        //     验证不会用上一次查表残留的陈旧数据来判定当前帧
        //     (原设计在byte_cnt==41才发起查表, 短帧下几乎没有余量;
        //      现改为接收VLID低字节的byte_cnt==5就发起, 这里用延迟=8拍验证鲁棒性)
        rx_lookup_ovr_en = 1'b1;
        rx_lookup_ovr_latency = 8;
        build_baseline_frame(12'd7, 11'd17, 8'd1);
        send_bytes(42+17+1);
        #1;
        if (frame_error === 1'b0 && frame_vlid === 12'd7 && frame_payload_len === 11'd17)
            $display("[PASS] C1 short payload + large lookup latency (8 cycles): correct decision (no stale data used)");
        else begin
            $display("[FAIL] C1 wrong decision with short payload + large lookup latency: frame_error=%b frame_vlid=%0d frame_payload_len=%0d",
                      frame_error, frame_vlid, frame_payload_len);
            error_cnt = error_cnt + 1;
        end
        repeat(5) @(posedge clk);

        // 再验证一次: 把VL配置为"invalid"(entry_valid=0), 延迟依旧=8拍,
        // 确认这次确实能正确报错(证明真的在用"this time"的查表结果, 不是
        // 侥幸沿用了上一次C1里"valid"的陈旧结果)
        ovr_entry_valid = 1'b0;
        build_baseline_frame(12'd7, 11'd17, 8'd1);
        send_bytes(42+17+1);
        #1;
        if (frame_error === 1'b1)
            $display("[PASS] C1b VL then made invalid, same short payload + large latency: error reported (proves the current lookup result is used)");
        else begin
            $display("[FAIL] C1b invalid VL not detected: stale lookup result in use");
            error_cnt = error_cnt + 1;
        end
        ovr_entry_valid = 1'b1;
        repeat(5) @(posedge clk);
        rx_lookup_ovr_en = 1'b0;

        // C2: 序列号后仍有多余字节(畸形帧), 验证不会把多余字节误当成
        //     下一帧的开头, 并且这些多余字节被正确丢弃、frame_error=1
        build_baseline_frame(12'd7, 11'd17, 8'd1);
        inj_mem[42+17+1]   = 8'hEE;  // 序列号(下标59)之后追加3个垃圾字节
        inj_mem[42+17+1+1] = 8'hEE;
        inj_mem[42+17+1+2] = 8'hEE;
        send_bytes(42+17+1+3);
        #1;
        if (frame_error === 1'b1 && frame_vlid === 12'd7)
            $display("[PASS] C2 extra bytes after the SN are dropped and the frame is flagged as error");
        else begin
            $display("[FAIL] C2 extra bytes handled wrongly: frame_error=%b frame_vlid=%0d", frame_error, frame_vlid);
            error_cnt = error_cnt + 1;
        end
        repeat(3) @(posedge clk);

        // 紧接着发一个完全正常的帧, 验证C2的畸形帧没有破坏后续帧同步
        build_baseline_frame(12'd7, 11'd17, 8'd2);
        send_bytes(60);
        #1;
        if (frame_error === 1'b0 && frame_seq_num === 8'd2)
            $display("[PASS] C2 follow-up: next normal frame unaffected by the malformed one");
        else begin
            $display("[FAIL] C2 follow-up: normal frame affected: frame_error=%b frame_seq_num=%0d", frame_error, frame_seq_num);
            error_cnt = error_cnt + 1;
        end

        mac_rx_data_ovr_en = 1'b0;
        repeat(3) @(posedge clk);

        if (error_cnt == 0) $display("==== frame_decap_filter_tb: ALL PASS ====");
        else $display("==== frame_decap_filter_tb: %0d ERROR(S) ====", error_cnt);
        $finish;
    end

endmodule
