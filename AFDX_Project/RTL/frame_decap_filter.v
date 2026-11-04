//=====================================================================
// 模块名 : frame_decap_filter
// 功能   : AFDX接收帧解析、VL合法性过滤与帧摘要输出
//          Ethernet(14B)+IPv4(20B)+UDP(8B)+AFDX payload+SN
//
// 规范口径：
//   - 目的MAC的低12位作为本项目RX VLID，支持4096条RX VL；
//   - 目的MAC高32位由EXPECTED_OUI约束；
//   - AFDX payload不足最小17B时，发送端已在UDP/AFDX payload区补0，
//     本模块按IP总长度字段解析该“封装后的payload长度”；
//   - RX VL表min_len/max_len统一表示“以太网线速帧长(含FCS)”64..1518B；
//     本模块的MAC接口已剥离FCS，所以比较时使用MAC数据长度+4。
//
// 健壮性：
//   1) VLID低字节刚收完就发起查表，且直接使用当前mac_rx_data，避免
//      非阻塞赋值造成的旧hdr_buf数据竞争；
//   2) 查表结果未到时进入S_WAIT_VL，绝不使用上一帧表项；
//   3) 序列号后出现额外字节进入S_DRAIN，直到真正帧尾才重新对齐；
//   4) 最终错误判定会包含当前结束字节的mac_rx_err。
//=====================================================================
module frame_decap_filter #(
    parameter RX_IDX_W = 12,
    parameter [31:0] EXPECTED_OUI = 32'h0300_0000 // 低2位必须是11(组播+本地管理位)
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- MAC接收流：FCS由MAC核处理并剥离 ----
    input  wire [7:0]  mac_rx_data,
    input  wire        mac_rx_valid,
    input  wire        mac_rx_last,
    input  wire        mac_rx_err,

    // ---- RX VL查表 ----
    output reg                  rx_lookup_valid,
    output reg  [RX_IDX_W-1:0] rx_lookup_vlid,
    input  wire                 rx_lookup_rdy,
    input  wire [10:0]          rx_min_len,
    input  wire [10:0]          rx_max_len,
    input  wire                  rx_entry_valid,

    // ---- 载荷流 ----
    output reg                  out_payload_valid,
    output reg  [7:0]           out_payload_data,
    output reg                  out_payload_last,

    // ---- 帧摘要 ----
    output reg                  frame_done,
    output reg  [RX_IDX_W-1:0]  frame_vlid,
    output reg  [7:0]           frame_seq_num,
    output reg  [10:0]          frame_payload_len,
    output reg                  frame_error,

    // ---- IP分片摘要(本轮新增，供ip_reasm.v使用) ----
    output reg  [15:0]          frame_ip_id,      // IP.Identification
    output reg                  frame_mf,         // IP.Flags.MF (1=后面还有分片)
    output reg  [12:0]          frame_offset_8b,  // IP.FragOffset，单位8字节，0代表首片/未分片

    // ---- UDP端口(取自本帧UDP头；仅首片/未分片帧有意义) ----
    // 来自报头缓冲，在frame_done拍及其后到下一帧UDP头到达之前保持稳定，
    // 下游应在frame_done拍采样。
    output wire [15:0]          frame_udp_src_port,
    output wire [15:0]          frame_udp_dst_port
);

    localparam S_IDLE    = 3'd0;
    localparam S_HDR     = 3'd1;
    localparam S_PAYLOAD = 3'd2;
    localparam S_SEQ     = 3'd3;
    localparam S_WAIT_VL = 3'd4;
    localparam S_DRAIN   = 3'd5;

    // 收到byte0后，byte_cnt=1；因此byte_cnt=5表示当前正在接收hdr[5]，
    // 即VLID低8位。必须使用当前mac_rx_data构造查表索引。
    localparam [5:0] VLID_LOW_BYTE_CNT = 6'd5;

    localparam [10:0] MIN_PAYLOAD_LEN = 11'd17;
    localparam [10:0] MAX_PAYLOAD_LEN = 11'd1471;
    localparam [10:0] CONT_MAX_PAYLOAD_LEN = 11'd1479;
    localparam [11:0] MIN_WIRE_FRAME  = 12'd64;
    localparam [11:0] MAX_WIRE_FRAME  = 12'd1518;

    reg [2:0]   state;
    reg [5:0]   byte_cnt;
    reg [15:0]  payload_remaining;
    reg         phy_err_latched;
    reg         frame_truncated;
    reg         excess_bytes;
    reg [10:0]  pad_cnt;          // S_SEQ中已收到的补零字节数
    reg         pad_bad;          // 补零字节数与期望不符

    reg [7:0]   seq_captured;
    reg [RX_IDX_W-1:0] vlid_captured;
    reg [10:0]  plen_captured;
    reg [15:0]  ipid_captured;
    reg         mf_captured;
    reg [12:0]  fragoff_captured;

    reg [7:0] hdr_buf [0:41];

    // ---- 当前帧VL查表结果 ----
    reg         vl_valid_latched;
    reg [10:0]  vl_min_len_latched;
    reg [10:0]  vl_max_len_latched;
    reg         vl_result_ready;
    reg         vl_lookup_pending;

    wire [11:0] derived_vlid12 = {hdr_buf[4][3:0], hdr_buf[5]};
    wire [RX_IDX_W-1:0] derived_vlid = derived_vlid12[RX_IDX_W-1:0];

    wire        oui_ok       = ({hdr_buf[0],hdr_buf[1],hdr_buf[2],hdr_buf[3]} == EXPECTED_OUI);
    wire        ethertype_ok = (hdr_buf[12] == 8'h08) && (hdr_buf[13] == 8'h00);
    wire        protocol_ok  = (hdr_buf[23] == 8'h11);

    // ---- IP分片字段(IP头第4/6/7字节，byte_cnt=21时已全部收完) ----
    wire [15:0] frag_id_w     = {hdr_buf[18], hdr_buf[19]};
    wire        frag_mf_w     = hdr_buf[20][5];
    wire [12:0] frag_offset_w = {hdr_buf[20][4:0], hdr_buf[21]};
    wire        has_udp       = (frag_offset_w == 13'd0);
    wire [5:0]  header_end_cnt = has_udp ? 6'd41 : 6'd33;

    wire [15:0] ip_total_len_w = {hdr_buf[16],hdr_buf[17]};
    wire [15:0] min_ip_total_w = has_udp ? 16'd28 : 16'd20;
    wire [15:0] payload_len_full_w = (ip_total_len_w >= min_ip_total_w) ?
                                      (ip_total_len_w - min_ip_total_w) : 16'd0;
    wire [10:0] payload_len_w       = payload_len_full_w[10:0];
    // 填充规则(与发送端一致): 带UDP头的帧UDP载荷补到17字节，延续分片IP载荷补到25字节；
    // 补零位于载荷之后、SN之前，不计入IP/UDP长度。
    wire [10:0] pad_min  = has_udp ? MIN_PAYLOAD_LEN : 11'd25;
    wire [10:0] pad_exp  = (payload_len_w < pad_min) ? (pad_min - payload_len_w) : 11'd0;
    wire [10:0] eff_max_payload     = has_udp ? MAX_PAYLOAD_LEN      : CONT_MAX_PAYLOAD_LEN;
    wire        payload_len_overflow= (payload_len_full_w > {5'd0,eff_max_payload});
    wire        len_underflow       = (ip_total_len_w < min_ip_total_w);

    assign frame_udp_src_port = {hdr_buf[34], hdr_buf[35]};
    assign frame_udp_dst_port = {hdr_buf[36], hdr_buf[37]};

    wire [15:0] udp_len_w = {hdr_buf[38],hdr_buf[39]};
    wire        version_ihl_ok = (hdr_buf[14] == 8'h45);
    wire        udp_len_ok_raw = (!len_underflow) &&
                             (udp_len_w == (ip_total_len_w - 16'd20));
    wire        udp_len_ok = has_udp ? udp_len_ok_raw : 1'b1;

    // ---- IPv4头校验和 ----
    wire [15:0] cw0 = {hdr_buf[14],hdr_buf[15]};
    wire [15:0] cw1 = {hdr_buf[16],hdr_buf[17]};
    wire [15:0] cw2 = {hdr_buf[18],hdr_buf[19]};
    wire [15:0] cw3 = {hdr_buf[20],hdr_buf[21]};
    wire [15:0] cw4 = {hdr_buf[22],hdr_buf[23]};
    wire [15:0] cw5 = {hdr_buf[24],hdr_buf[25]};
    wire [15:0] cw6 = {hdr_buf[26],hdr_buf[27]};
    wire [15:0] cw7 = {hdr_buf[28],hdr_buf[29]};
    wire [15:0] cw8 = {hdr_buf[30],hdr_buf[31]};
    wire [15:0] cw9 = {hdr_buf[32],hdr_buf[33]};

    wire [19:0] csum_raw   = cw0+cw1+cw2+cw3+cw4+cw5+cw6+cw7+cw8+cw9;
    wire [16:0] csum_fold1 = {1'b0,csum_raw[15:0]} + {13'd0,csum_raw[19:16]};
    wire [15:0] csum_fold2 = csum_fold1[15:0] + {15'd0,csum_fold1[16]};
    wire        checksum_ok = (csum_fold2 == 16'hFFFF);

    // MAC接口观察到的长度不包含FCS，因此恢复线速帧长时+4。
    // 期望的补零字节数已计入(pad_exp)，实际收到的补零数在S_SEQ中逐字节核对。
    wire [11:0] header_overhead = has_udp ? 12'd43 : 12'd35;
    wire [11:0] mac_frame_len  = header_overhead + {1'b0,payload_len_w} + {1'b0,pad_exp};
    wire [11:0] wire_frame_len = mac_frame_len + 12'd4;

    // 长度规则：AFDX payload最终应为17..1471B，线速帧应为64..1518B，
    // 同时必须落在当前VL配置的LMIN/LMAX内。
    wire length_error =
        (payload_len_full_w > {5'd0,eff_max_payload}) ||
        (wire_frame_len < MIN_WIRE_FRAME) ||
        (wire_frame_len > MAX_WIRE_FRAME) ||
        (wire_frame_len < {1'b0,vl_min_len_latched}) ||
        (wire_frame_len > {1'b0,vl_max_len_latched});

    wire frame_error_calc = phy_err_latched || frame_truncated || excess_bytes || pad_bad ||
                            !oui_ok || !ethertype_ok || !version_ihl_ok ||
                            !protocol_ok || !udp_len_ok || !checksum_ok ||
                            len_underflow || payload_len_overflow ||
                            !vl_valid_latched ||
                            length_error;

    // ==================== 主状态机 ====================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                <= S_IDLE;
            byte_cnt             <= 6'd0;
            payload_remaining    <= 16'd0;
            phy_err_latched      <= 1'b0;
            frame_truncated      <= 1'b0;
            excess_bytes         <= 1'b0;
            pad_cnt              <= 11'd0;
            pad_bad              <= 1'b0;
            seq_captured         <= 8'd0;
            vlid_captured        <= {RX_IDX_W{1'b0}};
            plen_captured        <= 11'd0;
            ipid_captured        <= 16'd0;
            mf_captured          <= 1'b0;
            fragoff_captured     <= 13'd0;

            rx_lookup_valid      <= 1'b0;
            rx_lookup_vlid       <= {RX_IDX_W{1'b0}};
            vl_valid_latched     <= 1'b0;
            vl_min_len_latched   <= 11'd0;
            vl_max_len_latched   <= 11'd0;
            vl_result_ready      <= 1'b0;
            vl_lookup_pending    <= 1'b0;

            frame_done            <= 1'b0;
            frame_vlid            <= {RX_IDX_W{1'b0}};
            frame_seq_num         <= 8'd0;
            frame_payload_len     <= 11'd0;
            frame_error           <= 1'b0;
            frame_ip_id           <= 16'd0;
            frame_mf              <= 1'b0;
            frame_offset_8b       <= 13'd0;
            out_payload_valid     <= 1'b0;
            out_payload_data      <= 8'd0;
            out_payload_last      <= 1'b0;
        end else begin
            // 一拍脉冲输出默认清零。
            frame_done        <= 1'b0;
            out_payload_valid <= 1'b0;
            out_payload_last  <= 1'b0;
            rx_lookup_valid   <= 1'b0;

            // 查表接口约定为固定延迟返回；只有当前帧确实存在未完成请求时
            // 才能接受返回，防止上一帧的迟到结果污染下一帧。
            if (vl_lookup_pending && rx_lookup_rdy) begin
                vl_valid_latched   <= rx_entry_valid;
                vl_min_len_latched <= rx_min_len;
                vl_max_len_latched <= rx_max_len;
                vl_result_ready    <= 1'b1;
                vl_lookup_pending   <= 1'b0;
            end

            case (state)
                S_IDLE: begin
                    byte_cnt          <= 6'd0;
                    phy_err_latched   <= 1'b0;
                    frame_truncated   <= 1'b0;
                    excess_bytes      <= 1'b0;
                    pad_cnt           <= 11'd0;
                    pad_bad           <= 1'b0;
                    vl_result_ready   <= 1'b0;
                    vl_lookup_pending <= 1'b0;

                    if (mac_rx_valid) begin
                        hdr_buf[0] <= mac_rx_data;
                        byte_cnt   <= 6'd1;
                        if (mac_rx_err) phy_err_latched <= 1'b1;

                        if (mac_rx_last) begin
                            frame_truncated  <= 1'b1;
                            frame_done       <= 1'b1;
                            frame_vlid       <= {RX_IDX_W{1'b0}};
                            frame_seq_num    <= 8'd0;
                            frame_payload_len<= 11'd0;
                            frame_error      <= 1'b1;
                            frame_ip_id      <= 16'd0;
                            frame_mf         <= 1'b0;
                            frame_offset_8b  <= 13'd0;
                            state            <= S_IDLE;
                        end else begin
                            state <= S_HDR;
                        end
                    end
                end

                S_HDR: begin
                    if (mac_rx_valid) begin
                        if (mac_rx_err) phy_err_latched <= 1'b1;
                        hdr_buf[byte_cnt] <= mac_rx_data;

                        // 当前byte_cnt=5时正在接收hdr[5]，必须直接使用本拍数据。
                        if (byte_cnt == VLID_LOW_BYTE_CNT) begin
                            rx_lookup_valid <= 1'b1;
                            rx_lookup_vlid  <= {hdr_buf[4][3:0], mac_rx_data};
                            vl_lookup_pending <= 1'b1;
                            vl_result_ready   <= 1'b0;
                        end

                        if (mac_rx_last) begin
                            frame_truncated <= 1'b1;
                            if (byte_cnt >= 6'd6)
                                vlid_captured <= derived_vlid;
                            else if (byte_cnt == VLID_LOW_BYTE_CNT)
                                vlid_captured <= {hdr_buf[4][3:0], mac_rx_data};
                            else
                                vlid_captured <= {RX_IDX_W{1'b0}};
                            plen_captured <= 11'd0;
                            ipid_captured <= 16'd0;
                            mf_captured   <= 1'b0;
                            fragoff_captured <= 13'd0;

                            if (vl_lookup_pending ||
                                (byte_cnt == VLID_LOW_BYTE_CNT)) begin
                                // 查询结果仍可能在路上，必须等它回来后再结束当前帧。
                                state <= S_WAIT_VL;
                            end else begin
                                frame_done        <= 1'b1;
                                frame_vlid        <= vlid_captured;
                                frame_seq_num     <= 8'd0;
                                frame_payload_len <= 11'd0;
                                frame_error       <= 1'b1;
                                frame_ip_id       <= 16'd0;
                                frame_mf          <= 1'b0;
                                frame_offset_8b   <= 13'd0;
                                state             <= S_IDLE;
                            end
                        end else if (byte_cnt == header_end_cnt) begin
                            payload_remaining <= payload_len_full_w;
                            if (payload_len_w == 11'd0)
                                state <= S_SEQ;
                            else
                                state <= S_PAYLOAD;
                        end else begin
                            byte_cnt <= byte_cnt + 6'd1;
                        end
                    end
                end

                S_PAYLOAD: begin
                    if (mac_rx_valid) begin
                        if (mac_rx_err) phy_err_latched <= 1'b1;
                        out_payload_valid <= 1'b1;
                        out_payload_data  <= mac_rx_data;

                        if (mac_rx_last) begin
                            out_payload_last   <= 1'b1;
                            frame_truncated     <= 1'b1;
                            frame_vlid          <= derived_vlid;
                            frame_payload_len   <= payload_len_w;
                            frame_seq_num       <= 8'd0;
                            frame_ip_id         <= frag_id_w;
                            frame_mf            <= frag_mf_w;
                            frame_offset_8b     <= frag_offset_w;
                            vlid_captured       <= derived_vlid;
                            plen_captured       <= payload_len_w;
                            ipid_captured       <= frag_id_w;
                            mf_captured         <= frag_mf_w;
                            fragoff_captured    <= frag_offset_w;

                            if (vl_lookup_pending || !vl_result_ready) begin
                                state <= S_WAIT_VL;
                            end else begin
                                frame_done  <= 1'b1;
                                frame_error <= frame_error_calc || mac_rx_err;
                                state       <= S_IDLE;
                            end
                        end else if (payload_remaining == 16'd1) begin
                            payload_remaining <= 16'd0;
                            state <= S_SEQ;
                        end else begin
                            payload_remaining <= payload_remaining - 16'd1;
                        end
                    end
                end

                S_SEQ: begin
                    if (mac_rx_valid) begin
                        if (mac_rx_err) phy_err_latched <= 1'b1;

                        if (mac_rx_last) begin
                            frame_vlid        <= derived_vlid;
                            frame_seq_num     <= mac_rx_data;
                            frame_payload_len <= payload_len_w;
                            frame_ip_id       <= frag_id_w;
                            frame_mf          <= frag_mf_w;
                            frame_offset_8b   <= frag_offset_w;
                            vlid_captured     <= derived_vlid;
                            seq_captured      <= mac_rx_data;
                            plen_captured     <= payload_len_w;
                            ipid_captured     <= frag_id_w;
                            mf_captured       <= frag_mf_w;
                            fragoff_captured  <= frag_offset_w;

                            pad_bad <= (pad_cnt != pad_exp);
                            if (vl_result_ready) begin
                                frame_done  <= 1'b1;
                                frame_error <= frame_error_calc || mac_rx_err || (pad_cnt != pad_exp);
                                state       <= S_IDLE;
                            end else begin
                                state <= S_WAIT_VL;
                            end
                        end else if (pad_cnt < pad_exp) begin
                            pad_cnt <= pad_cnt + 11'd1;       // 补零字节：丢弃，继续等SN
                        end else begin
                            excess_bytes <= 1'b1;
                            vlid_captured <= derived_vlid;
                            seq_captured  <= mac_rx_data;
                            plen_captured <= payload_len_w;
                            ipid_captured <= frag_id_w;
                            mf_captured   <= frag_mf_w;
                            fragoff_captured <= frag_offset_w;
                            state         <= S_DRAIN;
                        end
                    end
                end

                S_WAIT_VL: begin
                    // 若是在HDR提前截断，seq尚未得到；其它路径已经缓存了摘要。
                    if (vl_result_ready) begin
                        frame_done        <= 1'b1;
                        frame_vlid        <= vlid_captured;
                        frame_seq_num     <= seq_captured;
                        frame_payload_len <= plen_captured;
                        frame_error       <= frame_error_calc;
                        frame_ip_id       <= ipid_captured;
                        frame_mf          <= mf_captured;
                        frame_offset_8b   <= fragoff_captured;
                        state             <= S_IDLE;
                    end
                end

                S_DRAIN: begin
                    if (mac_rx_valid) begin
                        if (mac_rx_err) phy_err_latched <= 1'b1;

                        if (mac_rx_last) begin
                            if (vl_result_ready) begin
                                frame_done        <= 1'b1;
                                frame_vlid        <= vlid_captured;
                                frame_seq_num     <= seq_captured;
                                frame_payload_len <= plen_captured;
                                frame_error       <= frame_error_calc || mac_rx_err;
                                frame_ip_id       <= ipid_captured;
                                frame_mf          <= mf_captured;
                                frame_offset_8b   <= fragoff_captured;
                                state             <= S_IDLE;
                            end else begin
                                state <= S_WAIT_VL;
                            end
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
