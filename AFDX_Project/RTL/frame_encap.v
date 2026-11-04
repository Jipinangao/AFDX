//=====================================================================
// 模块名 : frame_encap
// 功能   : AFDX发送封装
//          Ethernet(14B) + IPv4(20B) + [UDP(8B)] + AFDX payload + SN
//          FCS由MAC核追加，本模块输出的mac_tx_*不包含FCS。
//
// 规范口径：
//   - AFDX payload原始有效长度：1..1471B(首帧/不分片场景，含UDP头开销)
//   - payload不足17B时补0到17B
//   - 以太网线速帧长：64..1518B（含4B FCS）
//   - 因此本模块MAC侧输出的数据长度为：60..1514B
//   - grant_max_len按“含FCS的线速帧长”解释，与vl_table统一
//   - 序列号0保留用于复位语义，正常发送序号1..255循环
//
// IP分片支持：
//   本模块原本总是构造一个完整的、独立的(未分片的)IPv4+UDP头，
//   IP.Identification/Flags/FragOffset字段硬编码为0。为了支持
//   ip_frag.v把超过1471字节的应用消息拆成多个IP分片分别发送，
//   新增4个输入端口：
//     frag_ip_id        : 本分片所属原始IP数据报的Identification，
//                          同一条消息的所有分片必须相同(供接收端
//                          重组时判断"这些分片属于同一条消息")。
//     frag_mf           : More Fragments标志，除最后一片外均为1。
//     frag_offset_8b    : FragOffset字段(以8字节为单位)，标识本分片
//                          数据在"原始IP载荷(UDP头+用户数据)"里的
//                          起始位置。
//     frag_skip_udp_hdr : 1=本分片是延续分片(非首片)，不携带UDP头，
//                          payload_data视为对原始IP载荷的纯字节延续
//                          (不含UDP头)；0=本分片自带完整UDP头(不分片
//                          场景，或分片场景下的首片)——现有全部
//                          testbench都把这4个端口分别接0，行为与
//                          修改前完全一致，不影响任何已通过的测试。
//   frag_skip_udp_hdr=1时头部只有Ethernet(14B)+IP(20B)共34字节，
//   省下的8字节UDP头空间转化为更大的可用载荷容量(最多1479字节，
//   而非首片场景的1471字节)。
//=====================================================================
module frame_encap #(
    parameter TX_IDX_W     = 7,
    parameter NUM_TX_VL    = 128,
    parameter [47:0] SRC_MAC      = 48'h02_00_00_00_01_20, // 02:00:00(常量)+0001(用户标识)+001_00000(接口A+常量0), 引用XAPP1130 Figure5
    parameter [31:0] SRC_IP       = 32'hC0A80101,
    parameter [31:0] DST_IP       = 32'hC0A801FF
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // ---- bag_scheduler授权 ----
    input  wire                 grant_valid,
    input  wire [TX_IDX_W-1:0]  grant_vlid,
    input  wire [47:0]          grant_dest_mac,
    input  wire [10:0]          grant_max_len,
    output wire                 grant_ready,

    // ---- 上游载荷 ----
    input  wire                 payload_valid,
    input  wire [7:0]           payload_data,
    input  wire                 payload_last,
    input  wire [10:0]          payload_len,
    output wire                 payload_ready,

    // ---- IP分片相关字段(见上方模块头说明；不分片时全部接0/1'b0) ----
    input  wire [15:0]          frag_ip_id,
    input  wire                 frag_mf,
    input  wire [12:0]          frag_offset_8b,
    input  wire                 frag_skip_udp_hdr,

    // ---- UDP端口(随每个分片提交；延续分片无UDP头，该值不被使用) ----
    input  wire [15:0]          udp_src_port,
    input  wire [15:0]          udp_dst_port,

    // ---- MAC发送流 ----
    output wire [7:0]           mac_tx_data,
    output wire                 mac_tx_valid,
    output wire                 mac_tx_last,
    input  wire                 mac_tx_ready,

    // ---- LMAX/协议长度违规脉冲 ----
    output wire                 frame_len_violation
);

    localparam [2:0] S_IDLE         = 3'd0;
    localparam [2:0] S_LATCH_LEN    = 3'd1;
    localparam [2:0] S_SEND_HDR     = 3'd2;
    localparam [2:0] S_SEND_PAYLOAD = 3'd3;
    localparam [2:0] S_SEND_PAD     = 3'd4;
    localparam [2:0] S_SEND_SEQ     = 3'd5;

    localparam [10:0] MIN_PAYLOAD_LEN      = 11'd17;
    localparam [10:0] MAX_PAYLOAD_LEN      = 11'd1471; // 首片/不分片场景(自带UDP头)
    localparam [10:0] MAX_PAYLOAD_LEN_CONT = 11'd1479; // 延续分片场景(无UDP头，多8字节容量)
    localparam [11:0] MIN_WIRE_FRAME   = 12'd64;
    localparam [11:0] MAX_WIRE_FRAME   = 12'd1518;

    reg [2:0]          state;
    reg [TX_IDX_W-1:0] cand_vlid;
    reg [47:0]         cand_dest_mac;
    reg [10:0]         cand_max_len;
    reg [10:0]         payload_len_reg;
    reg [10:0]         payload_remaining;
    reg [10:0]         pad_total;
    reg [10:0]         pad_remaining;
    reg [7:0]          first_byte;
    reg                first_byte_last;
    reg                first_byte_pending;
    reg [5:0]          byte_idx;
    reg                cand_skip_udp_hdr;
    reg [15:0]         cand_frag_ip_id;
    reg                cand_frag_mf;
    reg [12:0]         cand_frag_offset_8b;

    reg [7:0] hdr_buf [0:41];
    reg [7:0] seq_num [0:NUM_TX_VL-1];
    integer k;


    // AFDX/ARINC 664: 载荷不足时在"载荷之后、序号SN之前"补0，补零字节不计入
    // IP/UDP长度字段(XAPP1130 Figure 3: AFDX Payload | Padding 0-16 | SN)。
    //   带UDP头的帧   : UDP载荷补到17字节   -> 线速帧>=64字节
    //   延续分片(无UDP头): IP载荷补到25字节 -> 线速帧>=64字节
    wire [10:0] pad_min   = frag_skip_udp_hdr ? 11'd25 : MIN_PAYLOAD_LEN;
    wire [10:0] pad_len   = (payload_len < pad_min) ? (pad_min - payload_len) : 11'd0;
    wire [15:0] ip_total_len = frag_skip_udp_hdr ?
                               (16'd20 + {5'd0, payload_len}) :
                               (16'd28 + {5'd0, payload_len});
    wire [15:0] udp_len      = 16'd8  + {5'd0, payload_len};

    // IPv4 header checksum。Identification/Flags+FragOffset字段
    // 改为使用分片相关输入，不再硬编码为0。
    wire [15:0] w0 = 16'h4500;
    wire [15:0] w1 = ip_total_len;
    wire [15:0] w2 = frag_ip_id;
    wire [15:0] w3 = {1'b0, 1'b0, frag_mf, frag_offset_8b};
    wire [15:0] w4 = 16'h4011;
    wire [15:0] w5 = 16'h0000;
    wire [15:0] w6 = SRC_IP[31:16];
    wire [15:0] w7 = SRC_IP[15:0];
    wire [15:0] w8 = DST_IP[31:16];
    wire [15:0] w9 = DST_IP[15:0];

    wire [19:0] chk_sum_raw = w0+w1+w2+w3+w4+w5+w6+w7+w8+w9;
    wire [16:0] chk_fold1   = {1'b0,chk_sum_raw[15:0]} + {13'd0,chk_sum_raw[19:16]};
    wire [15:0] chk_fold2   = chk_fold1[15:0] + {15'd0,chk_fold1[16]};
    wire [15:0] ip_checksum  = ~chk_fold2;

    // 头部长度: 延续分片(无UDP头)只有Ethernet(14B)+IP(20B)=34B(索引0..33)；
    // 首片/不分片场景是34B+UDP(8B)=42B(索引0..41)。
    wire [5:0]  hdr_last_idx  = cand_skip_udp_hdr ? 6'd33 : 6'd41;
    wire [11:0] hdr_total_len = cand_skip_udp_hdr ? 12'd34 : 12'd42;

    // MAC侧不输出FCS；线速帧长需加4字节FCS。
    wire [11:0] mac_frame_len  = hdr_total_len + 12'd1 + {1'b0,payload_len} + {1'b0,pad_len};
    wire [11:0] wire_frame_len = mac_frame_len + 12'd4;

    wire [10:0] max_payload_for_mode = cand_skip_udp_hdr ? MAX_PAYLOAD_LEN_CONT : MAX_PAYLOAD_LEN;

    // 对上游协议上限、题目规定的以太网最大帧长以及本VL的LMAX均做指示。
    // 不截断，保证数据完整性。
    wire len_violation_calc =
        (payload_len < 11'd1) ||
        (payload_len > max_payload_for_mode) ||
        (wire_frame_len > MAX_WIRE_FRAME) ||
        (wire_frame_len < MIN_WIRE_FRAME) ||
        (wire_frame_len > {1'b0,cand_max_len});

    // ==================== 状态机 ====================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= S_IDLE;
            cand_vlid          <= {TX_IDX_W{1'b0}};
            cand_dest_mac      <= 48'd0;
            cand_max_len       <= 11'd0;
            payload_len_reg    <= 11'd0;
            payload_remaining  <= 11'd0;
            pad_total          <= 11'd0;
            pad_remaining      <= 11'd0;
            first_byte         <= 8'd0;
            first_byte_last    <= 1'b0;
            first_byte_pending <= 1'b0;
            byte_idx           <= 6'd0;
            cand_skip_udp_hdr    <= 1'b0;
            cand_frag_ip_id      <= 16'd0;
            cand_frag_mf         <= 1'b0;
            cand_frag_offset_8b  <= 13'd0;
            // XAPP1130: 复位后每条VL的首帧SN=0(发送端复位标志)，之后1~255循环
            for (k = 0; k < NUM_TX_VL; k = k + 1)
                seq_num[k] <= 8'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (grant_valid) begin
                        cand_vlid     <= grant_vlid;
                        cand_dest_mac <= grant_dest_mac;
                        cand_max_len  <= grant_max_len;
                        state         <= S_LATCH_LEN;
                    end
                end

                // 先缓存上游第一字节，保证IP/UDP长度在发送头之前已经确定。
                S_LATCH_LEN: begin
                    if (payload_valid) begin
                        payload_len_reg     <= payload_len;
                        pad_total           <= pad_len;
                        first_byte          <= payload_data;
                        first_byte_last     <= payload_last;
                        cand_skip_udp_hdr   <= frag_skip_udp_hdr;
                        cand_frag_ip_id     <= frag_ip_id;
                        cand_frag_mf        <= frag_mf;
                        cand_frag_offset_8b <= frag_offset_8b;

                        hdr_buf[0]  <= cand_dest_mac[47:40];
                        hdr_buf[1]  <= cand_dest_mac[39:32];
                        hdr_buf[2]  <= cand_dest_mac[31:24];
                        hdr_buf[3]  <= cand_dest_mac[23:16];
                        hdr_buf[4]  <= cand_dest_mac[15:8];
                        hdr_buf[5]  <= cand_dest_mac[7:0];
                        hdr_buf[6]  <= SRC_MAC[47:40];
                        hdr_buf[7]  <= SRC_MAC[39:32];
                        hdr_buf[8]  <= SRC_MAC[31:24];
                        hdr_buf[9]  <= SRC_MAC[23:16];
                        hdr_buf[10] <= SRC_MAC[15:8];
                        hdr_buf[11] <= SRC_MAC[7:0];
                        hdr_buf[12] <= 8'h08;
                        hdr_buf[13] <= 8'h00;
                        hdr_buf[14] <= 8'h45;
                        hdr_buf[15] <= 8'h00;
                        hdr_buf[16] <= ip_total_len[15:8];
                        hdr_buf[17] <= ip_total_len[7:0];
                        hdr_buf[18] <= w2[15:8];
                        hdr_buf[19] <= w2[7:0];
                        hdr_buf[20] <= w3[15:8];
                        hdr_buf[21] <= w3[7:0];
                        hdr_buf[22] <= 8'h40;
                        hdr_buf[23] <= 8'h11;
                        hdr_buf[24] <= ip_checksum[15:8];
                        hdr_buf[25] <= ip_checksum[7:0];
                        hdr_buf[26] <= SRC_IP[31:24];
                        hdr_buf[27] <= SRC_IP[23:16];
                        hdr_buf[28] <= SRC_IP[15:8];
                        hdr_buf[29] <= SRC_IP[7:0];
                        hdr_buf[30] <= DST_IP[31:24];
                        hdr_buf[31] <= DST_IP[23:16];
                        hdr_buf[32] <= DST_IP[15:8];
                        hdr_buf[33] <= DST_IP[7:0];
                        hdr_buf[34] <= udp_src_port[15:8];
                        hdr_buf[35] <= udp_src_port[7:0];
                        hdr_buf[36] <= udp_dst_port[15:8];
                        hdr_buf[37] <= udp_dst_port[7:0];
                        hdr_buf[38] <= udp_len[15:8];
                        hdr_buf[39] <= udp_len[7:0];
                        hdr_buf[40] <= 8'h00;
                        hdr_buf[41] <= 8'h00;

                        byte_idx <= 6'd0;
                        state    <= S_SEND_HDR;
                    end
                end

                S_SEND_HDR: begin
                    if (mac_tx_ready) begin
                        if (byte_idx == hdr_last_idx) begin
                            byte_idx           <= 6'd0;
                            payload_remaining  <= (payload_len_reg == 0) ? 11'd0 : payload_len_reg - 11'd1;
                            first_byte_pending <= 1'b1;
                            state              <= S_SEND_PAYLOAD;
                        end else begin
                            byte_idx <= byte_idx + 6'd1;
                        end
                    end
                end

                S_SEND_PAYLOAD: begin
                    if (first_byte_pending) begin
                        if (mac_tx_ready) begin
                            first_byte_pending <= 1'b0;
                            if (first_byte_last) begin
                                if (pad_total != 11'd0) begin
                                    pad_remaining <= pad_total - 11'd1;
                                    state         <= S_SEND_PAD;
                                end else begin
                                    state <= S_SEND_SEQ;
                                end
                            end
                        end
                    end else if (payload_valid && mac_tx_ready) begin
                        if (payload_last) begin
                            if (pad_total != 11'd0) begin
                                pad_remaining <= pad_total - 11'd1;
                                state <= S_SEND_PAD;
                            end else begin
                                state <= S_SEND_SEQ;
                            end
                        end
                    end
                end

                S_SEND_PAD: begin
                    if (mac_tx_ready) begin
                        if (pad_remaining == 11'd0)
                            state <= S_SEND_SEQ;
                        else
                            pad_remaining <= pad_remaining - 11'd1;
                    end
                end

                S_SEND_SEQ: begin
                    if (mac_tx_ready) begin
                        seq_num[cand_vlid] <= (seq_num[cand_vlid] == 8'd255) ?
                                              8'd1 : seq_num[cand_vlid] + 8'd1;
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    assign grant_ready = (state == S_IDLE);

    assign payload_ready = (state == S_LATCH_LEN) ||
                           ((state == S_SEND_PAYLOAD) &&
                            !first_byte_pending && mac_tx_ready);

    assign mac_tx_data = (state == S_SEND_HDR) ?
                         hdr_buf[byte_idx] :
                         (state == S_SEND_PAYLOAD) ?
                         (first_byte_pending ? first_byte : payload_data) :
                         (state == S_SEND_PAD) ? 8'h00 :
                         (state == S_SEND_SEQ) ? seq_num[cand_vlid] : 8'h00;

    assign mac_tx_valid = (state == S_SEND_HDR) ||
                          (state == S_SEND_PAD) ||
                          (state == S_SEND_SEQ) ||
                          (state == S_SEND_PAYLOAD && (first_byte_pending || payload_valid));

    assign mac_tx_last = (state == S_SEND_SEQ);

    assign frame_len_violation = (state == S_SEND_SEQ) && len_violation_calc;

endmodule

