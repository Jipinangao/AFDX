//=====================================================================
// 模块名 : ip_frag
// 功能   : AFDX发送侧IP分片，选题指南"UDP/IP协议处理"字面要求的一部分。
//          把航电分区提交的、可能超过单帧AFDX payload上限(1471字节)
//          的应用消息(最大8192字节，ARINC653常见的端口消息上限)，
//          拆成多个符合IP分片规则的片段，逐个提交给frame_encap.v
//          编码成独立的AFDX/以太网帧发送。
//
// 分片边界的精确计算依据（对照frame_encap.v的真实帧结构算出，
// 不是套用教科书里笼统的"MTU-20=1480"）：
//   - 首片(携带UDP头): frame_encap.v的AFDX Payload字段上限固定为
//     1471字节(XAPP1130原文"data payload between 1 and 1471 bytes")。
//     按IP分片规则，非最后一片的长度必须是8的整数倍；结合UDP头(8B)
//     本身已经是8的倍数，因此首片"用户数据"部分也必须是8的整数倍：
//     floor(1471/8)*8 = 1464字节。
//   - 延续片(不携带UDP头，见frame_encap.v的frag_skip_udp_hdr)：省下
//     UDP头的8字节开销后，同样的1518字节线速帧预算能容纳的纯延续数据
//     上限是1479字节(1518-14以太网-20IP-4FCS-1AFDX序列号)，比常见的
//     "MTU-20=1480"少1字节——因为AFDX在每一帧末尾都固定多加1字节序列
//     号，这是通用以太网/IP协议本身没有的开销。同理，非最后一片的
//     延续片长度也必须是8的整数倍：floor(1479/8)*8 = 1472字节。
//   - 用最大8192字节消息代入上述边界验证：1464(首片) + 1472×4(中间
//     延续片) + 840(末片) = 8192，共6片，与常见资料引用的"8192字节
//     消息最多6片"吻合。
//
// 与frame_encap.v的对接方式：
//   本模块先把分区提交的整条消息完整接收进内部缓冲区，再逐片"喂"给
//   frame_encap.v的payload端口——每一片都是一次独立的payload提交，
//   frame_encap.v据此产生一个独立的、有自己的BAG授权、自己的AFDX
//   序列号的以太网帧。frame_encap.v本身不需要知道"分片"这件事，只需
//   要本模块正确设置frag_ip_id/frag_mf/frag_offset_8b/
//   frag_skip_udp_hdr这4个字段(见frame_encap.v头注释)。
//
// 未分片的普通消息(≤1471字节)走"首片即末片"的路径，产生的字段
// (frag_mf=0, frag_offset_8b=0, frag_skip_udp_hdr=0)与修改前
// frame_encap.v的硬编码行为完全一致，因此对现有全部使用小消息的
// 测试是透明的、不引入功能差异(唯一差异是多了"先完整接收整条消息
// 再开始转发"这一步骤带来的固定延迟，数量级是消息字节数个时钟周期，
// 详见ip_frag_tb.v)。
//
// 接口契约（务必遵守，否则会重现开发过程中真实踩过的一个坑）：
//   上游必须先看到msg_ready=1才能开始拉高msg_valid提交新消息的第一个
//   字节，不能自己认为"上一条消息应该已经发完了"就直接提交。开发过程
//   中我们自己写的testbench一度没有遵守这一点——在"上一条消息刚发完
//   最后一个payload字节"的同一拍就抢着提交下一条消息的第一个字节，
//   而ip_frag内部把内部状态机从发送阶段切回接收阶段的转换也正好在这
//   一拍才真正完成，两者抢在同一拍导致下一条消息的第一个字节被状态机
//   的case分支直接吃掉、后续内容整体错位一位。这不是本模块的bug——
//   msg_ready在那一拍确实已经正确显示为0，问题出在调用方没有遵守
//   "生产者必须等ready才能发起传输"这条基本握手规则。
//=====================================================================
module ip_frag #(
    parameter MAX_MSG_LEN = 8192,
    parameter TX_IDX_W    = 7
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- 上游: 分区提交的完整应用消息 ----
    input  wire        msg_valid,
    input  wire [7:0]  msg_data,
    input  wire        msg_last,
    input  wire [13:0] msg_len,      // 应用消息总长度，最大8192
    input  wire [TX_IDX_W-1:0] msg_vlid, // 本消息所属的发送VL，消息第1个字节(msg_valid首拍)有效
    input  wire [15:0] msg_udp_src_port, // 本消息的UDP端口，消息第1个字节有效
    input  wire [15:0] msg_udp_dst_port,
    output wire        msg_ready,

    // ---- 发送状态: 消息已接收完、正在等待/进行分片发送 ----
    // tx_pending期间tx_vlid指明该消息所属VL，供BAG调度器只对该VL授权。
    output wire                 tx_pending,
    output reg  [TX_IDX_W-1:0]  tx_vlid,
    output reg  [15:0]          frag_udp_src_port,   // 随每个分片输出(同一消息的所有分片相同)
    output reg  [15:0]          frag_udp_dst_port,

    // ---- 下游: 对接frame_encap.v的payload端口 ----
    output wire          payload_valid,
    output wire [7:0]   payload_data,
    output wire           payload_last,
    output wire [10:0]  payload_len,
    input  wire          payload_ready,

    output reg  [15:0]  frag_ip_id,
    output reg           frag_mf,
    output reg  [12:0]  frag_offset_8b,
    output reg           frag_skip_udp_hdr,

    // ---- 诊断 ----
    output reg  [31:0]  frag_count,   // 已产生的分片(以太网帧)总数
    output reg  [31:0]  msg_count,    // 已处理完毕的完整消息数
    output reg  [31:0]  msg_overrun_count // 消息长度超过MAX_MSG_LEN时的丢弃计数
);

    localparam [10:0] FIRST_FRAG_MAX = 11'd1464; // 首片(带UDP头)非末片时的用户数据长度
    localparam [10:0] FIRST_FRAG_SOLO_MAX = 11'd1471; // 不需要分片时的单帧上限
    localparam [10:0] CONT_FRAG_MAX  = 11'd1472; // 延续片非末片时的长度
    localparam [10:0] CONT_FRAG_SOLO_MAX = 11'd1479; // 延续片作为末片时的长度上限

    localparam [1:0] S_RECV      = 2'd0;
    localparam [1:0] S_SETUP_SEG = 2'd1;
    localparam [1:0] S_XMIT      = 2'd2;

    reg [1:0]  state;
    reg [13:0] write_idx;
    reg [13:0] total_len;
    reg [13:0] sent_so_far;
    reg [13:0] read_idx;
    reg [10:0] seg_len;
    reg [10:0] seg_consumed;
    reg        seg_is_first;
    reg        len_overrun; // 本条消息长度超限，标记后仍完整接收但发送阶段丢弃不转发

    // ---------------- 消息缓冲：同步读RAM(可映射到Cyclone IV的M9K块RAM) ----------------
    // 综合预检(yosys synth_intel -family cycloneiv)实测：此前msg_buf是带
    // 组合读口(assign payload_data = msg_buf[read_idx])的8192x8数组，被
    // 综合器整体展开成约6.5万个触发器+巨型多路选择器，Cyclone IV放不下，
    // 是真实的综合阻塞项。改为afdx_sdp_ram(1写口+1寄存读口，读延迟1拍)。
    //
    // 读口延迟1拍之后，要保证"任何一拍的payload_data都精确对应当拍的
    // read_idx"，办法是把RAM读地址提前一拍给出——raddr恒等于"下一拍
    // read_idx将要取的值"：
    //   - S_SETUP_SEG拍：下一拍read_idx=sent_so_far(新分片起点)，同时
    //     下一拍进入S_XMIT，于是S_XMIT第一拍的payload_data已就位；
    //   - S_XMIT拍且本拍被下游接收(payload_ready)且不是最后一字节：
    //     下一拍read_idx=read_idx+1；
    //   - 其它情况read_idx保持不变，raddr=read_idx，RAM重读同一地址，
    //     payload_data保持稳定(下游不ready时数据不会变)。
    // 这样既消除了大数组展开，又保留了此前用ip_frag_encap_tb逐字节
    // 比对验证过的"数据与索引无滞后"语义；同样的回归测试直接复用。
    localparam AW = 13;   // 2^13=8192，MAX_MSG_LEN不得超过该值
    wire seg_last_byte = (seg_consumed == seg_len - 11'd1);

    wire        ram_we    = (state == S_RECV) && msg_valid && (write_idx < MAX_MSG_LEN);
    wire [AW-1:0] ram_raddr =
        (state == S_SETUP_SEG) ? sent_so_far[AW-1:0] :
        (state == S_XMIT && payload_ready && !seg_last_byte) ? (read_idx[AW-1:0] + 1'b1) :
                                                               read_idx[AW-1:0];
    wire [7:0]  ram_rdata;

    afdx_sdp_ram #(.AW(AW), .DW(8), .DEPTH(MAX_MSG_LEN)) u_msg_ram (
        .clk(clk), .we(ram_we), .waddr(write_idx[AW-1:0]), .wdata(msg_data),
        .raddr(ram_raddr), .rdata(ram_rdata)
    );

    assign payload_valid     = (state == S_XMIT);
    assign payload_data      = ram_rdata;
    assign payload_last      = (state == S_XMIT) && seg_last_byte;
    assign payload_len       = seg_len;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= S_RECV;
            write_idx         <= 14'd0;
            total_len         <= 14'd0;
            sent_so_far       <= 14'd0;
            read_idx          <= 14'd0;
            seg_len           <= 11'd0;
            seg_consumed      <= 11'd0;
            seg_is_first      <= 1'b1;
            len_overrun       <= 1'b0;
            tx_vlid           <= {TX_IDX_W{1'b0}};
            frag_udp_src_port <= 16'd0;
            frag_udp_dst_port <= 16'd0;

            frag_ip_id        <= 16'd0;
            frag_mf           <= 1'b0;
            frag_offset_8b    <= 13'd0;
            frag_skip_udp_hdr <= 1'b0;

            frag_count        <= 32'd0;
            msg_count         <= 32'd0;
            msg_overrun_count <= 32'd0;
        end else begin
            case (state)
                // ---------------- 接收完整消息 ----------------
                S_RECV: begin
                    if (msg_valid) begin
                        if (write_idx == 14'd0) begin
                            tx_vlid           <= msg_vlid;
                            frag_udp_src_port <= msg_udp_src_port;
                            frag_udp_dst_port <= msg_udp_dst_port;
                        end
                        if (msg_last) begin
                            total_len   <= (msg_len > MAX_MSG_LEN[13:0]) ? MAX_MSG_LEN[13:0] : msg_len;
                            len_overrun <= (msg_len > MAX_MSG_LEN[13:0]);
                            if (msg_len > MAX_MSG_LEN[13:0])
                                msg_overrun_count <= msg_overrun_count + 32'd1;
                            write_idx    <= 14'd0;
                            sent_so_far  <= 14'd0;
                            seg_is_first <= 1'b1;
                            state        <= S_SETUP_SEG;
                        end else begin
                            write_idx <= write_idx + 14'd1;
                        end
                    end
                end

                // ---------------- 计算下一个分片的边界(1拍) ----------------
                S_SETUP_SEG: begin
                    if (len_overrun) begin
                        // 长度超限：不生成任何分片，直接判定该消息处理完毕、丢弃。
                        msg_count <= msg_count + 32'd1;
                        state     <= S_RECV;
                    end else begin
                        read_idx <= sent_so_far;
                        if (seg_is_first) begin
                            if (total_len <= FIRST_FRAG_SOLO_MAX) begin
                                seg_len        <= total_len;
                                frag_mf        <= 1'b0;
                                frag_offset_8b <= 13'd0;
                                frag_skip_udp_hdr <= 1'b0;
                            end else begin
                                seg_len        <= FIRST_FRAG_MAX;
                                frag_mf        <= 1'b1;
                                frag_offset_8b <= 13'd0;
                                frag_skip_udp_hdr <= 1'b0;
                            end
                        end else begin
                            // 延续片在"原始IP载荷(UDP头8B+用户数据)"坐标系里的
                            // 起始偏移是 8+sent_so_far，按8字节为单位换算成
                            // FragOffset字段值。sent_so_far在首片(1464的整数倍
                            // 关系)和历次延续片(1472的整数倍)构造下，8+sent_so_far
                            // 恒为8的整数倍，右移3位不会丢精度。
                            frag_offset_8b <= (14'd8 + sent_so_far) >> 3;
                            frag_skip_udp_hdr <= 1'b1;
                            if ((total_len - sent_so_far) <= CONT_FRAG_SOLO_MAX) begin
                                seg_len <= total_len - sent_so_far;
                                frag_mf <= 1'b0;
                            end else begin
                                seg_len <= CONT_FRAG_MAX;
                                frag_mf <= 1'b1;
                            end
                        end
                        seg_consumed <= 11'd0;
                        state        <= S_XMIT;
                    end
                end

                // ---------------- 把当前分片逐字节喂给frame_encap ----------------
                // payload_valid/payload_data/payload_last/payload_len已经是
                // 上面的组合逻辑assign，这里只需要根据payload_ready推进
                // read_idx/seg_consumed，不再重复赋值这几个输出端口。
                S_XMIT: begin
                    if (payload_ready) begin
                        if (seg_consumed == seg_len - 11'd1) begin
                            // 本分片发送完毕
                            frag_count  <= frag_count + 32'd1;
                            sent_so_far <= sent_so_far + seg_len;
                            if (sent_so_far + seg_len >= total_len) begin
                                // 整条消息的全部分片都已发完
                                msg_count <= msg_count + 32'd1;
                                state     <= S_RECV;
                            end else begin
                                seg_is_first <= 1'b0;
                                state        <= S_SETUP_SEG;
                            end
                        end else begin
                            read_idx     <= read_idx + 14'd1;
                            seg_consumed <= seg_consumed + 11'd1;
                        end
                    end
                end

                default: state <= S_RECV;
            endcase

            // IP.Identification: 每完整处理完一条消息就换一个新值，供接收端
            // 区分"这是哪一条消息的分片"。从1开始(0保留，与AFDX SN习惯一致，
            // 也避免和复位后的默认值混淆)，255后回绕但跳过0。
            if (state == S_XMIT && payload_ready &&
                (seg_consumed == seg_len - 11'd1) &&
                (sent_so_far + seg_len >= total_len)) begin
                frag_ip_id <= (frag_ip_id == 16'hFFFF) ? 16'd1 : frag_ip_id + 16'd1;
            end
        end
    end

    assign msg_ready  = (state == S_RECV);
    // 超长被丢弃的消息(len_overrun)不对外申请发送，避免调度器对它产生无载荷的授权
    assign tx_pending = (state != S_RECV) && !len_overrun;

endmodule
