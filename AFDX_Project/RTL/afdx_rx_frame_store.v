//=====================================================================
// 模块名 : afdx_rx_frame_store  (每个接收网络一个)
// 功能   : 暂存解封装后的载荷字节，等待完整性/冗余判决：
//            - 载荷字节写入环形RAM(2^RING_AW字节)；
//            - 每帧在frame_done时生成一条"描述符"(VL/SN/长度/IP分片信息/UDP端口/
//              起始地址/已写字节数)，进描述符FIFO；
//            - 判决事件到来时由上层读取队首描述符(head_*)并弹出(head_pop)；
//            - 上层把该帧载荷释放(或丢弃)完毕后，用free_*推进环形读指针，
//              空间被回收。帧严格按到达顺序回收，因此环里最老的帧永远不会
//              被新数据覆盖。
//          这样长帧在慢速释放期间(1字节/拍)到达的后续帧(包括被放行的)
//          都有地方暂存，不再丢失。
// 过载保护: 环写满 -> 该帧标记ovf(不会被交付)，overrun_pulse计数；
//           描述符FIFO满 -> 整帧丢弃并回退写指针，overrun_pulse计数。
//=====================================================================
module afdx_rx_frame_store #(
    parameter RX_IDX_W = 12,
    parameter RING_AW  = 12,
    parameter DESC_AW  = 5
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // ---- 来自frame_decap_filter ----
    input  wire                 payload_valid,
    input  wire [7:0]           payload_data,
    input  wire                 frame_done,
    input  wire [RX_IDX_W-1:0]  frame_vlid,
    input  wire [7:0]           frame_seq,
    input  wire [10:0]          frame_plen,
    input  wire [15:0]          frame_ipid,
    input  wire                 frame_mf,
    input  wire [12:0]          frame_off,
    input  wire [15:0]          frame_sport,
    input  wire [15:0]          frame_dport,

    // ---- 队首描述符(最老的未判决帧) ----
    output wire                 head_have,
    output wire [RX_IDX_W-1:0]  head_vlid,
    output wire [7:0]           head_seq,
    output wire [10:0]          head_plen,
    output wire [15:0]          head_ipid,
    output wire                 head_mf,
    output wire [12:0]          head_off,
    output wire [15:0]          head_sport,
    output wire [15:0]          head_dport,
    output wire [RING_AW:0]     head_start,
    output wire [10:0]          head_nbytes,
    output wire                 head_ovf,
    input  wire                 head_pop,

    // ---- 空间回收 ----
    input  wire                 free_valid,
    input  wire [RING_AW:0]     free_ptr,

    // ---- 读口(同步读，下一拍有效) ----
    input  wire [RING_AW-1:0]   raddr,
    output wire [7:0]           rdata,

    output reg                  overrun_pulse
);
    localparam integer RING_N = (1 << RING_AW);
    localparam integer DESC_N = (1 << DESC_AW);
    localparam integer DW = RX_IDX_W + 8 + 11 + 16 + 1 + 13 + 16 + 16 + (RING_AW+1) + 11 + 1;

    reg [RING_AW:0] wr_ptr, rd_ptr;
    reg [10:0]      cur_cnt;        // 当前帧已写入的字节数
    reg             cur_ovf;        // 当前帧遇到环满
    wire [RING_AW:0] used  = wr_ptr - rd_ptr;
    wire             we    = payload_valid && (used < RING_N);

    afdx_sdp_ram #(.AW(RING_AW), .DW(8), .DEPTH(RING_N)) u_ring (
        .clk(clk), .we(we), .waddr(wr_ptr[RING_AW-1:0]), .wdata(payload_data),
        .raddr(raddr), .rdata(rdata));

    // ---------------- 描述符FIFO(同步读RAM，先出字预读) ----------------
    wire            d_full;
    wire [10:0]     nb_now    = cur_cnt + {10'd0, we};
    wire [RING_AW:0] start_now = wr_ptr - {{(RING_AW-10){1'b0}}, cur_cnt};
    wire            ovf_now   = cur_ovf | (payload_valid & ~we);
    wire            push      = frame_done & ~d_full;

    wire [DW-1:0] hd;
    afdx_fwft_fifo #(.DW(DW), .AW(DESC_AW)) u_dfifo (
        .clk(clk), .rst_n(rst_n),
        .wr_en(push),
        .wr_data({frame_vlid, frame_seq, frame_plen, frame_ipid, frame_mf, frame_off,
                  frame_sport, frame_dport, start_now, nb_now, ovf_now}),
        .full(d_full), .rd_en(head_pop), .rd_data(hd), .rd_valid(head_have));
    assign {head_vlid, head_seq, head_plen, head_ipid, head_mf, head_off,
            head_sport, head_dport, head_start, head_nbytes, head_ovf} = hd;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= {(RING_AW+1){1'b0}};
            rd_ptr <= {(RING_AW+1){1'b0}};
            cur_cnt <= 11'd0; cur_ovf <= 1'b0;
            overrun_pulse <= 1'b0;
        end else begin
            overrun_pulse <= 1'b0;

            if (we) begin
                wr_ptr  <= wr_ptr + 1'b1;
                cur_cnt <= cur_cnt + 11'd1;
            end
            if (payload_valid && !we) cur_ovf <= 1'b1;

            if (frame_done) begin
                cur_cnt <= 11'd0;
                cur_ovf <= 1'b0;
                if (d_full) begin
                    wr_ptr <= start_now;          // 描述符满：整帧丢弃并回退
                    overrun_pulse <= 1'b1;
                end else begin
                    if (ovf_now) overrun_pulse <= 1'b1;
                end
            end

            if (free_valid) rd_ptr <= free_ptr;
        end
    end
endmodule
