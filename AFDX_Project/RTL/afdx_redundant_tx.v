//=====================================================================
// 模块名 : afdx_redundant_tx
// 功能   : AFDX发送侧完整帧A/B双路冗余复制 (ARINC 664 Part 7: 同一帧在
//          网络A、网络B同时发送，两份副本的发送间隔须 <= 0.5ms)
//
// 做法   : 先把整帧(不含前导码/SFD/FCS，最长1514字节)存入一块同步RAM，
//          存满(in_last)后用同一个读指针逐字节读出，同一拍驱动A、B两路，
//          因此两路副本逐拍同步，发送间隔恒为0(<=0.5ms)。
//          两路副本只有一个字节不同：源MAC地址的最后一个字节(帧内第12个字节，
//          偏移11)是"网络接口标识"，ARINC 664 Part 7 / XAPP1130 Figure 5 规定
//          其高3位为接口ID(001=网络A, 010=网络B)、低5位为0，所以A路强制为0x20，
//          B路强制为0x40；其余字节完全相同。
//          out_valid在一帧内连续有效，帧与帧之间至少空1拍
//          (下一帧要先完整存入才开始读出)。
// 接口   : GMII无反压，所以没有ready；in_ready=1表示可接收新帧字节
//          (读出阶段的最后一字节发出读请求后即可接收下一帧)。
// 时延   : 帧最后一字节写入后第3拍开始输出第一个字节。
//=====================================================================
module afdx_redundant_tx #(
    parameter FRAME_MEM_DEPTH = 2048
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       in_valid,
    input  wire [7:0] in_data,
    input  wire       in_last,
    output wire       in_ready,

    output reg        out_valid,       // 网络A、网络B同拍有效
    output reg  [7:0] out_data,        // 网络A(源MAC接口ID=001)
    output reg  [7:0] out_data_b,      // 网络B(源MAC接口ID=010)
    output reg        out_last,

    output reg  [31:0] frame_count
);
    localparam S_CAPTURE = 1'b0;
    localparam S_SEND    = 1'b1;

    reg         state;
    reg  [10:0] frame_len;
    reg  [10:0] cap_idx;
    reg  [10:0] rd_idx;
    wire [7:0]  rdata;
    reg         s1_valid, s1_last;
    reg  [10:0] s1_idx;                    // s1级对应的帧内字节序号

    assign in_ready = (state == S_CAPTURE) && (cap_idx < FRAME_MEM_DEPTH);
    wire   wr_en    = in_valid && in_ready;

    // 物理深度4096(综合工具只对深度>=4096的存储体推断块RAM)，帧最长1514字节只用低2048个地址
    afdx_sdp_ram #(.AW(12), .DW(8), .DEPTH(4096)) u_frame_mem (
        .clk(clk), .we(wr_en), .waddr({1'b0, cap_idx}), .wdata(in_data), .raddr({1'b0, rd_idx}), .rdata(rdata));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_CAPTURE; frame_len <= 11'd0; cap_idx <= 11'd0; rd_idx <= 11'd0;
            s1_valid <= 1'b0; s1_last <= 1'b0; s1_idx <= 11'd0;
            out_valid <= 1'b0; out_data <= 8'd0; out_data_b <= 8'd0; out_last <= 1'b0;
            frame_count <= 32'd0;
        end else begin
            s1_valid <= 1'b0;
            s1_last  <= 1'b0;

            case (state)
                S_CAPTURE: begin
                    if (wr_en) begin
                        if (in_last) begin
                            frame_len <= cap_idx + 11'd1;
                            cap_idx   <= 11'd0;
                            rd_idx    <= 11'd0;
                            state     <= S_SEND;
                        end else begin
                            cap_idx <= cap_idx + 11'd1;
                        end
                    end
                end
                S_SEND: begin
                    s1_valid <= 1'b1;                       // 本拍发出读地址rd_idx
                    s1_idx   <= rd_idx;
                    if (rd_idx == frame_len - 11'd1) begin
                        s1_last     <= 1'b1;
                        state       <= S_CAPTURE;
                        frame_count <= frame_count + 32'd1;
                    end else begin
                        rd_idx <= rd_idx + 11'd1;
                    end
                end
            endcase

            // RAM数据与s1_*同拍到达，寄存后同时送A/B
            out_valid <= s1_valid;
            out_last  <= s1_last;
            out_data   <= (s1_idx == 11'd11) ? 8'h20 : rdata;   // 接口ID 001, 其余5位0
            out_data_b <= (s1_idx == 11'd11) ? 8'h40 : rdata;   // 接口ID 010, 其余5位0
        end
    end
endmodule
