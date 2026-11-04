//=====================================================================
// 模块名 : afdx_app_rx_if  —— 应用接收口适配
// 把核心交付的消息(去冗余、已重组、带VLID旁路)改成与发送口同样的格式：
//   rx_valid连续为1的一段 = 一条消息；前两个字节是VLID(高字节在前)，其后是应用数据。
//   rx_ready不使用(消息按每拍1字节连续输出，不会被反压)。
//   载荷经3级延迟线，头部2拍插入后第1个载荷字节恰好到达第3级。
//=====================================================================
module afdx_app_rx_if #(
    parameter RX_IDX_W = 12
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       p_valid,
    input  wire [7:0] p_data,
    input  wire       p_last,
    input  wire [RX_IDX_W-1:0] p_vlid,

    output reg        rx_valid,
    output reg  [7:0] rx_data
);
    reg        d0_v, d1_v, d2_v, d0_l, d1_l, d2_l;
    reg [7:0]  d0_d, d1_d, d2_d;
    reg [1:0]  hdr;                    // 2=下一拍发VLID高字节, 1=发低字节, 0=发数据
    reg        busy;
    reg [RX_IDX_W-1:0] vl;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d0_v <= 0; d1_v <= 0; d2_v <= 0; d0_l <= 0; d1_l <= 0; d2_l <= 0; d0_d <= 0; d1_d <= 0; d2_d <= 0;
            hdr <= 2'd0; busy <= 1'b0; vl <= 0; rx_valid <= 1'b0; rx_data <= 8'd0;
        end else begin
            d0_v <= p_valid; d0_d <= p_data; d0_l <= p_last;
            d1_v <= d0_v;    d1_d <= d0_d;   d1_l <= d0_l;
            d2_v <= d1_v;    d2_d <= d1_d;   d2_l <= d1_l;

            rx_valid <= 1'b0;

            if (p_valid && !busy) begin
                busy <= 1'b1; hdr <= 2'd2; vl <= p_vlid;
            end else if (hdr == 2'd2) begin
                rx_valid <= 1'b1; rx_data <= {{(8-(RX_IDX_W-8)){1'b0}}, vl[RX_IDX_W-1:8]}; hdr <= 2'd1;
            end else if (hdr == 2'd1) begin
                rx_valid <= 1'b1; rx_data <= vl[7:0]; hdr <= 2'd0;
            end else if (busy) begin
                rx_valid <= d2_v; rx_data <= d2_d;
                if (d2_v && d2_l) busy <= 1'b0;
            end
        end
    end
endmodule
