//=====================================================================
// 模块名 : afdx_mac_rx_framer
// 功能   : 由 rx_valid 的下降沿恢复"帧尾"标志。
//
// 约定   : in_err 在帧内任意一拍为1，则整帧标记错误(out_err 在
//          帧内从该字节起保持为1，帧尾字节必为1)。
// 时延   : 固定2个时钟周期。
//=====================================================================
module afdx_mac_rx_framer (
    input  wire       clk,
    input  wire       rst_n,

    input  wire [7:0] in_data,
    input  wire       in_valid,
    input  wire       in_err,

    output reg  [7:0] out_data,
    output reg        out_valid,
    output reg        out_last,
    output reg        out_err
);
    reg [7:0] d0;
    reg       v0;
    reg       e0;
    reg       err_acc;   // 帧内累计错误(不含当前d0)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d0 <= 8'd0; v0 <= 1'b0; e0 <= 1'b0;
            out_data <= 8'd0; out_valid <= 1'b0; out_last <= 1'b0; out_err <= 1'b0;
            err_acc <= 1'b0;
        end else begin
            // 第一级：采样输入
            d0 <= in_data;
            v0 <= in_valid;
            e0 <= in_valid & in_err;

            // 第二级：输出；本拍的 in_valid 即"d0 之后下一字节是否仍有效"
            out_data  <= d0;
            out_valid <= v0;
            out_last  <= v0 & ~in_valid;
            out_err   <= v0 & (err_acc | e0);

            if (v0) err_acc <= in_valid ? (err_acc | e0) : 1'b0;
            else    err_acc <= 1'b0;
        end
    end
endmodule
