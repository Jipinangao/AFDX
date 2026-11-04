//=====================================================================
// 模块名 : afdx_gmii_mac_rx
// 功能   : 最小的千兆以太网接收MAC(GMII, 8bit@125MHz)
//          GMII接收口 -> 找SFD(0xD5) -> 帧字节流(去掉前导码/SFD/FCS) + 帧错误指示
// 输出   : o_valid一帧内连续，下降沿=帧尾；o_err在帧的最后一个字节为1表示
//          FCS不符或GMII报告了rx_er，下游据此丢帧。
// 做法   : 4字节延迟线剥掉FCS；整帧(含FCS)累计CRC-32，帧尾CRC应等于0xDEBB20E3；
//          输出再多保留一级，等dv下降沿才知道哪个字节是最后一个数据字节。
//=====================================================================
module afdx_gmii_mac_rx (
    input  wire       clk,
    input  wire       rst_n,

    input  wire [7:0] gmii_rxd,
    input  wire       gmii_rx_dv,
    input  wire       gmii_rx_er,

    output reg  [7:0] o_data,
    output reg        o_valid,
    output reg        o_err
);
    function [31:0] crc32_byte;
        input [31:0] crc;
        input [7:0]  d;
        integer i;
        reg [31:0] c;
        begin
            c = crc ^ {24'd0, d};
            for (i = 0; i < 8; i = i + 1)
                c = c[0] ? ((c >> 1) ^ 32'hEDB88320) : (c >> 1);
            crc32_byte = c;
        end
    endfunction

    reg        in_frame;
    reg [7:0]  d0, d1, d2, d3;        // 4字节延迟线(用来剥掉FCS)
    reg [2:0]  fill;                  // 已进入延迟线的字节数(到4为止)
    reg [31:0] crc;
    reg        er_seen;
    reg        p_v;                   // 待输出级：保留一拍，等dv下降沿才能确定它是不是最后一字节
    reg [7:0]  p_d;

    wire        cand_v    = gmii_rx_dv && in_frame && (fill == 3'd4);   // 本拍有一个字节离开延迟线
    wire [31:0] crc_next  = crc32_byte(crc, gmii_rxd);
    wire        last_p    = p_v && !cand_v;                              // 本拍没有后继字节 => p是帧最后一个数据字节
    wire        bad       = (crc != 32'hDEBB20E3) || er_seen;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_frame <= 1'b0; d0 <= 0; d1 <= 0; d2 <= 0; d3 <= 0; fill <= 0;
            crc <= 32'hFFFFFFFF; er_seen <= 1'b0; p_v <= 1'b0; p_d <= 8'd0;
            o_data <= 8'd0; o_valid <= 1'b0; o_err <= 1'b0;
        end else begin
            o_valid <= p_v;
            o_data  <= p_d;
            o_err   <= last_p && bad;

            p_v <= cand_v;
            p_d <= d3;

            if (gmii_rx_dv) begin
                if (!in_frame) begin
                    if (gmii_rxd == 8'hD5) begin          // SFD
                        in_frame <= 1'b1; fill <= 3'd0; crc <= 32'hFFFFFFFF; er_seen <= 1'b0;
                    end
                end else begin
                    crc <= crc_next;
                    if (gmii_rx_er) er_seen <= 1'b1;
                    d0 <= gmii_rxd; d1 <= d0; d2 <= d1; d3 <= d2;
                    if (fill != 3'd4) fill <= fill + 3'd1;
                end
            end else begin
                in_frame <= 1'b0;
            end
        end
    end
endmodule
