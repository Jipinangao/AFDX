//=====================================================================
// 模块名 : afdx_gmii_mac_tx
// 功能   : 最小的千兆以太网发送MAC(IEEE 802.3 / GMII, 8bit@125MHz)
//          输入  : 一帧"目的MAC起、序号SN止"的字节流(s_valid一帧内连续，下降沿=帧尾)
//          输出  : GMII发送口  7字节前导码0x55 + SFD 0xD5 + 帧 + FCS(CRC-32) + 12字节IFG
//
// 做法   : 输入先经8级延迟线，让前导码+SFD(共8字节)在帧数据之前发出；
//          帧数据逐字节发出的同时累计CRC-32，帧尾后把取反的CRC按"低字节先发"追加4字节。
// 约束   : 输入帧之间至少间隔24个时钟(8级延迟线排空+FCS 4字节+IFG 12字节)。
//          上游afdx_redundant_tx整帧缓存后才开始输出，两帧起点间隔>=帧长+3，
//          最短帧(60字节)也大于该值，满足约束。
//=====================================================================
module afdx_gmii_mac_tx (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       s_valid,
    input  wire [7:0] s_data,

    output reg  [7:0] gmii_txd,
    output reg        gmii_tx_en,
    output wire       gmii_tx_er
);
    assign gmii_tx_er = 1'b0;

    // CRC-32(以太网，反射算法)按字节更新
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

    reg [7:0] dl_d [0:7];
    reg [7:0] dl_v;
    integer k;

    localparam [2:0] S_IDLE = 3'd0, S_PRE = 3'd1, S_DATA = 3'd2, S_FCS = 3'd3, S_IFG = 3'd4;
    reg [2:0]  state;
    reg [3:0]  n;
    reg [31:0] crc;
    reg [31:0] fcs_hold;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (k = 0; k < 8; k = k + 1) dl_d[k] <= 8'd0;
            dl_v <= 8'd0;
            state <= S_IDLE; n <= 4'd0; crc <= 32'hFFFFFFFF; fcs_hold <= 32'd0;
            gmii_txd <= 8'd0; gmii_tx_en <= 1'b0;
        end else begin
            // 8级延迟线(输入->d[0]->...->d[7])
            dl_d[0] <= s_data; dl_v[0] <= s_valid;
            for (k = 1; k < 8; k = k + 1) begin dl_d[k] <= dl_d[k-1]; dl_v[k] <= dl_v[k-1]; end

            gmii_tx_en <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (s_valid) begin              // 第一个数据字节到达：开始发前导码
                        gmii_txd   <= 8'h55;
                        gmii_tx_en <= 1'b1;
                        n          <= 4'd1;
                        crc        <= 32'hFFFFFFFF;
                        state      <= S_PRE;
                    end
                end
                S_PRE: begin
                    gmii_tx_en <= 1'b1;
                    gmii_txd   <= (n == 4'd7) ? 8'hD5 : 8'h55;
                    if (n == 4'd7) state <= S_DATA; else n <= n + 4'd1;
                end
                S_DATA: begin
                    if (dl_v[7]) begin
                        gmii_tx_en <= 1'b1;
                        gmii_txd   <= dl_d[7];
                        crc        <= crc32_byte(crc, dl_d[7]);
                    end else begin
                        // 帧数据发完：把取反的CRC按低字节先发
                        fcs_hold   <= ~crc;
                        state      <= S_FCS;
                        gmii_tx_en <= 1'b1;
                        gmii_txd   <= ~crc[7:0];
                        n          <= 4'd1;
                    end
                end
                S_FCS: begin
                    gmii_tx_en <= 1'b1;
                    gmii_txd   <= (n == 4'd1) ? fcs_hold[15:8] : (n == 4'd2) ? fcs_hold[23:16] : fcs_hold[31:24];
                    if (n == 4'd3) begin state <= S_IFG; n <= 4'd0; end
                    else n <= n + 4'd1;
                end
                S_IFG: begin                        // 12字节帧间隔(GMII空闲)
                    if (n == 4'd11) state <= S_IDLE; else n <= n + 4'd1;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
