//=====================================================================
// 模块名 : afdx_auto_config —— VL表自动配置
//=====================================================================
module afdx_auto_config #(
    parameter NUM_TX_VL = 128,
    parameter NUM_RX_VL = 4096,
    parameter [2:0]  BAG_SEL  = 3'd0,
    parameter [10:0] TX_LMAX  = 11'd1518,
    parameter [10:0] RX_LMIN  = 11'd64,
    parameter [10:0] RX_LMAX  = 11'd1518
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        init_done,

    output reg         cfg_we,
    output reg  [15:0] cfg_addr,
    output reg  [31:0] cfg_wdata,
    output reg         cfg_done
);
    localparam [1:0] S_WAIT = 2'd0, S_TX_LO = 2'd1, S_TX_HI = 2'd2, S_RX = 2'd3;
    reg [1:0]  state;
    reg [12:0] i;

    wire [47:0] mac_full = {32'h0300_0000, 16'd0} | {35'd0, i};   // 0300_0000_<VLID>

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_WAIT; i <= 13'd0; cfg_we <= 1'b0; cfg_addr <= 16'd0; cfg_wdata <= 32'd0; cfg_done <= 1'b0;
        end else begin
            cfg_we <= 1'b0;
            case (state)
                S_WAIT: if (init_done && !cfg_done) begin i <= 13'd0; state <= S_TX_LO; end
                S_TX_LO: begin
                    cfg_we    <= 1'b1;
                    cfg_addr  <= {8'd0, i[6:0], 1'b0};
                    cfg_wdata <= {mac_full[16:0], TX_LMAX, 1'b1, BAG_SEL};
                    state     <= S_TX_HI;
                end
                S_TX_HI: begin
                    cfg_we    <= 1'b1;
                    cfg_addr  <= {8'd0, i[6:0], 1'b1};
                    cfg_wdata <= {1'b0, mac_full[47:17]};
                    if (i == NUM_TX_VL-1) begin i <= 13'd0; state <= S_RX; end
                    else begin i <= i + 13'd1; state <= S_TX_LO; end
                end
                S_RX: begin
                    cfg_we    <= 1'b1;
                    cfg_addr  <= {1'b1, 3'd0, i[11:0]};
                    cfg_wdata <= {9'd0, 1'b1, RX_LMAX, RX_LMIN};
                    if (i == NUM_RX_VL-1) begin cfg_done <= 1'b1; state <= S_WAIT; end
                    else i <= i + 13'd1;
                end
            endcase
        end
    end
endmodule
