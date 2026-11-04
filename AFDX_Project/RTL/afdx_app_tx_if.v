//=====================================================================
// 模块名 : afdx_app_tx_if  —— 应用发送口适配
//=====================================================================
module afdx_app_tx_if #(
    parameter NUM_TX_VL = 128,
    parameter TX_IDX_W  = 7
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       tx_valid,
    input  wire [7:0] tx_data,
    output wire       tx_ready,

    input  wire       cfg_done,                 // VL表自动配置完成
    output wire [TX_IDX_W-1:0] p_vlid,
    output wire       p_valid,
    output wire [7:0] p_data,
    output wire       p_last,
    output wire [13:0] p_len,
    input  wire       p_ready,

    output reg        drop_pulse
);
    reg        v_d;                  // tx_valid上一拍
    reg        in0_v;  reg [7:0] in0_d;
    reg        skip;
    reg [13:0] cnt;
    reg [7:0]  hi;
    reg [TX_IDX_W-1:0] vl_r;
    reg        bad_vl;
    reg [2:0]  hold;

    // tx_ready只取决于内部状态，不依赖tx_valid(否则应用一拉高valid它就组合地掉下来)
    assign tx_ready = cfg_done && p_ready && !in0_v && (hold == 3'd0);

    wire start     = tx_valid && !v_d;
    wire rej_start = start && !tx_ready;
    wire last0     = in0_v && !tx_valid;

    assign p_valid = in0_v && (cnt >= 14'd2) && !bad_vl;
    assign p_data  = in0_d;
    assign p_last  = last0;
    assign p_len   = cnt - 14'd1;
    assign p_vlid  = vl_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_d <= 1'b0; in0_v <= 1'b0; in0_d <= 8'd0; skip <= 1'b0; cnt <= 14'd0;
            hi <= 8'd0; vl_r <= {TX_IDX_W{1'b0}}; bad_vl <= 1'b0; hold <= 3'd0; drop_pulse <= 1'b0;
        end else begin
            v_d        <= tx_valid;
            drop_pulse <= 1'b0;
            if (hold != 3'd0) hold <= hold - 3'd1;

            if (rej_start) begin skip <= 1'b1; drop_pulse <= 1'b1; end
            else if (!tx_valid) skip <= 1'b0;

            in0_v <= tx_valid && !skip && !rej_start;
            in0_d <= tx_data;

            if (in0_v) begin
                if (cnt == 14'd0) hi <= in0_d;
                if (cnt == 14'd1) begin
                    vl_r   <= in0_d[TX_IDX_W-1:0];
                    bad_vl <= (hi != 8'd0) || (in0_d >= NUM_TX_VL);
                    if ((hi != 8'd0) || (in0_d >= NUM_TX_VL)) drop_pulse <= 1'b1;
                end
                if (last0) begin cnt <= 14'd0; hold <= 3'd5; end
                else       cnt <= cnt + 14'd1;
            end
        end
    end
endmodule
