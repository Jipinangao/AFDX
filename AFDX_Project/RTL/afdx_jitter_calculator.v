//=====================================================================
// 模块名 : afdx_jitter_calculator
// 功能   : 按XAPP1130 Equation 2的真实定义计算JitterMax：
//              JitterMax = min(500us, 40us + Σ_{i∈该端口VL集合} 8×(20+LMAX_i) / NBW)
//          后台持续扫描vl_table的全部TX VL表项(entry_valid=1的)，
//          累加Σ8×(20+LMAX_i)，每扫完一整轮就刷新一次jitter_max_us。
//
// 这是对此前实现的修正说明（如实记录，不是凭空新增）：
//   此前afdx_bag_jitter_scheduler.v内部用一个"JITTER_SET_LMAX_BYTES"
//   参数(默认1518)算JitterMax，本质是"拿一个固定常数当LMAX"，既没有
//   对vl_table里实际配置的VL集合求和，公式结构也不对(缺少每条VL
//   "+20"字节的固定开销项)。用XAPP1130原文的算例可以直接验证这个错误：
//   文档里给出3条VL(LMAX=1518/1024/512字节，100Mb/s介质)算出的
//   JitterMax=289us；如果用"一个固定LMAX常数"的算法，无论如何都无法
//   同时对出这3条VL组合出的289us，因为固定常数算法根本不看VL的组合。
//   本模块改为对vl_table做真实的后台全表扫描求和。
//
//=====================================================================
module afdx_jitter_calculator #(
    parameter NUM_TX_VL  = 128,
    parameter TX_IDX_W   = 7,
    parameter MEDIUM_BPS = 100_000_000
)(
    input  wire clk,
    input  wire rst_n,

    // ---- 后台扫描用的TX查表口(经afdx_vl_tx_arbiter仲裁后接到vl_table) ----
    output reg                  lookup_valid,
    output reg  [TX_IDX_W-1:0]  lookup_vlid,
    input  wire                 lookup_rdy,
    input  wire [10:0]          max_len,
    input  wire                 entry_valid,

    output reg  [9:0]  jitter_max_us,
    output reg         calc_valid,        // 至少完成过一轮全表扫描后才置1
    output reg  [31:0] scan_pass_count    // 完成的扫描轮数，供测试/诊断确认"真的扫过"
);

    localparam integer BPS_MHZ = (MEDIUM_BPS / 1_000_000);

    localparam S_ISSUE = 1'd0;
    localparam S_WAIT  = 1'd1;

    reg              state;
    reg [TX_IDX_W-1:0] scan_idx;
    reg [31:0]       accum_bits;   // 本轮累加中的Σ8×(20+LMAX_i)，单位: bit

    wire [15:0] term_bits = 16'd160 + ({5'd0, max_len} << 3); // 8×(20+LMAX) = 160 + 8×LMAX

    // 组合出"这一轮扫描结束后应得到的JitterMax(us)"，扫完一整圈时锁存输出。
    function [9:0] jitter_from_bits;
        input [31:0] bits_sum;
        reg [31:0] extra_us;
        reg [31:0] raw_us;
        begin
            extra_us = (BPS_MHZ > 0) ? ((bits_sum + BPS_MHZ - 1) / BPS_MHZ) : 32'd500;
            raw_us   = 32'd40 + extra_us;
            jitter_from_bits = (raw_us > 32'd500) ? 10'd500 : raw_us[9:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_ISSUE;
            scan_idx        <= {TX_IDX_W{1'b0}};
            accum_bits      <= 32'd0;
            lookup_valid    <= 1'b0;
            lookup_vlid     <= {TX_IDX_W{1'b0}};
            jitter_max_us   <= 10'd40; // 尚未扫描完成前的保守起始值(=只含固定40us部分)
            calc_valid      <= 1'b0;
            scan_pass_count <= 32'd0;
        end else begin
            lookup_valid <= 1'b0;

            case (state)
                S_ISSUE: begin
                    lookup_valid <= 1'b1;
                    lookup_vlid  <= scan_idx;
                    state        <= S_WAIT;
                end

                S_WAIT: begin
                    if (lookup_rdy) begin
                        if (entry_valid)
                            accum_bits <= accum_bits + {16'd0, term_bits};

                        if (scan_idx == NUM_TX_VL-1) begin
                            // 扫完一整轮：刷新输出，重新开始下一轮。
                            jitter_max_us <= jitter_from_bits(
                                entry_valid ? (accum_bits + {16'd0, term_bits}) : accum_bits);
                            calc_valid      <= 1'b1;
                            scan_pass_count <= scan_pass_count + 32'd1;
                            accum_bits      <= 32'd0;
                            scan_idx        <= {TX_IDX_W{1'b0}};
                        end else begin
                            scan_idx <= scan_idx + 1'b1;
                        end
                        state <= S_ISSUE;
                    end
                end

                default: state <= S_ISSUE;
            endcase
        end
    end

endmodule
