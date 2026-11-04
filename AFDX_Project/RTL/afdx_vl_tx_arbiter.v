//=====================================================================
// 模块名 : afdx_vl_tx_arbiter
// 功能   : 让"发送调度器的实时授权查表"(高优先级)与"抖动计算器的后台
//          全表扫描查表"(低优先级)共享同一个单端口vl_table TX查表接口。
//
// 背景：
//   ARINC664/XAPP1130 Equation 2要求JitterMax按"该物理端口上所有已配置
//   VL的Σ8×(20+LMAX_i)"动态计算，因此需要一个后台任务持续/周期性地
//   遍历vl_table的全部TX VL表项。但vl_table的TX查表口是单端口、每拍
//   只能处理一次，而发送调度器(afdx_bag_jitter_scheduler)本身也要用
//   这个口做实时的发送授权查表——两者必须共享，且不能让后台扫描
//   影响发送调度器的授权延迟(那是真实业务路径，直接影响能不能按时
//   发数据)。因此后台扫描给低优先级，只在调度器本拍不需要查表时才能
//   借用端口。
//
// 与afdx_vl_rx_arbiter.v的关系：
//   仲裁策略和时序处理方式(1拍返回、延后1拍的选路寄存器)完全沿用
//   afdx_vl_rx_arbiter.v已经过8项测试验证过的设计，这里只是把"网络A/B"
//   换成"高优先级(调度器)/低优先级(抖动计算器)"，不重新发明。
//
//=====================================================================
module afdx_vl_tx_arbiter #(
    parameter TX_IDX_W = 7
)(
    input  wire clk,
    input  wire rst_n,

    // ---- 高优先级: 发送调度器的实时授权查表 ----
    input  wire                 hi_lookup_valid,
    input  wire [TX_IDX_W-1:0]  hi_lookup_vlid,
    output wire                 hi_lookup_rdy,
    output wire [2:0]           hi_bag_sel,
    output wire [10:0]          hi_max_len,
    output wire [47:0]          hi_dest_mac,
    output wire                 hi_entry_valid,

    // ---- 低优先级: 抖动计算器的后台全表扫描查表 ----
    input  wire                 lo_lookup_valid,
    input  wire [TX_IDX_W-1:0]  lo_lookup_vlid,
    output wire                 lo_lookup_rdy,
    output wire [2:0]           lo_bag_sel,
    output wire [10:0]          lo_max_len,
    output wire [47:0]          lo_dest_mac,
    output wire                 lo_entry_valid,

    // ---- 共享的vl_table TX查表口 ----
    output reg                  vlt_lookup_valid,
    output reg  [TX_IDX_W-1:0]  vlt_lookup_vlid,
    input  wire                 vlt_lookup_rdy,
    input  wire [2:0]           vlt_bag_sel,
    input  wire [10:0]          vlt_max_len,
    input  wire [47:0]          vlt_dest_mac,
    input  wire                 vlt_entry_valid,

    output reg [31:0] arbitration_defer_count
);

    reg        lo_queued;
    reg [TX_IDX_W-1:0] lo_queued_vlid;
    reg        pending_sel;
    reg        pending_sel_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vlt_lookup_valid <= 1'b0;
            vlt_lookup_vlid  <= {TX_IDX_W{1'b0}};
            lo_queued        <= 1'b0;
            lo_queued_vlid   <= {TX_IDX_W{1'b0}};
            pending_sel      <= 1'b0;
            pending_sel_d1   <= 1'b0;
            arbitration_defer_count <= 32'd0;
        end else begin
            vlt_lookup_valid <= 1'b0;
            pending_sel_d1   <= pending_sel;

            if (hi_lookup_valid) begin
                vlt_lookup_valid <= 1'b1;
                vlt_lookup_vlid  <= hi_lookup_vlid;
                pending_sel      <= 1'b0;
                if (lo_lookup_valid) begin
                    lo_queued      <= 1'b1;
                    lo_queued_vlid <= lo_lookup_vlid;
                    arbitration_defer_count <= arbitration_defer_count + 32'd1;
                end
            end else if (lo_lookup_valid) begin
                vlt_lookup_valid <= 1'b1;
                vlt_lookup_vlid  <= lo_lookup_vlid;
                pending_sel      <= 1'b1;
            end else if (lo_queued) begin
                vlt_lookup_valid <= 1'b1;
                vlt_lookup_vlid  <= lo_queued_vlid;
                pending_sel      <= 1'b1;
                lo_queued        <= 1'b0;
            end
        end
    end

    assign hi_lookup_rdy = vlt_lookup_rdy && !pending_sel_d1;
    assign lo_lookup_rdy = vlt_lookup_rdy &&  pending_sel_d1;

    assign hi_bag_sel     = vlt_bag_sel;
    assign hi_max_len     = vlt_max_len;
    assign hi_dest_mac    = vlt_dest_mac;
    assign hi_entry_valid = vlt_entry_valid;
    assign lo_bag_sel     = vlt_bag_sel;
    assign lo_max_len     = vlt_max_len;
    assign lo_dest_mac    = vlt_dest_mac;
    assign lo_entry_valid = vlt_entry_valid;

endmodule
