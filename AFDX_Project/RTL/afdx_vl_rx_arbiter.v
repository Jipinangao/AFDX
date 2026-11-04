//=====================================================================
// 模块名 : afdx_vl_rx_arbiter
// 功能   : 让网络A、网络B两路frame_decap_filter共享同一个单端口
//          vl_table的RX查表接口。
//
// 背景：
//   AFDX的高完整性要求两个网络承载完全相同的冗余帧，因此A、B两路
//   frame_decap_filter几乎总是在同一拍发起RX查表请求(各自到达
//   VLID低字节的那一拍)，而vl_table的RX查表口是单端口、每拍只能
//   处理一次请求。直接把两路请求接到同一个vl_table会造成总线冲突。
//
// 仲裁策略：
//   同拍两路都要查表时，本拍先服务网络A、把网络B的请求锁存排队，
//   下一个空闲拍再发给vl_table。vl_table的RX查表是固定1拍返回，
//   因此仲裁只需要一个"上一拍是谁在查"的1比特寄存器，就能把返回的
//   结果准确地转发回发起请求的那一路，不会串给另一路。
//   frame_decap_filter本身能容忍查表结果"迟一点回来"（用
//   vl_lookup_pending/S_WAIT_VL处理），所以让B的请求晚1拍不会破坏
//   其正确性。
//
//=====================================================================
module afdx_vl_rx_arbiter #(
    parameter RX_IDX_W = 12
)(
    input  wire clk,
    input  wire rst_n,

    // ---- 网络A侧(frame_decap_filter A) ----
    input  wire                 a_lookup_valid,
    input  wire [RX_IDX_W-1:0]  a_lookup_vlid,
    output wire                 a_lookup_rdy,
    output wire [10:0]          a_min_len,
    output wire [10:0]          a_max_len,
    output wire                 a_entry_valid,

    // ---- 网络B侧(frame_decap_filter B) ----
    input  wire                 b_lookup_valid,
    input  wire [RX_IDX_W-1:0]  b_lookup_vlid,
    output wire                 b_lookup_rdy,
    output wire [10:0]          b_min_len,
    output wire [10:0]          b_max_len,
    output wire                 b_entry_valid,

    // ---- 共享的vl_table RX查表口 ----
    output reg                  vlt_lookup_valid,
    output reg  [RX_IDX_W-1:0]  vlt_lookup_vlid,
    input  wire                 vlt_lookup_rdy,
    input  wire [10:0]          vlt_min_len,
    input  wire [10:0]          vlt_max_len,
    input  wire                 vlt_entry_valid,

    // ---- 诊断：B请求因A同拍抢先而被排队等待的次数 ----
    output reg [31:0] arbitration_defer_count
);

    reg        b_queued;
    reg [RX_IDX_W-1:0] b_queued_vlid;
    reg        pending_sel;   // 本拍发给vl_table的请求属于哪一路: 0=A 1=B
    reg        pending_sel_d1; // 延后1拍，对齐vl_table固定1拍返回的时机

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vlt_lookup_valid <= 1'b0;
            vlt_lookup_vlid  <= {RX_IDX_W{1'b0}};
            b_queued         <= 1'b0;
            b_queued_vlid    <= {RX_IDX_W{1'b0}};
            pending_sel      <= 1'b0;
            pending_sel_d1   <= 1'b0;
            arbitration_defer_count <= 32'd0;
        end else begin
            vlt_lookup_valid <= 1'b0;
            pending_sel_d1   <= pending_sel;

            if (a_lookup_valid) begin
                // 网络A优先。若网络B这一拍也在请求，把它排到队列里下一拍再发。
                vlt_lookup_valid <= 1'b1;
                vlt_lookup_vlid  <= a_lookup_vlid;
                pending_sel      <= 1'b0;
                if (b_lookup_valid) begin
                    b_queued      <= 1'b1;
                    b_queued_vlid <= b_lookup_vlid;
                    arbitration_defer_count <= arbitration_defer_count + 32'd1;
                end
            end else if (b_lookup_valid) begin
                vlt_lookup_valid <= 1'b1;
                vlt_lookup_vlid  <= b_lookup_vlid;
                pending_sel      <= 1'b1;
            end else if (b_queued) begin
                vlt_lookup_valid <= 1'b1;
                vlt_lookup_vlid  <= b_queued_vlid;
                pending_sel      <= 1'b1;
                b_queued         <= 1'b0;
            end
        end
    end

    // vl_table的rx_lookup_rdy/数据在发起请求后固定1拍返回，
    // 用延后1拍的pending_sel_d1把结果准确转发回发起请求的那一路。
    assign a_lookup_rdy = vlt_lookup_rdy && !pending_sel_d1;
    assign b_lookup_rdy = vlt_lookup_rdy &&  pending_sel_d1;

    assign a_min_len     = vlt_min_len;
    assign a_max_len     = vlt_max_len;
    assign a_entry_valid = vlt_entry_valid;
    assign b_min_len     = vlt_min_len;
    assign b_max_len     = vlt_max_len;
    assign b_entry_valid = vlt_entry_valid;

endmodule
