//=====================================================================
// 模块名 : afdx_rx_payload_deliver
// 功能   : 把完整性/冗余管理的判决落实到载荷字节上：只把"被放行的那一份"
//          载荷交付给上层，冗余副本与校验失败的帧一律丢弃。
//
// 结构   : A、B两路各一个afdx_rx_frame_store(环形RAM + 描述符FIFO)暂存
//          载荷。帧结束时描述符入队；afdx_integrity_redundancy对每个事件
//          输出event_valid(无论放行/冗余/出错)，本模块在该拍取出对应网络队首
//          描述符，核对身份(VL+SN)后把"判决结果"压入释放队列。
//          释放引擎按队列顺序工作：被放行的帧以1字节/拍从环里读出交付，
//          其余帧直接回收空间。
//          因此放行判决发生时即使引擎还在释放前一帧(最长约1.5K拍)，新到达
//          并被放行的帧也不会丢失——只要环/队列足够，线速背靠背流量可以持续。
//
// 交付接口: deliver_valid/data/last + 该帧的vlid、IP分片信息、UDP端口，
//          deliver_*相对判决有固定的流水延迟(约3拍)。
// 诊断   : overrun_count          环或队列溢出导致的丢帧次数
//          correlation_miss_count 事件与队首描述符身份不符/无描述符的次数
//=====================================================================
module afdx_rx_payload_deliver #(
    parameter RX_IDX_W = 12,
    parameter RING_AW  = 12,      // 每网络环形缓冲 2^RING_AW 字节
    parameter DESC_AW  = 5,       // 每网络描述符FIFO深度 2^DESC_AW
    parameter REL_AW   = 6        // 释放队列深度 2^REL_AW
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- 网络A ----
    input  wire        a_payload_valid,
    input  wire [7:0]  a_payload_data,
    input  wire        a_payload_last,
    input  wire        a_frame_done,
    input  wire [RX_IDX_W-1:0] a_frame_vlid,
    input  wire [7:0]  a_frame_seq_num,
    input  wire [10:0] a_frame_payload_len,
    input  wire        a_frame_error,
    input  wire [15:0] a_frame_ip_id,
    input  wire        a_frame_mf,
    input  wire [12:0] a_frame_offset_8b,
    input  wire [15:0] a_frame_udp_src_port,
    input  wire [15:0] a_frame_udp_dst_port,

    // ---- 网络B ----
    input  wire        b_payload_valid,
    input  wire [7:0]  b_payload_data,
    input  wire        b_payload_last,
    input  wire        b_frame_done,
    input  wire [RX_IDX_W-1:0] b_frame_vlid,
    input  wire [7:0]  b_frame_seq_num,
    input  wire [10:0] b_frame_payload_len,
    input  wire        b_frame_error,
    input  wire [15:0] b_frame_ip_id,
    input  wire        b_frame_mf,
    input  wire [12:0] b_frame_offset_8b,
    input  wire [15:0] b_frame_udp_src_port,
    input  wire [15:0] b_frame_udp_dst_port,

    // ---- afdx_integrity_redundancy 的判决 ----
    input  wire        out_valid,
    input  wire [RX_IDX_W-1:0] out_vlid,
    input  wire [7:0]  out_seq_num,
    input  wire [10:0] out_frame_len,
    input  wire        out_network,
    input  wire        out_error,
    input  wire        event_valid,
    input  wire [RX_IDX_W-1:0] event_vlid,
    input  wire [7:0]  event_seq_num,
    input  wire        event_network,

    // ---- 交付 ----
    output reg         deliver_valid,
    output reg  [7:0]  deliver_data,
    output reg         deliver_last,
    output reg  [RX_IDX_W-1:0] deliver_vlid,
    output reg  [15:0] deliver_ip_id,
    output reg         deliver_mf,
    output reg  [12:0] deliver_frag_offset_8b,
    output reg  [15:0] deliver_udp_src_port,
    output reg  [15:0] deliver_udp_dst_port,

    output reg  [31:0] overrun_count,
    output reg  [31:0] correlation_miss_count
);
    localparam integer REL_N = (1 << REL_AW);

    // ---------------- 两个网络的暂存 ----------------
    wire [RING_AW-1:0] rd_addr;
    wire [7:0] a_rdata, b_rdata;
    wire a_ovp, b_ovp;

    wire a_have, b_have, a_mf, b_mf, a_ovf, b_ovf;
    wire [RX_IDX_W-1:0] a_hv, b_hv;
    wire [7:0] a_hs, b_hs;
    wire [10:0] a_hp, b_hp, a_hn, b_hn;
    wire [15:0] a_hi, b_hi, a_hsp, b_hsp, a_hdp, b_hdp;
    wire [12:0] a_ho, b_ho;
    wire [RING_AW:0] a_hst, b_hst;
    reg  a_pop, b_pop, a_free, b_free;
    reg  [RING_AW:0] free_ptr_r;

    afdx_rx_frame_store #(.RX_IDX_W(RX_IDX_W), .RING_AW(RING_AW), .DESC_AW(DESC_AW)) u_store_a (
        .clk(clk), .rst_n(rst_n),
        .payload_valid(a_payload_valid), .payload_data(a_payload_data), .frame_done(a_frame_done),
        .frame_vlid(a_frame_vlid), .frame_seq(a_frame_seq_num), .frame_plen(a_frame_payload_len),
        .frame_ipid(a_frame_ip_id), .frame_mf(a_frame_mf), .frame_off(a_frame_offset_8b),
        .frame_sport(a_frame_udp_src_port), .frame_dport(a_frame_udp_dst_port),
        .head_have(a_have), .head_vlid(a_hv), .head_seq(a_hs), .head_plen(a_hp), .head_ipid(a_hi),
        .head_mf(a_mf), .head_off(a_ho), .head_sport(a_hsp), .head_dport(a_hdp),
        .head_start(a_hst), .head_nbytes(a_hn), .head_ovf(a_ovf), .head_pop(a_pop),
        .free_valid(a_free), .free_ptr(free_ptr_r), .raddr(rd_addr), .rdata(a_rdata), .overrun_pulse(a_ovp));

    afdx_rx_frame_store #(.RX_IDX_W(RX_IDX_W), .RING_AW(RING_AW), .DESC_AW(DESC_AW)) u_store_b (
        .clk(clk), .rst_n(rst_n),
        .payload_valid(b_payload_valid), .payload_data(b_payload_data), .frame_done(b_frame_done),
        .frame_vlid(b_frame_vlid), .frame_seq(b_frame_seq_num), .frame_plen(b_frame_payload_len),
        .frame_ipid(b_frame_ip_id), .frame_mf(b_frame_mf), .frame_off(b_frame_offset_8b),
        .frame_sport(b_frame_udp_src_port), .frame_dport(b_frame_udp_dst_port),
        .head_have(b_have), .head_vlid(b_hv), .head_seq(b_hs), .head_plen(b_hp), .head_ipid(b_hi),
        .head_mf(b_mf), .head_off(b_ho), .head_sport(b_hsp), .head_dport(b_hdp),
        .head_start(b_hst), .head_nbytes(b_hn), .head_ovf(b_ovf), .head_pop(b_pop),
        .free_valid(b_free), .free_ptr(free_ptr_r), .raddr(rd_addr), .rdata(b_rdata), .overrun_pulse(b_ovp));

    // ---- 判决信号延迟2拍：描述符经同步读FIFO后约在frame_done后3拍才在队首可见，
    //      而判决事件恰好也是约3拍后到达，多等2拍留出余量 ----
    reg        ev_v1, ev_v2, ev_net1, ev_net2, ov1, ov2, oe1, oe2, on1, on2;
    reg [RX_IDX_W-1:0] ev_vl1, ev_vl2;
    reg [7:0]  ev_sn1, ev_sn2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ev_v1 <= 0; ev_v2 <= 0; ev_net1 <= 0; ev_net2 <= 0; ov1 <= 0; ov2 <= 0; oe1 <= 0; oe2 <= 0;
            on1 <= 0; on2 <= 0; ev_vl1 <= 0; ev_vl2 <= 0; ev_sn1 <= 0; ev_sn2 <= 0;
        end else begin
            ev_v1 <= event_valid; ev_net1 <= event_network; ev_vl1 <= event_vlid; ev_sn1 <= event_seq_num;
            ov1 <= out_valid; oe1 <= out_error; on1 <= out_network;
            ev_v2 <= ev_v1; ev_net2 <= ev_net1; ev_vl2 <= ev_vl1; ev_sn2 <= ev_sn1;
            ov2 <= ov1; oe2 <= oe1; on2 <= on1;
        end
    end

    // ---------------- 释放队列 ----------------
    // 条目: {sel, accept, start, nbytes, len, vlid, ipid, mf, off, sport, dport}
    localparam integer EW = 1 + 1 + (RING_AW+1) + 11 + 11 + RX_IDX_W + 16 + 1 + 13 + 16 + 16;
    wire [EW-1:0]   qh;
    wire            q_full, q_valid;
    wire            q_empty = ~q_valid;
    reg             q_pop;
    // 事件到来：取对应网络队首描述符，核对身份，压入释放队列
    wire ev_a = ev_v2 & ~ev_net2;
    wire ev_b = ev_v2 &  ev_net2;
    wire              h_have = ev_net2 ? b_have : a_have;
    wire [RX_IDX_W-1:0] h_vlid = ev_net2 ? b_hv : a_hv;
    wire [7:0]        h_seq  = ev_net2 ? b_hs : a_hs;
    wire [10:0]       h_plen = ev_net2 ? b_hp : a_hp;
    wire [15:0]       h_ipid = ev_net2 ? b_hi : a_hi;
    wire              h_mf   = ev_net2 ? b_mf : a_mf;
    wire [12:0]       h_off  = ev_net2 ? b_ho : a_ho;
    wire [15:0]       h_sp   = ev_net2 ? b_hsp : a_hsp;
    wire [15:0]       h_dp   = ev_net2 ? b_hdp : a_hdp;
    wire [RING_AW:0]  h_st   = ev_net2 ? b_hst : a_hst;
    wire [10:0]       h_nb   = ev_net2 ? b_hn : a_hn;
    wire              h_ovf  = ev_net2 ? b_ovf : a_ovf;

    wire match  = h_have && (h_vlid == ev_vl2) && (h_seq == ev_sn2);
    wire accept = match && ov2 && !oe2 && (on2 == ev_net2) && !h_ovf;
    wire [10:0] h_len = (h_plen < h_nb) ? h_plen : h_nb;
    wire push_q = ev_v2 && h_have && !q_full;

    afdx_fwft_fifo #(.DW(EW), .AW(REL_AW)) u_relq (
        .clk(clk), .rst_n(rst_n),
        .wr_en(push_q),
        .wr_data({ev_net2, accept, h_st, h_nb, h_len, h_vlid, h_ipid, h_mf, h_off, h_sp, h_dp}),
        .full(q_full), .rd_en(q_pop), .rd_data(qh), .rd_valid(q_valid));

    // ---------------- 释放引擎 ----------------
    wire              qh_sel;  wire qh_acc;
    wire [RING_AW:0]  qh_st;   wire [10:0] qh_nb, qh_len;
    wire [RX_IDX_W-1:0] qh_vl; wire [15:0] qh_ip; wire qh_mf; wire [12:0] qh_off; wire [15:0] qh_sp, qh_dp;
    assign {qh_sel, qh_acc, qh_st, qh_nb, qh_len, qh_vl, qh_ip, qh_mf, qh_off, qh_sp, qh_dp} = qh;

    reg        busy;                 // 正在流式释放
    reg        r_sel;
    reg [RING_AW:0]   r_st;
    reg [10:0] r_nb, r_len, r_idx;
    reg [RX_IDX_W-1:0] r_vl; reg [15:0] r_ip, r_sp, r_dp; reg r_mf; reg [12:0] r_off;

    assign rd_addr = r_st[RING_AW-1:0] + {{(RING_AW-11){1'b0}}, r_idx};

    reg        s1_valid, s1_last, s1_sel;
    reg [RX_IDX_W-1:0] s1_vl; reg [15:0] s1_ip, s1_sp, s1_dp; reg s1_mf; reg [12:0] s1_off;

    always @(*) begin
        a_pop = ev_a && a_have && !q_full;
        b_pop = ev_b && b_have && !q_full;
        q_pop = !busy && !q_empty;       // 释放引擎每次取走队首一项
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 1'b0; r_sel <= 1'b0; r_st <= {(RING_AW+1){1'b0}}; r_nb <= 11'd0; r_len <= 11'd0; r_idx <= 11'd0;
            r_vl <= {RX_IDX_W{1'b0}}; r_ip <= 16'd0; r_sp <= 16'd0; r_dp <= 16'd0; r_mf <= 1'b0; r_off <= 13'd0;
            a_free <= 1'b0; b_free <= 1'b0; free_ptr_r <= {(RING_AW+1){1'b0}};
            s1_valid <= 1'b0; s1_last <= 1'b0; s1_sel <= 1'b0; s1_vl <= {RX_IDX_W{1'b0}};
            s1_ip <= 16'd0; s1_sp <= 16'd0; s1_dp <= 16'd0; s1_mf <= 1'b0; s1_off <= 13'd0;
            deliver_valid <= 1'b0; deliver_data <= 8'd0; deliver_last <= 1'b0; deliver_vlid <= {RX_IDX_W{1'b0}};
            deliver_ip_id <= 16'd0; deliver_mf <= 1'b0; deliver_frag_offset_8b <= 13'd0;
            deliver_udp_src_port <= 16'd0; deliver_udp_dst_port <= 16'd0;
            overrun_count <= 32'd0; correlation_miss_count <= 32'd0;
        end else begin
            a_free <= 1'b0; b_free <= 1'b0;
            s1_valid <= 1'b0; s1_last <= 1'b0;
            deliver_valid <= 1'b0; deliver_last <= 1'b0;

            // 溢出统计
            overrun_count <= overrun_count + {31'd0, a_ovp} + {31'd0, b_ovp} + {31'd0, (ev_v2 && h_have && q_full)};

            // 事件 -> 释放队列
            if (ev_v2) begin
                if (!h_have)       correlation_miss_count <= correlation_miss_count + 32'd1;
                else if (!match)   correlation_miss_count <= correlation_miss_count + 32'd1;
            end

            // 释放引擎
            if (!busy) begin
                if (!q_empty) begin
                    if (qh_acc && qh_len != 11'd0) begin
                        busy  <= 1'b1;
                        r_sel <= qh_sel; r_st <= qh_st; r_nb <= qh_nb; r_len <= qh_len; r_idx <= 11'd0;
                        r_vl <= qh_vl; r_ip <= qh_ip; r_mf <= qh_mf; r_off <= qh_off; r_sp <= qh_sp; r_dp <= qh_dp;
                    end else begin
                        // 不交付：直接回收这一帧占用的空间
                        free_ptr_r <= qh_st + {{(RING_AW-10){1'b0}}, qh_nb};
                        if (qh_sel) b_free <= 1'b1; else a_free <= 1'b1;
                    end
                end
            end else begin
                s1_valid <= 1'b1; s1_sel <= r_sel; s1_vl <= r_vl; s1_ip <= r_ip; s1_mf <= r_mf;
                s1_off <= r_off; s1_sp <= r_sp; s1_dp <= r_dp;
                if (r_idx == r_len - 11'd1) begin
                    s1_last <= 1'b1;
                    busy    <= 1'b0;
                    free_ptr_r <= r_st + {{(RING_AW-10){1'b0}}, r_nb};
                    if (r_sel) b_free <= 1'b1; else a_free <= 1'b1;
                end else begin
                    r_idx <= r_idx + 11'd1;
                end
            end

            // 第二级：RAM输出就位后寄存交付
            if (s1_valid) begin
                deliver_valid <= 1'b1; deliver_last <= s1_last;
                deliver_data  <= s1_sel ? b_rdata : a_rdata;
                deliver_vlid  <= s1_vl; deliver_ip_id <= s1_ip; deliver_mf <= s1_mf;
                deliver_frag_offset_8b <= s1_off; deliver_udp_src_port <= s1_sp; deliver_udp_dst_port <= s1_dp;
            end
        end
    end
endmodule
