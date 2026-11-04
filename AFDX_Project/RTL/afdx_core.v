//=====================================================================
// 模块名 : afdx_core —— AFDX端系统协议核心(ARINC 664 Part 7)，不含MAC。
//   系统顶层AFDX_top = 应用口适配 + 自动配置 + 本核心 + 两路GMII MAC；本文件保留细粒度接口
//   (按VL的分区口、合并配置总线、MAC侧字节流)，供核心级测试使用。
//
// 数据通路：
//   RX: rx_*(A/B) -> afdx_mac_rx_framer -> frame_decap_filter(解封装/VL过滤)
//       -> afdx_integrity_redundancy(完整性检查+冗余管理)
//       -> afdx_rx_payload_deliver(只交付被放行的那一份载荷)
//       -> 路由：UDP目的端口为161/6150/8319的未分片报文 -> afdx_mgmt_plane；
//                其余 -> ip_reasm(IP重组) -> partition_rx_*
//   TX: partition_tx_* 与 管理平面应答 -> 报文仲裁 -> ip_frag(分片)
//       -> afdx_bag_jitter_scheduler(BAG/Jitter，只对该报文所属VL授权)
//       -> frame_encap(封装) -> afdx_redundant_tx(A/B复制) -> tx_*
//   管理平面(SNMP/615A/RTC)因此完全经由以太网帧收发，不存在旁路端口。
//
// 参数默认值：GMII 8bit@125MHz(时钟与接口速率匹配)。
//   MEDIUM_BPS=选题指南规定的AFDX传输速率100Mbps，用于Jitter上限公式(XAPP1130 Eq.2)的NBW。
//   MGMT_TX_VL：管理应答报文使用的发送VL(须已在vl_table中配置有效)。
//
// 使用顺序：reset释放后等待 init_done_o=1(各表扫描清零，约NUM_RX_VL个
//   周期)，再写VL表/完整性/Policer配置。
//=====================================================================
module afdx_core #(
    parameter NUM_TX_VL   = 128,
    parameter TX_IDX_W    = 7,
    parameter NUM_RX_VL   = 4096,
    parameter RX_IDX_W    = 12,
    parameter CLK_FREQ_HZ = 125_000_000,
    parameter MEDIUM_BPS  = 100_000_000,       // AFDX标称速率100Mbps，用于Jitter上限公式的NBW
    parameter MGMT_TX_VL  = 0,
    parameter [31:0] EXPECTED_OUI        = 32'h0300_0000,
    parameter [31:0] LOADER_TARGET_HW_ID = 32'hA5A5_0001,
    parameter integer LOADER_MEM_DEPTH   = 4096,
    parameter integer REASM_TIMEOUT_CYCLES = 250000,
    parameter [15:0] PART_UDP_SRC_PORT   = 16'd5000,   // 航电分区数据报文的UDP端口
    parameter [15:0] PART_UDP_DST_PORT   = 16'd5001
)(
    input  wire        clk,
    input  wire        reset,

    // ============ 模板端口(网络A) ============
    output wire [7:0]  tx_data,
    output wire        tx_valid,
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,
    output wire        tx_ready,
    input  wire        rx_ready,

    // ============ 网络B(冗余) ============
    output wire [7:0]  tx_data_b,
    output wire        tx_valid_b,
    input  wire [7:0]  rx_data_b,
    input  wire        rx_valid_b,

    input  wire        rx_err,
    input  wire        rx_err_b,

    // ============ 航电分区接口: 发送 ============
    // 一条消息 = partition_tx_valid连续有效的partition_tx_len个字节，
    // 最后一字节partition_tx_last=1；partition_tx_vlid/partition_tx_len
    // 在首字节有效(len在末字节也须有效)。须等partition_tx_ready=1才可开始。
    // 消息>1471字节时自动IP分片，每片占用该VL的一个BAG时隙。
    input  wire [TX_IDX_W-1:0] partition_tx_vlid,
    input  wire        partition_tx_valid,
    input  wire [7:0]  partition_tx_data,
    input  wire        partition_tx_last,
    input  wire [13:0] partition_tx_len,
    output wire        partition_tx_ready,

    // ============ 航电分区接口: 接收(已去冗余/重组) ============
    output wire        partition_rx_valid,
    output wire [7:0]  partition_rx_data,
    output wire        partition_rx_last,
    output wire [RX_IDX_W-1:0] partition_rx_vlid,

    // ============ 配置总线(合并为一条) ============
    // cfg_sel: 0=VL表  1=完整性/冗余  2=Policer  3=RTC。
    // 写：cfg_we=1一拍；读(仅VL表支持)：cfg_re=1一拍，cfg_rdy=1时cfg_rdata有效。
    // 地址/数据格式见各子模块说明；复位后须等init_done_o=1再配置。
    input  wire [1:0]  cfg_sel,
    input  wire        cfg_we,
    input  wire        cfg_re,
    input  wire [15:0] cfg_addr,
    input  wire [31:0] cfg_wdata,
    output wire [31:0] cfg_rdata,
    output wire        cfg_rdy,

    input  wire        link_status,

    // ============ 状态/诊断(窄口；完整计数器可经SNMP MIB读取) ============
    output wire        init_done_o,
    output wire [31:0] rtc_seconds_o,
    output wire        rtc_locked_o,
    output wire [2:0]  loader_session_state_o,
    output wire [9:0]  tx_jitter_max_us_o,       // XAPP1130 Eq.2算出的Jitter上限(us)
    output wire        tx_jitter_calc_valid_o,
    output wire [7:0]  tx_frag_count_o,
    output wire [7:0]  reasm_complete_count_o,
    // 错误/告警标志(粘滞，复位清零)：
    //   [0] 接收缓冲溢出  [1] 判决与帧描述符不符  [2] 发送Jitter超限  [3] 发送帧长越限
    //   [4] 发送消息过长被丢  [5] 重组出错(超时/重叠/溢出/孤儿)  [6] 保留  [7] 保留
    output reg  [7:0]  err_flags_o
);

    wire rst_n = ~reset;

    // 内部诊断信号(原顶层输出，现仅供标志/少量输出使用)
    wire [31:0] rx_overrun_count_o, rx_correlation_miss_count_o, rx_arbitration_defer_count_o;
    wire        tx_jitter_violation_o, tx_frame_len_violation_o;
    wire [9:0]  rtc_ms_o;
    wire [31:0] loader_bytes_loaded_o, tx_frag_count_w, tx_msg_count_o, tx_msg_overrun_count_o;
    wire [31:0] reasm_timeout_count_o, reasm_overlap_count_o, reasm_overflow_count_o, reasm_orphan_count_o, reasm_complete_count_w;
    assign tx_frag_count_o        = tx_frag_count_w[7:0];
    assign reasm_complete_count_o = reasm_complete_count_w[7:0];

    // 配置总线分发
    wire        vlt_cfg_we  = cfg_we & (cfg_sel == 2'd0);
    wire        vlt_cfg_re  = cfg_re & (cfg_sel == 2'd0);
    wire        int_cfg_we  = cfg_we & (cfg_sel == 2'd1);
    wire        pol_cfg_we  = cfg_we & (cfg_sel == 2'd2);
    wire        rtc_cfg_we  = cfg_we & (cfg_sel == 2'd3);
    wire [15:0] vlt_cfg_addr  = cfg_addr, int_cfg_addr = cfg_addr, pol_cfg_addr = cfg_addr, rtc_cfg_addr = cfg_addr;
    wire [31:0] vlt_cfg_wdata = cfg_wdata, int_cfg_wdata = cfg_wdata, pol_cfg_wdata = cfg_wdata, rtc_cfg_wdata = cfg_wdata;
    wire [31:0] vlt_cfg_rdata;
    wire        vlt_cfg_rdy;
    assign cfg_rdata = vlt_cfg_rdata;
    assign cfg_rdy   = vlt_cfg_rdy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) err_flags_o <= 8'd0;
        else begin
            if (rx_overrun_count_o != 32'd0)         err_flags_o[0] <= 1'b1;
            if (rx_correlation_miss_count_o != 32'd0) err_flags_o[1] <= 1'b1;
            if (tx_jitter_violation_o)               err_flags_o[2] <= 1'b1;
            if (tx_frame_len_violation_o)            err_flags_o[3] <= 1'b1;
            if (tx_msg_overrun_count_o != 32'd0)     err_flags_o[4] <= 1'b1;
            if ((reasm_timeout_count_o | reasm_overlap_count_o | reasm_overflow_count_o | reasm_orphan_count_o) != 32'd0)
                err_flags_o[5] <= 1'b1;
        end
    end

    assign tx_ready = 1'b1;
    wire   unused_rx_ready = rx_ready;   

    // ==================== RX: 帧边界恢复 + 解封装 ====================
    wire [7:0] a_mac_data, b_mac_data;
    wire a_mac_valid, a_mac_last, a_mac_err;
    wire b_mac_valid, b_mac_last, b_mac_err;

    afdx_mac_rx_framer u_framer_a (.clk(clk), .rst_n(rst_n),
        .in_data(rx_data), .in_valid(rx_valid), .in_err(rx_err),
        .out_data(a_mac_data), .out_valid(a_mac_valid), .out_last(a_mac_last), .out_err(a_mac_err));
    afdx_mac_rx_framer u_framer_b (.clk(clk), .rst_n(rst_n),
        .in_data(rx_data_b), .in_valid(rx_valid_b), .in_err(rx_err_b),
        .out_data(b_mac_data), .out_valid(b_mac_valid), .out_last(b_mac_last), .out_err(b_mac_err));

    wire a_rx_lookup_valid, a_rx_lookup_rdy, a_rx_entry_valid;
    wire [RX_IDX_W-1:0] a_rx_lookup_vlid;
    wire [10:0] a_rx_min_len, a_rx_max_len;
    wire a_payload_valid, a_payload_last, a_frame_done, a_frame_error, a_frame_mf;
    wire [7:0] a_payload_data, a_frame_seq_num;
    wire [RX_IDX_W-1:0] a_frame_vlid;
    wire [10:0] a_frame_payload_len;
    wire [15:0] a_frame_ip_id, a_udp_sp, a_udp_dp;
    wire [12:0] a_frame_offset_8b;

    frame_decap_filter #(.RX_IDX_W(RX_IDX_W), .EXPECTED_OUI(EXPECTED_OUI)) u_decap_a (
        .clk(clk), .rst_n(rst_n),
        .mac_rx_data(a_mac_data), .mac_rx_valid(a_mac_valid), .mac_rx_last(a_mac_last), .mac_rx_err(a_mac_err),
        .rx_lookup_valid(a_rx_lookup_valid), .rx_lookup_vlid(a_rx_lookup_vlid), .rx_lookup_rdy(a_rx_lookup_rdy),
        .rx_min_len(a_rx_min_len), .rx_max_len(a_rx_max_len), .rx_entry_valid(a_rx_entry_valid),
        .out_payload_valid(a_payload_valid), .out_payload_data(a_payload_data), .out_payload_last(a_payload_last),
        .frame_done(a_frame_done), .frame_vlid(a_frame_vlid), .frame_seq_num(a_frame_seq_num),
        .frame_payload_len(a_frame_payload_len), .frame_error(a_frame_error),
        .frame_ip_id(a_frame_ip_id), .frame_mf(a_frame_mf), .frame_offset_8b(a_frame_offset_8b),
        .frame_udp_src_port(a_udp_sp), .frame_udp_dst_port(a_udp_dp));

    wire b_rx_lookup_valid, b_rx_lookup_rdy, b_rx_entry_valid;
    wire [RX_IDX_W-1:0] b_rx_lookup_vlid;
    wire [10:0] b_rx_min_len, b_rx_max_len;
    wire b_payload_valid, b_payload_last, b_frame_done, b_frame_error, b_frame_mf;
    wire [7:0] b_payload_data, b_frame_seq_num;
    wire [RX_IDX_W-1:0] b_frame_vlid;
    wire [10:0] b_frame_payload_len;
    wire [15:0] b_frame_ip_id, b_udp_sp, b_udp_dp;
    wire [12:0] b_frame_offset_8b;

    frame_decap_filter #(.RX_IDX_W(RX_IDX_W), .EXPECTED_OUI(EXPECTED_OUI)) u_decap_b (
        .clk(clk), .rst_n(rst_n),
        .mac_rx_data(b_mac_data), .mac_rx_valid(b_mac_valid), .mac_rx_last(b_mac_last), .mac_rx_err(b_mac_err),
        .rx_lookup_valid(b_rx_lookup_valid), .rx_lookup_vlid(b_rx_lookup_vlid), .rx_lookup_rdy(b_rx_lookup_rdy),
        .rx_min_len(b_rx_min_len), .rx_max_len(b_rx_max_len), .rx_entry_valid(b_rx_entry_valid),
        .out_payload_valid(b_payload_valid), .out_payload_data(b_payload_data), .out_payload_last(b_payload_last),
        .frame_done(b_frame_done), .frame_vlid(b_frame_vlid), .frame_seq_num(b_frame_seq_num),
        .frame_payload_len(b_frame_payload_len), .frame_error(b_frame_error),
        .frame_ip_id(b_frame_ip_id), .frame_mf(b_frame_mf), .frame_offset_8b(b_frame_offset_8b),
        .frame_udp_src_port(b_udp_sp), .frame_udp_dst_port(b_udp_dp));

    // ==================== VL表 + 查表仲裁 ====================
    wire vlt_rx_lookup_valid, vlt_rx_lookup_rdy, vlt_rx_entry_valid;
    wire [RX_IDX_W-1:0] vlt_rx_lookup_vlid;
    wire [10:0] vlt_rx_min_len, vlt_rx_max_len;

    afdx_vl_rx_arbiter #(.RX_IDX_W(RX_IDX_W)) u_vl_rx_arb (
        .clk(clk), .rst_n(rst_n),
        .a_lookup_valid(a_rx_lookup_valid), .a_lookup_vlid(a_rx_lookup_vlid), .a_lookup_rdy(a_rx_lookup_rdy),
        .a_min_len(a_rx_min_len), .a_max_len(a_rx_max_len), .a_entry_valid(a_rx_entry_valid),
        .b_lookup_valid(b_rx_lookup_valid), .b_lookup_vlid(b_rx_lookup_vlid), .b_lookup_rdy(b_rx_lookup_rdy),
        .b_min_len(b_rx_min_len), .b_max_len(b_rx_max_len), .b_entry_valid(b_rx_entry_valid),
        .vlt_lookup_valid(vlt_rx_lookup_valid), .vlt_lookup_vlid(vlt_rx_lookup_vlid), .vlt_lookup_rdy(vlt_rx_lookup_rdy),
        .vlt_min_len(vlt_rx_min_len), .vlt_max_len(vlt_rx_max_len), .vlt_entry_valid(vlt_rx_entry_valid),
        .arbitration_defer_count(rx_arbitration_defer_count_o));

    wire vlt_tx_lookup_valid, vlt_tx_lookup_rdy, vlt_tx_entry_valid;
    wire [TX_IDX_W-1:0] vlt_tx_lookup_vlid;
    wire [2:0]  vlt_tx_bag_sel;
    wire [10:0] vlt_tx_max_len;
    wire [47:0] vlt_tx_dest_mac;
    wire vl_init_done;

    vl_table #(.NUM_TX_VL(NUM_TX_VL), .NUM_RX_VL(NUM_RX_VL), .TX_IDX_W(TX_IDX_W), .RX_IDX_W(RX_IDX_W)) u_vl_table (
        .clk(clk), .rst_n(rst_n),
        .reg_addr(vlt_cfg_addr), .reg_wdata(vlt_cfg_wdata), .reg_we(vlt_cfg_we), .reg_re(vlt_cfg_re),
        .reg_rdata(vlt_cfg_rdata), .reg_rdy(vlt_cfg_rdy),
        .tx_lookup_valid(vlt_tx_lookup_valid), .tx_lookup_vlid(vlt_tx_lookup_vlid), .tx_lookup_rdy(vlt_tx_lookup_rdy),
        .tx_bag_sel(vlt_tx_bag_sel), .tx_max_len(vlt_tx_max_len), .tx_dest_mac(vlt_tx_dest_mac), .tx_entry_valid(vlt_tx_entry_valid),
        .rx_lookup_valid(vlt_rx_lookup_valid), .rx_lookup_vlid(vlt_rx_lookup_vlid), .rx_lookup_rdy(vlt_rx_lookup_rdy),
        .rx_min_len(vlt_rx_min_len), .rx_max_len(vlt_rx_max_len), .rx_entry_valid(vlt_rx_entry_valid),
        .init_done(vl_init_done));

    wire tx_lookup_valid, tx_lookup_rdy, tx_entry_valid;
    wire [TX_IDX_W-1:0] tx_lookup_vlid;
    wire [2:0]  tx_bag_sel;
    wire [10:0] tx_max_len_tbl;
    wire [47:0] tx_dest_mac;
    wire jit_lookup_valid, jit_lookup_rdy, jit_entry_valid;
    wire [TX_IDX_W-1:0] jit_lookup_vlid;
    wire [10:0] jit_max_len;
    wire [31:0] tx_vl_arbitration_defer_count;

    afdx_vl_tx_arbiter #(.TX_IDX_W(TX_IDX_W)) u_vl_tx_arb (
        .clk(clk), .rst_n(rst_n),
        .hi_lookup_valid(tx_lookup_valid), .hi_lookup_vlid(tx_lookup_vlid), .hi_lookup_rdy(tx_lookup_rdy),
        .hi_bag_sel(tx_bag_sel), .hi_max_len(tx_max_len_tbl), .hi_dest_mac(tx_dest_mac), .hi_entry_valid(tx_entry_valid),
        .lo_lookup_valid(jit_lookup_valid), .lo_lookup_vlid(jit_lookup_vlid), .lo_lookup_rdy(jit_lookup_rdy),
        .lo_bag_sel(), .lo_max_len(jit_max_len), .lo_dest_mac(), .lo_entry_valid(jit_entry_valid),
        .vlt_lookup_valid(vlt_tx_lookup_valid), .vlt_lookup_vlid(vlt_tx_lookup_vlid), .vlt_lookup_rdy(vlt_tx_lookup_rdy),
        .vlt_bag_sel(vlt_tx_bag_sel), .vlt_max_len(vlt_tx_max_len), .vlt_dest_mac(vlt_tx_dest_mac), .vlt_entry_valid(vlt_tx_entry_valid),
        .arbitration_defer_count(tx_vl_arbitration_defer_count));

    wire [9:0] jitter_max_us;
    wire [31:0] jitter_scan_pass_count;
    assign tx_jitter_max_us_o = jitter_max_us;

    afdx_jitter_calculator #(.NUM_TX_VL(NUM_TX_VL), .TX_IDX_W(TX_IDX_W), .MEDIUM_BPS(MEDIUM_BPS)) u_jitter_calc (
        .clk(clk), .rst_n(rst_n),
        .lookup_valid(jit_lookup_valid), .lookup_vlid(jit_lookup_vlid), .lookup_rdy(jit_lookup_rdy),
        .max_len(jit_max_len), .entry_valid(jit_entry_valid),
        .jitter_max_us(jitter_max_us), .calc_valid(tx_jitter_calc_valid_o), .scan_pass_count(jitter_scan_pass_count));

    // ==================== 完整性检查 + 冗余管理 ====================
    wire int_out_valid, int_out_error, int_sequence_error, int_duplicate_drop, int_out_network;
    wire [RX_IDX_W-1:0] int_out_vlid;
    wire [7:0]  int_out_seq_num;
    wire [10:0] int_out_frame_len;
    wire int_event_valid, int_event_network, int_init_done;
    wire [RX_IDX_W-1:0] int_event_vlid;
    wire [7:0]  int_event_seq_num;
    wire        int_mib_re, int_mib_rvalid;
    wire [15:0] int_mib_addr;
    wire [31:0] int_mib_rdata;

    afdx_integrity_redundancy #(.NUM_RX_VL(NUM_RX_VL), .RX_IDX_W(RX_IDX_W),
                                .DEFAULT_SKEW_CYCLES(CLK_FREQ_HZ/200000)) u_integrity (
        .clk(clk), .rst_n(rst_n),
        .a_valid(a_frame_done), .a_vlid(a_frame_vlid), .a_seq_num(a_frame_seq_num),
        .a_frame_len(a_frame_payload_len), .a_error(a_frame_error),
        .b_valid(b_frame_done), .b_vlid(b_frame_vlid), .b_seq_num(b_frame_seq_num),
        .b_frame_len(b_frame_payload_len), .b_error(b_frame_error),
        .cfg_we(int_cfg_we), .cfg_addr(int_cfg_addr), .cfg_wdata(int_cfg_wdata),
        .out_valid(int_out_valid), .out_vlid(int_out_vlid), .out_seq_num(int_out_seq_num),
        .out_frame_len(int_out_frame_len), .out_network(int_out_network),
        .out_error(int_out_error), .sequence_error(int_sequence_error), .duplicate_drop(int_duplicate_drop),
        .event_valid(int_event_valid), .event_vlid(int_event_vlid), .event_seq_num(int_event_seq_num), .event_network(int_event_network),
        .mib_re(int_mib_re), .mib_addr(int_mib_addr), .mib_rvalid(int_mib_rvalid), .mib_rdata(int_mib_rdata),
        .init_done(int_init_done));

    // ==================== 载荷交付 ====================
    wire dl_valid, dl_last, dl_mf;
    wire [7:0] dl_data;
    wire [RX_IDX_W-1:0] dl_vlid;
    wire [15:0] dl_ip_id, dl_sport, dl_dport;
    wire [12:0] dl_frag_off;

    afdx_rx_payload_deliver #(.RX_IDX_W(RX_IDX_W)) u_payload_deliver (
        .clk(clk), .rst_n(rst_n),
        .a_payload_valid(a_payload_valid), .a_payload_data(a_payload_data), .a_payload_last(a_payload_last),
        .a_frame_done(a_frame_done), .a_frame_vlid(a_frame_vlid), .a_frame_seq_num(a_frame_seq_num),
        .a_frame_payload_len(a_frame_payload_len), .a_frame_error(a_frame_error),
        .a_frame_ip_id(a_frame_ip_id), .a_frame_mf(a_frame_mf), .a_frame_offset_8b(a_frame_offset_8b),
        .a_frame_udp_src_port(a_udp_sp), .a_frame_udp_dst_port(a_udp_dp),
        .b_payload_valid(b_payload_valid), .b_payload_data(b_payload_data), .b_payload_last(b_payload_last),
        .b_frame_done(b_frame_done), .b_frame_vlid(b_frame_vlid), .b_frame_seq_num(b_frame_seq_num),
        .b_frame_payload_len(b_frame_payload_len), .b_frame_error(b_frame_error),
        .b_frame_ip_id(b_frame_ip_id), .b_frame_mf(b_frame_mf), .b_frame_offset_8b(b_frame_offset_8b),
        .b_frame_udp_src_port(b_udp_sp), .b_frame_udp_dst_port(b_udp_dp),
        .out_valid(int_out_valid), .out_vlid(int_out_vlid), .out_seq_num(int_out_seq_num),
        .out_frame_len(int_out_frame_len), .out_network(int_out_network), .out_error(int_out_error),
        .event_valid(int_event_valid), .event_vlid(int_event_vlid), .event_seq_num(int_event_seq_num), .event_network(int_event_network),
        .deliver_valid(dl_valid), .deliver_data(dl_data), .deliver_last(dl_last), .deliver_vlid(dl_vlid),
        .deliver_ip_id(dl_ip_id), .deliver_mf(dl_mf), .deliver_frag_offset_8b(dl_frag_off),
        .deliver_udp_src_port(dl_sport), .deliver_udp_dst_port(dl_dport),
        .overrun_count(rx_overrun_count_o), .correlation_miss_count(rx_correlation_miss_count_o));

    // ==================== 路由: 管理报文 vs 航电分区数据 ====================
    wire is_mgmt = !dl_mf && (dl_frag_off == 13'd0) &&
                   (dl_dport == 16'd161 || dl_dport == 16'd6150 || dl_dport == 16'd8319);
    wire mg_rx_valid = dl_valid &  is_mgmt;
    wire ra_in_valid = dl_valid & ~is_mgmt;

    ip_reasm #(.RX_IDX_W(RX_IDX_W), .NUM_CTX(4), .MAX_MSG_BYTES(8192),
               .TIMEOUT_CYCLES(REASM_TIMEOUT_CYCLES)) u_ip_reasm (
        .clk(clk), .rst_n(rst_n),
        .in_valid(ra_in_valid), .in_data(dl_data), .in_last(dl_last),
        .in_vlid(dl_vlid), .in_ip_id(dl_ip_id), .in_mf(dl_mf), .in_frag_offset_8b(dl_frag_off),
        .out_valid(partition_rx_valid), .out_data(partition_rx_data),
        .out_last(partition_rx_last), .out_vlid(partition_rx_vlid),
        .reasm_timeout_count(reasm_timeout_count_o), .reasm_overlap_count(reasm_overlap_count_o),
        .reasm_overflow_count(reasm_overflow_count_o), .reasm_orphan_count(reasm_orphan_count_o),
        .reasm_complete_count(reasm_complete_count_w));

    // ==================== Policer(入口流量统计) ====================
    wire pol_mib_re, pol_mib_rvalid, pol_init_done, pol_allow, pol_policed;
    wire [11:0] pol_mib_vlid;
    wire [31:0] pol_mib_drop_count, pol_time_ms;
    wire pol_ms_tick;

    afdx_token_bucket_policer #(.NUM_RX_VL(NUM_RX_VL), .RX_IDX_W(RX_IDX_W), .CLK_FREQ_HZ(CLK_FREQ_HZ)) u_policer (
        .clk(clk), .rst_n(rst_n), .ms_tick(pol_ms_tick), .time_ms(pol_time_ms),
        .frame_valid(int_out_valid && !int_out_error), .frame_vlid(int_out_vlid), .frame_len(int_out_frame_len),
        .frame_allow(pol_allow), .frame_policed(pol_policed),
        .cfg_we(pol_cfg_we), .cfg_addr(pol_cfg_addr), .cfg_wdata(pol_cfg_wdata),
        .mib_re(pol_mib_re), .mib_vlid(pol_mib_vlid[RX_IDX_W-1:0]),
        .mib_rvalid(pol_mib_rvalid), .mib_drop_count(pol_mib_drop_count),
        .init_done(pol_init_done));

    assign init_done_o = vl_init_done & int_init_done & pol_init_done;

    // ==================== TX: 报文来源仲裁(分区 / 管理平面) ====================
    wire [7:0]  mg_tx_data;
    wire        mg_tx_valid, mg_tx_last, mg_tx_ready;
    wire [15:0] mg_tx_sport, mg_tx_dport;

    wire        fr_msg_ready;
    reg  [1:0]  tx_owner;        // 0=空闲 1=分区 2=管理平面
    reg  [13:0] mg_cnt;

    wire sel_part = (tx_owner == 2'd1) || (tx_owner == 2'd0 && partition_tx_valid);
    wire sel_mgmt = (tx_owner == 2'd2) || (tx_owner == 2'd0 && !partition_tx_valid && mg_tx_valid);

    wire        fr_valid = sel_part ? partition_tx_valid : (sel_mgmt & mg_tx_valid);
    wire [7:0]  fr_data  = sel_part ? partition_tx_data  : mg_tx_data;
    wire        fr_last  = sel_part ? partition_tx_last  : mg_tx_last;
    wire [13:0] fr_len   = sel_part ? partition_tx_len   : (mg_cnt + 14'd1);
    wire [TX_IDX_W-1:0] fr_vlid = sel_part ? partition_tx_vlid : MGMT_TX_VL[TX_IDX_W-1:0];
    wire [15:0] fr_sport = sel_part ? PART_UDP_SRC_PORT : mg_tx_sport;
    wire [15:0] fr_dport = sel_part ? PART_UDP_DST_PORT : mg_tx_dport;

    assign partition_tx_ready = fr_msg_ready & (tx_owner != 2'd2);
    // ready不依赖mg_tx_valid(管理服务"先看到ready才置valid")；分区同拍申请时分区优先
    assign mg_tx_ready        = fr_msg_ready & ((tx_owner == 2'd2) | ((tx_owner == 2'd0) & ~partition_tx_valid));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_owner <= 2'd0;
            mg_cnt   <= 14'd0;
        end else begin
            if (fr_valid && fr_msg_ready) begin
                if (fr_last) begin
                    tx_owner <= 2'd0;
                    mg_cnt   <= 14'd0;
                end else begin
                    tx_owner <= sel_part ? 2'd1 : 2'd2;
                    if (sel_mgmt) mg_cnt <= mg_cnt + 14'd1;
                end
            end
        end
    end

    // ==================== TX: 分片 -> BAG/Jitter调度 -> 封装 -> 冗余发送 ====================
    wire ip_payload_valid, ip_payload_last, ip_payload_ready, ip_frag_mf, ip_frag_skip_udp_hdr;
    wire [7:0]  ip_payload_data;
    wire [10:0] ip_payload_len;
    wire [15:0] ip_frag_ip_id, ip_udp_sp, ip_udp_dp;
    wire [12:0] ip_frag_offset_8b;
    wire        fr_tx_pending;
    wire [TX_IDX_W-1:0] fr_tx_vlid;

    ip_frag #(.MAX_MSG_LEN(8192), .TX_IDX_W(TX_IDX_W)) u_ip_frag (
        .clk(clk), .rst_n(rst_n),
        .msg_valid(fr_valid), .msg_data(fr_data), .msg_last(fr_last), .msg_len(fr_len),
        .msg_vlid(fr_vlid), .msg_udp_src_port(fr_sport), .msg_udp_dst_port(fr_dport),
        .msg_ready(fr_msg_ready), .tx_pending(fr_tx_pending), .tx_vlid(fr_tx_vlid),
        .frag_udp_src_port(ip_udp_sp), .frag_udp_dst_port(ip_udp_dp),
        .payload_valid(ip_payload_valid), .payload_data(ip_payload_data),
        .payload_last(ip_payload_last), .payload_len(ip_payload_len), .payload_ready(ip_payload_ready),
        .frag_ip_id(ip_frag_ip_id), .frag_mf(ip_frag_mf),
        .frag_offset_8b(ip_frag_offset_8b), .frag_skip_udp_hdr(ip_frag_skip_udp_hdr),
        .frag_count(tx_frag_count_w), .msg_count(tx_msg_count_o), .msg_overrun_count(tx_msg_overrun_count_o));

    // 发送路径只有一个消息缓冲："已接收完整消息、等待发送"的那一条VL向调度器申请

    wire grant_valid, grant_ready;
    wire [TX_IDX_W-1:0] grant_vlid;
    wire [47:0] grant_dest_mac;
    wire [10:0] grant_max_len;

    afdx_bag_jitter_scheduler #(.NUM_TX_VL(NUM_TX_VL), .TX_IDX_W(TX_IDX_W), .CLK_FREQ_HZ(CLK_FREQ_HZ)) u_sched (
        .clk(clk), .rst_n(rst_n), .req_valid(fr_tx_pending), .req_vlid(fr_tx_vlid),
        .tx_lookup_valid(tx_lookup_valid), .tx_lookup_vlid(tx_lookup_vlid), .tx_lookup_rdy(tx_lookup_rdy),
        .tx_bag_sel(tx_bag_sel), .tx_max_len(tx_max_len_tbl), .tx_dest_mac(tx_dest_mac), .tx_entry_valid(tx_entry_valid),
        .grant_valid(grant_valid), .grant_vlid(grant_vlid), .grant_dest_mac(grant_dest_mac),
        .grant_max_len(grant_max_len), .grant_ready(grant_ready),
        .jitter_max_us(jitter_max_us), .jitter_violation(tx_jitter_violation_o));

    wire pre_valid, pre_last, pre_ready;
    wire [7:0] pre_data;

    frame_encap #(.TX_IDX_W(TX_IDX_W), .NUM_TX_VL(NUM_TX_VL)) u_encap (
        .clk(clk), .rst_n(rst_n),
        .grant_valid(grant_valid), .grant_vlid(grant_vlid), .grant_dest_mac(grant_dest_mac),
        .grant_max_len(grant_max_len), .grant_ready(grant_ready),
        .payload_valid(ip_payload_valid), .payload_data(ip_payload_data),
        .payload_last(ip_payload_last), .payload_len(ip_payload_len), .payload_ready(ip_payload_ready),
        .frag_ip_id(ip_frag_ip_id), .frag_mf(ip_frag_mf),
        .frag_offset_8b(ip_frag_offset_8b), .frag_skip_udp_hdr(ip_frag_skip_udp_hdr),
        .udp_src_port(ip_udp_sp), .udp_dst_port(ip_udp_dp),
        .mac_tx_data(pre_data), .mac_tx_valid(pre_valid), .mac_tx_last(pre_last), .mac_tx_ready(pre_ready),
        .frame_len_violation(tx_frame_len_violation_o));

    wire        red_last_unused;
    wire [31:0] tx_frame_count;

    // A/B两路由同一个读指针同时驱动(间隔0，<=0.5ms)；两路仅源MAC接口ID字节不同(A=0x20,B=0x40)
    afdx_redundant_tx u_redundant_tx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(pre_valid), .in_data(pre_data), .in_last(pre_last), .in_ready(pre_ready),
        .out_valid(tx_valid), .out_data(tx_data), .out_data_b(tx_data_b), .out_last(red_last_unused), .frame_count(tx_frame_count));
    assign tx_valid_b = tx_valid;

    // 发送错误计数(帧长超出VL的LMAX或协议上限)，供SNMP txErrors
    reg        flv_d;
    reg [31:0] tx_err_cnt;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin flv_d <= 1'b0; tx_err_cnt <= 32'd0; end
        else begin flv_d <= tx_frame_len_violation_o; if (tx_frame_len_violation_o && !flv_d) tx_err_cnt <= tx_err_cnt + 32'd1; end

    // ==================== 管理平面 ====================
    afdx_mgmt_plane #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .LOADER_TARGET_HW_ID(LOADER_TARGET_HW_ID),
                      .LOADER_MEM_DEPTH(LOADER_MEM_DEPTH)) u_mgmt (
        .clk(clk), .rst_n(rst_n),
        .udp_rx_data(dl_data), .udp_rx_valid(mg_rx_valid), .udp_rx_last(dl_last),
        .udp_rx_src_port(dl_sport), .udp_rx_dst_port(dl_dport),
        .udp_tx_data(mg_tx_data), .udp_tx_valid(mg_tx_valid), .udp_tx_last(mg_tx_last),
        .udp_tx_src_port(mg_tx_sport), .udp_tx_dst_port(mg_tx_dport), .udp_tx_ready(mg_tx_ready),
        .rtc_cfg_we(rtc_cfg_we), .rtc_cfg_addr(rtc_cfg_addr), .rtc_cfg_wdata(rtc_cfg_wdata),
        .link_status(link_status), .tx_frames(tx_frame_count), .tx_errors(tx_err_cnt),
        .int_mib_re(int_mib_re), .int_mib_addr(int_mib_addr),
        .int_mib_rvalid(int_mib_rvalid), .int_mib_rdata(int_mib_rdata),
        .pol_mib_re(pol_mib_re), .pol_mib_vlid(pol_mib_vlid),
        .pol_mib_rvalid(pol_mib_rvalid), .pol_mib_drop_count(pol_mib_drop_count),
        .rtc_seconds_o(rtc_seconds_o), .rtc_ms_o(rtc_ms_o), .rtc_locked_o(rtc_locked_o),
        .loader_session_state_o(loader_session_state_o), .loader_bytes_loaded_o(loader_bytes_loaded_o));
endmodule
