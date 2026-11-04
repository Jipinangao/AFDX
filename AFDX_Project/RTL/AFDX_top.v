module AFDX_top (
    input  wire        clk,
    input  wire        reset,

    // ---------------- 应用数据口(模板端口) ----------------
    input  wire [7:0]  tx_data,
    input  wire        tx_valid,
    output wire        tx_ready,
    output wire [7:0]  rx_data,
    output wire        rx_valid,
    input  wire        rx_ready,

    // ---------------- GMII 网络A ----------------
    output wire [7:0]  gmii_a_txd,
    output wire        gmii_a_tx_en,
    output wire        gmii_a_tx_er,
    input  wire [7:0]  gmii_a_rxd,
    input  wire        gmii_a_rx_dv,
    input  wire        gmii_a_rx_er,

    // ---------------- GMII 网络B ----------------
    output wire [7:0]  gmii_b_txd,
    output wire        gmii_b_tx_en,
    output wire        gmii_b_tx_er,
    input  wire [7:0]  gmii_b_rxd,
    input  wire        gmii_b_rx_dv,
    input  wire        gmii_b_rx_er,

    // ---------------- 状态指示 ----------------
    // err_flags_o: [0]接收缓冲溢出 [1]判决与帧描述符不符 [2]发送Jitter超限 [3]发送帧长越限
    //              [4]发送消息过长被丢 [5]重组出错(超时/重叠/溢出/孤儿) [6]应用口丢弃了一条消息 [7]保留
    output wire [7:0]  err_flags_o
);
    wire rx_ready_unused = rx_ready;
    wire rst_n = ~reset;

    // ---------------- 自动配置 ----------------
    wire        cfg_we, cfg_done, core_init_done;
    wire [15:0] cfg_addr;
    wire [31:0] cfg_wdata;
    afdx_auto_config u_auto_cfg (
        .clk(clk), .rst_n(rst_n), .init_done(core_init_done),
        .cfg_we(cfg_we), .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata), .cfg_done(cfg_done));

    // ---------------- 应用口适配 ----------------
    wire [6:0]  p_tx_vlid;
    wire        p_tx_valid, p_tx_last, p_tx_ready, app_drop;
    wire [7:0]  p_tx_data;
    wire [13:0] p_tx_len;
    afdx_app_tx_if u_app_tx (
        .clk(clk), .rst_n(rst_n), .tx_valid(tx_valid), .tx_data(tx_data), .tx_ready(tx_ready),
        .cfg_done(cfg_done),
        .p_vlid(p_tx_vlid), .p_valid(p_tx_valid), .p_data(p_tx_data), .p_last(p_tx_last),
        .p_len(p_tx_len), .p_ready(p_tx_ready), .drop_pulse(app_drop));

    wire        p_rx_valid, p_rx_last;
    wire [7:0]  p_rx_data;
    wire [11:0] p_rx_vlid;
    afdx_app_rx_if u_app_rx (
        .clk(clk), .rst_n(rst_n), .p_valid(p_rx_valid), .p_data(p_rx_data), .p_last(p_rx_last),
        .p_vlid(p_rx_vlid), .rx_valid(rx_valid), .rx_data(rx_data));

    // ---------------- 协议核心 ----------------
    wire [7:0] ca_txd, cb_txd, ca_rxd, cb_rxd;
    wire       ca_tx_v, cb_tx_v, ca_rx_v, cb_rx_v, ca_rx_e, cb_rx_e;
    wire [7:0] core_flags;

    afdx_core u_core (
        .clk(clk), .reset(reset),
        .tx_data(ca_txd), .tx_valid(ca_tx_v), .rx_data(ca_rxd), .rx_valid(ca_rx_v),
        .tx_ready(), .rx_ready(1'b0),
        .tx_data_b(cb_txd), .tx_valid_b(cb_tx_v), .rx_data_b(cb_rxd), .rx_valid_b(cb_rx_v),
        .rx_err(ca_rx_e), .rx_err_b(cb_rx_e),
        .partition_tx_vlid(p_tx_vlid), .partition_tx_valid(p_tx_valid), .partition_tx_data(p_tx_data),
        .partition_tx_last(p_tx_last), .partition_tx_len(p_tx_len), .partition_tx_ready(p_tx_ready),
        .partition_rx_valid(p_rx_valid), .partition_rx_data(p_rx_data),
        .partition_rx_last(p_rx_last), .partition_rx_vlid(p_rx_vlid),
        .cfg_sel(2'd0), .cfg_we(cfg_we), .cfg_re(1'b0), .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata),
        .cfg_rdata(), .cfg_rdy(),
        .link_status(1'b1),
        .init_done_o(core_init_done), .rtc_seconds_o(), .rtc_locked_o(), .loader_session_state_o(),
        .tx_jitter_max_us_o(), .tx_jitter_calc_valid_o(), .tx_frag_count_o(), .reasm_complete_count_o(),
        .err_flags_o(core_flags));

    reg app_drop_sticky;
    always @(posedge clk or posedge reset)
        if (reset) app_drop_sticky <= 1'b0; else if (app_drop) app_drop_sticky <= 1'b1;
    assign err_flags_o = {core_flags[7], app_drop_sticky, core_flags[5:0]};

    // ---------------- 两网MAC ----------------
    afdx_gmii_mac_tx u_mac_a_tx (.clk(clk), .rst_n(rst_n), .s_valid(ca_tx_v), .s_data(ca_txd),
        .gmii_txd(gmii_a_txd), .gmii_tx_en(gmii_a_tx_en), .gmii_tx_er(gmii_a_tx_er));
    afdx_gmii_mac_rx u_mac_a_rx (.clk(clk), .rst_n(rst_n), .gmii_rxd(gmii_a_rxd), .gmii_rx_dv(gmii_a_rx_dv),
        .gmii_rx_er(gmii_a_rx_er), .o_data(ca_rxd), .o_valid(ca_rx_v), .o_err(ca_rx_e));
    afdx_gmii_mac_tx u_mac_b_tx (.clk(clk), .rst_n(rst_n), .s_valid(cb_tx_v), .s_data(cb_txd),
        .gmii_txd(gmii_b_txd), .gmii_tx_en(gmii_b_tx_en), .gmii_tx_er(gmii_b_tx_er));
    afdx_gmii_mac_rx u_mac_b_rx (.clk(clk), .rst_n(rst_n), .gmii_rxd(gmii_b_rxd), .gmii_rx_dv(gmii_b_rx_dv),
        .gmii_rx_er(gmii_b_rx_er), .o_data(cb_rxd), .o_valid(cb_rx_v), .o_err(cb_rx_e));
endmodule
