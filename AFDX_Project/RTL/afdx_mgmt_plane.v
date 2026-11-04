//=====================================================================
// 模块名 : afdx_mgmt_plane
// 功能   : 端系统管理平面 —— 把三个基于UDP的管理服务合并到同一对UDP字节接口：
//            SNMP网络管理(UDP/161)   : afdx_snmp_agent + afdx_snmp_mib
//            ARINC 615A数据加卸载(UDP/6150) : afdx_615a_loader
//            RTC时钟同步(UDP/8319)    : afdx_rtc_sync
//          上行报文按目的端口由各服务自行识别；下行(udp_tx)由SNMP与615A共用，
//          用一个"报文级互斥"仲裁器合并(报文发送期间锁定，SNMP空闲优先)。
//          RTC同步是单向接收，不参与下行仲裁。
//          sysUpTime与RTC共用同一计时基准(rtc_uptime_ms)。
//
// 上层(AFDX_top)负责把以太网/AFDX帧里的UDP报文送到udp_rx_*，并把udp_tx_*
// 封装成AFDX帧发送，因此本模块只处理UDP载荷。
//=====================================================================
module afdx_mgmt_plane #(
    parameter CLK_FREQ_HZ = 125_000_000,
    parameter [31:0] LOADER_TARGET_HW_ID = 32'hA5A5_0001,
    parameter integer LOADER_MEM_DEPTH   = 4096
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- UDP上行(已解封装的报文载荷+端口) ----
    input  wire [7:0]  udp_rx_data,
    input  wire        udp_rx_valid,
    input  wire        udp_rx_last,
    input  wire [15:0] udp_rx_src_port,
    input  wire [15:0] udp_rx_dst_port,

    // ---- UDP下行(SNMP/615A合并) ----
    output wire [7:0]  udp_tx_data,
    output wire        udp_tx_valid,
    output wire        udp_tx_last,
    output wire [15:0] udp_tx_src_port,
    output wire [15:0] udp_tx_dst_port,
    input  wire        udp_tx_ready,

    // ---- RTC配置 / 链路与发送统计 ----
    input  wire        rtc_cfg_we,
    input  wire [15:0] rtc_cfg_addr,
    input  wire [31:0] rtc_cfg_wdata,
    input  wire        link_status,
    input  wire [31:0] tx_frames,
    input  wire [31:0] tx_errors,

    // ---- 到完整性/冗余模块与Policer的MIB读口 ----
    output wire        int_mib_re,
    output wire [15:0] int_mib_addr,
    input  wire        int_mib_rvalid,
    input  wire [31:0] int_mib_rdata,
    output wire        pol_mib_re,
    output wire [11:0] pol_mib_vlid,
    input  wire        pol_mib_rvalid,
    input  wire [31:0] pol_mib_drop_count,

    // ---- 可观测状态 ----
    output wire [31:0] rtc_seconds_o,
    output wire [9:0]  rtc_ms_o,
    output wire        rtc_locked_o,
    output wire [2:0]  loader_session_state_o,
    output wire [31:0] loader_bytes_loaded_o
);

    // ---------------- RTC ----------------
    wire [31:0] rtc_uptime_ms;
    wire [31:0] rtc_sync_age_ms, rtc_sync_count, rtc_step_count, rtc_slew_count;

    afdx_rtc_sync #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .UDP_SYNC_PORT(16'd8319),
                    .STEP_THRESHOLD_MS(32'd1000), .SLEW_STEP_MS(32'd2), .DEFAULT_LOST_MS(32'd5000)) u_rtc (
        .clk(clk), .rst_n(rst_n),
        .udp_rx_data(udp_rx_data), .udp_rx_valid(udp_rx_valid), .udp_rx_last(udp_rx_last),
        .udp_rx_dst_port(udp_rx_dst_port),
        .cfg_we(rtc_cfg_we), .cfg_addr(rtc_cfg_addr), .cfg_wdata(rtc_cfg_wdata),
        .rtc_seconds(rtc_seconds_o), .rtc_ms(rtc_ms_o), .rtc_uptime_ms(rtc_uptime_ms),
        .rtc_locked(rtc_locked_o), .sync_age_ms(rtc_sync_age_ms), .sync_count(rtc_sync_count),
        .step_count(rtc_step_count), .slew_count(rtc_slew_count)
    );

    // ---------------- 615A ----------------
    wire [7:0]  ld_tx_data;
    wire        ld_tx_valid, ld_tx_last, ld_tx_ready;
    wire [15:0] ld_tx_src_port, ld_tx_dst_port;
    wire [31:0] loader_image_len, loader_crc_errors, loader_seq_errors;
    wire [31:0] loader_complete_count, loader_failed_count, loader_upload_count;

    afdx_615a_loader #(.MEM_DEPTH(LOADER_MEM_DEPTH), .TARGET_HW_ID(LOADER_TARGET_HW_ID),
                       .UDP_LOAD_PORT(16'd6150), .MAX_BLOCK_LEN(64), .MAX_UPLOAD_LEN(64)) u_loader (
        .clk(clk), .rst_n(rst_n),
        .udp_rx_data(udp_rx_data), .udp_rx_valid(udp_rx_valid), .udp_rx_last(udp_rx_last),
        .udp_rx_src_port(udp_rx_src_port), .udp_rx_dst_port(udp_rx_dst_port),
        .udp_tx_data(ld_tx_data), .udp_tx_valid(ld_tx_valid), .udp_tx_last(ld_tx_last),
        .udp_tx_src_port(ld_tx_src_port), .udp_tx_dst_port(ld_tx_dst_port), .udp_tx_ready(ld_tx_ready),
        .session_state(loader_session_state_o), .bytes_loaded(loader_bytes_loaded_o), .image_len(loader_image_len),
        .load_crc_errors(loader_crc_errors), .load_seq_errors(loader_seq_errors),
        .load_complete_count(loader_complete_count), .load_failed_count(loader_failed_count),
        .upload_count(loader_upload_count)
    );

    // ---------------- SNMP ----------------
    wire        snmp_mib_re, snmp_mib_rvalid;
    wire [15:0] snmp_mib_addr;
    wire [31:0] snmp_mib_rdata;

    afdx_snmp_mib u_mib (
        .clk(clk), .rst_n(rst_n),
        .mib_re(snmp_mib_re), .mib_addr(snmp_mib_addr),
        .mib_rvalid(snmp_mib_rvalid), .mib_rdata(snmp_mib_rdata),
        .sys_uptime_ms(rtc_uptime_ms), .link_status(link_status),
        .tx_frames(tx_frames), .tx_errors(tx_errors),
        .int_mib_re(int_mib_re), .int_mib_addr(int_mib_addr),
        .int_mib_rvalid(int_mib_rvalid), .int_mib_rdata(int_mib_rdata),
        .pol_mib_re(pol_mib_re), .pol_mib_vlid(pol_mib_vlid),
        .pol_mib_rvalid(pol_mib_rvalid), .pol_mib_drop_count(pol_mib_drop_count)
    );

    wire [7:0]  snmp_tx_data;
    wire        snmp_tx_valid, snmp_tx_last, snmp_tx_ready;
    wire [15:0] snmp_tx_src_port, snmp_tx_dst_port;

    afdx_snmp_agent u_snmp (
        .clk(clk), .rst_n(rst_n),
        .udp_rx_data(udp_rx_data), .udp_rx_valid(udp_rx_valid), .udp_rx_last(udp_rx_last),
        .udp_rx_src_port(udp_rx_src_port), .udp_rx_dst_port(udp_rx_dst_port),
        .udp_tx_data(snmp_tx_data), .udp_tx_valid(snmp_tx_valid), .udp_tx_last(snmp_tx_last),
        .udp_tx_src_port(snmp_tx_src_port), .udp_tx_dst_port(snmp_tx_dst_port), .udp_tx_ready(snmp_tx_ready),
        .mib_re(snmp_mib_re), .mib_addr(snmp_mib_addr), .mib_rvalid(snmp_mib_rvalid), .mib_rdata(snmp_mib_rdata)
    );

    // ---------------- 下行仲裁：报文级互斥，SNMP空闲优先 ----------------
    // 未锁定时按当前谁在申请直接给ready(两个服务都是"先看到ready才置valid"，
    // 若等对方先valid会死锁)；锁定后只服务被锁定方，直到该报文的last被接收。
    reg arb_locked;
    reg arb_sel;       // 0=SNMP 1=615A
    wire grant_snmp = arb_locked ? ~arb_sel : 1'b1;
    wire grant_ld   = arb_locked ?  arb_sel : ~snmp_tx_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arb_locked <= 1'b0;
            arb_sel    <= 1'b0;
        end else begin
            if (!arb_locked) begin
                if (snmp_tx_valid)    begin arb_locked <= 1'b1; arb_sel <= 1'b0; end
                else if (ld_tx_valid) begin arb_locked <= 1'b1; arb_sel <= 1'b1; end
            end
            if (udp_tx_valid && udp_tx_ready && udp_tx_last)
                arb_locked <= 1'b0;   // 写在后面：单拍报文也能在锁定的同一拍解锁
        end
    end

    assign udp_tx_data     = snmp_tx_valid ? snmp_tx_data     : ld_tx_data;
    assign udp_tx_valid    = snmp_tx_valid | ld_tx_valid;
    assign udp_tx_last     = snmp_tx_valid ? snmp_tx_last     : ld_tx_last;
    assign udp_tx_src_port = snmp_tx_valid ? snmp_tx_src_port : ld_tx_src_port;
    assign udp_tx_dst_port = snmp_tx_valid ? snmp_tx_dst_port : ld_tx_dst_port;
    assign snmp_tx_ready   = grant_snmp && udp_tx_ready;
    assign ld_tx_ready     = grant_ld   && udp_tx_ready;
endmodule
