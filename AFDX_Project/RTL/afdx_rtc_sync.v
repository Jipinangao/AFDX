//=====================================================================
// 模块名 : afdx_rtc_sync
// 功能   : AFDX端系统 RTC(实时时钟)与网络时间同步
//
// 设计范围与简化说明：
//   ARINC 664/AFDX标准本身并未规定具体的网络时间同步协议实现细节，
//   工程上通常由网络管理功能向各端系统下发时间基准。本模块实现的是
//   这一概念的简化闭环：
//     1) 内部维护一个由本地时钟驱动的自由运行RTC（秒+毫秒）；
//     2) 通过UDP接收网络时间基准报文（6字节：seconds[31:0]BE +
//        ms[15:0]BE，取低10位0..999），据此对本地RTC做修正；
//     3) 修正策略区分"小偏差牵引(slew)"与"大偏差跳变(step)"两种情况，
//        参考通用时钟同步系统(如NTP/PTP)的做法：小偏差每次同步只缓慢
//        修正SLEW_STEP_MS，避免时间跳变对依赖RTC的其它功能(如日志时
//        间戳排序)造成扰动；偏差超过STEP_THRESHOLD_MS判定为复位/失步
//        场景，直接跳变对齐，并计入step_count供诊断；
//     4) 超过LOST_THRESHOLD_MS未收到有效同步报文，rtc_locked置0，
//        表示时钟进入"失步"(holdover)状态，但RTC本身不停止计数。
//   本模块不实现完整的PTP/IEEE1588或某具体民航时间同步规范的报文格式，
//   端口号和报文格式为本项目自定义，仅用于演示RTC同步闭环的核心行为。
//=====================================================================
module afdx_rtc_sync #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [15:0]  UDP_SYNC_PORT     = 16'd8319,
    parameter [31:0]  STEP_THRESHOLD_MS = 32'd1000,
    parameter [31:0]  SLEW_STEP_MS      = 32'd2,
    parameter [31:0]  DEFAULT_LOST_MS   = 32'd5000
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- 网络时间基准报文(UDP payload) ----
    input  wire [7:0]  udp_rx_data,
    input  wire        udp_rx_valid,
    input  wire        udp_rx_last,
    input  wire [15:0] udp_rx_dst_port,

    // ---- 配置：cfg_addr=0 设置失步门限(ms) ----
    input  wire        cfg_we,
    input  wire [15:0] cfg_addr,
    input  wire [31:0] cfg_wdata,

    // ---- RTC输出 ----
    output reg  [31:0] rtc_seconds,
    output reg  [9:0]  rtc_ms,          // 0..999
    output reg  [31:0] rtc_uptime_ms,   // 自复位以来运行时间，供SNMP sysUpTime使用
    output reg          rtc_locked,
    output reg  [31:0] sync_age_ms,
    output reg  [31:0] sync_count,
    output reg  [31:0] step_count,
    output reg  [31:0] slew_count
);

    localparam integer CYCLES_PER_MS = CLK_FREQ_HZ / 1000;

    // ==================== 1ms 时间基准 ====================
    reg [31:0] clk_ms_counter;
    reg        ms_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_ms_counter <= 32'd0;
            ms_tick        <= 1'b0;
        end else begin
            ms_tick <= 1'b0;
            if (clk_ms_counter == CYCLES_PER_MS - 1) begin
                clk_ms_counter <= 32'd0;
                ms_tick        <= 1'b1;
            end else begin
                clk_ms_counter <= clk_ms_counter + 32'd1;
            end
        end
    end

    // ==================== 网络时间基准报文接收 ====================
    localparam S_RX    = 2'd0;
    localparam S_APPLY = 2'd1;

    reg [1:0] rx_state;
    reg [7:0] rx_buf [0:5];
    reg [2:0] rx_cnt;

    reg [31:0] sync_seconds;
    reg [9:0]  sync_ms;
    reg        sync_pending;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state     <= S_RX;
            rx_cnt       <= 3'd0;
            sync_pending <= 1'b0;
            sync_seconds <= 32'd0;
            sync_ms      <= 10'd0;
        end else begin
            sync_pending <= 1'b0;
            case (rx_state)
                S_RX: begin
                    if (udp_rx_valid && udp_rx_dst_port == UDP_SYNC_PORT) begin
                        if (rx_cnt < 3'd6)
                            rx_buf[rx_cnt] <= udp_rx_data;
                        if (udp_rx_last) begin
                            // 只接受长度恰为6字节的报文；不足或超长均视为畸形，丢弃不生效。
                            // 注意：rx_buf[rx_cnt]<=udp_rx_data是本拍的非阻塞赋值，本拍内
                            // 直接读rx_buf[rx_cnt]会读到写入前的旧值；最后一字节必须直接
                            // 使用当拍的udp_rx_data（做法与frame_decap_filter一致）。
                            if (rx_cnt == 3'd5) begin
                                sync_seconds <= {rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]};
                                sync_ms      <= {rx_buf[4][1:0], udp_rx_data};
                                sync_pending <= 1'b1;
                            end
                            rx_cnt <= 3'd0;
                        end else begin
                            rx_cnt <= rx_cnt + 3'd1;
                        end
                    end
                end
                default: rx_state <= S_RX;
            endcase
        end
    end

    // ==================== RTC主体 + 同步修正 ====================
    reg [31:0] lost_threshold_ms;

    // 组合计算：网络基准时刻与本地RTC的差值（毫秒，带符号）。
    // 1000为常数，综合为一次常数乘法，不构成关键路径问题。
    wire signed [63:0] diff_ms =
        ($signed({1'b0, sync_seconds}) - $signed({1'b0, rtc_seconds})) * 64'sd1000 +
        ($signed({1'b0, sync_ms})      - $signed({1'b0, rtc_ms}));

    wire diff_is_large = (diff_ms >  $signed({32'd0, STEP_THRESHOLD_MS})) ||
                         (diff_ms < -$signed({32'd0, STEP_THRESHOLD_MS}));

    reg signed [31:0] slew_correction;
    reg signed [31:0] new_ms_signed;

    always @(*) begin
        // 每次同步事件最多牵引SLEW_STEP_MS，其余偏差留待下次同步继续收敛。
        if (diff_ms > 0)
            slew_correction = (diff_ms > $signed({32'd0, SLEW_STEP_MS})) ?
                              $signed(SLEW_STEP_MS) : diff_ms[31:0];
        else if (diff_ms < 0)
            slew_correction = (-diff_ms > $signed({32'd0, SLEW_STEP_MS})) ?
                              -$signed(SLEW_STEP_MS) : diff_ms[31:0];
        else
            slew_correction = 32'sd0;

        new_ms_signed = $signed({22'd0, rtc_ms}) + slew_correction;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rtc_seconds        <= 32'd0;
            rtc_ms             <= 10'd0;
            rtc_uptime_ms      <= 32'd0;
            rtc_locked         <= 1'b0;
            sync_age_ms        <= 32'd0;
            sync_count         <= 32'd0;
            step_count         <= 32'd0;
            slew_count         <= 32'd0;
            lost_threshold_ms  <= DEFAULT_LOST_MS;
        end else begin
            if (cfg_we && cfg_addr == 16'd0)
                lost_threshold_ms <= cfg_wdata;

            // ---- 自由运行计数：每ms递增一次 ----
            if (ms_tick) begin
                rtc_uptime_ms <= rtc_uptime_ms + 32'd1;

                if (rtc_ms == 10'd999) begin
                    rtc_ms      <= 10'd0;
                    rtc_seconds <= rtc_seconds + 32'd1;
                end else begin
                    rtc_ms <= rtc_ms + 10'd1;
                end

                if (sync_age_ms < 32'hFFFF_FFFF)
                    sync_age_ms <= sync_age_ms + 32'd1;
                if (sync_age_ms + 32'd1 >= lost_threshold_ms)
                    rtc_locked <= 1'b0;
            end

            // ---- 网络时间基准修正：优先于同一拍的自由计数结果 ----
            if (sync_pending) begin
                sync_count  <= sync_count + 32'd1;
                sync_age_ms <= 32'd0;
                rtc_locked  <= 1'b1;

                if (diff_is_large) begin
                    rtc_seconds <= sync_seconds;
                    rtc_ms      <= sync_ms;
                    step_count  <= step_count + 32'd1;
                end else if (slew_correction != 32'sd0) begin
                    slew_count <= slew_count + 32'd1;
                    if (new_ms_signed >= 32'sd1000) begin
                        rtc_ms      <= new_ms_signed[9:0] - 10'd1000;
                        rtc_seconds <= rtc_seconds + 32'd1;
                    end else if (new_ms_signed < 32'sd0) begin
                        rtc_ms      <= new_ms_signed[9:0] + 10'd1000;
                        rtc_seconds <= rtc_seconds - 32'd1;
                    end else begin
                        rtc_ms <= new_ms_signed[9:0];
                    end
                end
            end
        end
    end

endmodule
