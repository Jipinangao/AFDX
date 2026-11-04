//=====================================================================
// 模块名 : afdx_bag_jitter_scheduler
// 功能   : AFDX发送侧 BAG + Jitter regulator 
//
// 规则:
//   1) BAG取值 2^k ms (k=0..7)，同一VL两次授权的间隔不得小于其BAG。
//   2) 每条VL记录"最近一次授权的时刻"(32位自由计数器时间戳)。请求到来时
//      查该VL的BAG(vl_table)和上次授权时刻，已满BAG(或从未授权过)就授权，
//      否则休眠到BAG结束再重查，期间不占用vl_table查表口。
//   3) Jitter：请求"已具备授权条件"(BAG已到或无需等待)之后到真正授权
//      之间的等待周期数，超过jitter_max_us(由afdx_jitter_calculator按
//      XAPP1130 Eq.2动态算出)就置位jitter_violation(粘滞标志)。
//
// 为什么只处理一个请求：发送路径只有一个消息缓冲(ip_frag)，同一时刻至多
//   一条VL有数据待发，因此不需要对128条VL并行维护计数器，只需对"当前
//   请求的那条VL"做时间戳比较。每VL状态放同步RAM，复位后用NUM_TX_VL拍
//   扫成"从未授权"。
// 时间戳为32位，125MHz下约34秒回绕：一条VL空闲超过34秒后，时间戳可能
//   让它多等一个BAG(<=128ms)才被授权，但绝不会让BAG被缩短。
//=====================================================================
module afdx_bag_jitter_scheduler #(
    parameter NUM_TX_VL   = 128,
    parameter TX_IDX_W    = 7,
    parameter CLK_FREQ_HZ = 125_000_000
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // ---- 发送请求：消息缓冲里有一条等待发送的消息，属于req_vlid ----
    input  wire                   req_valid,
    input  wire [TX_IDX_W-1:0]    req_vlid,

    // ---- vl_table TX lookup ----
    output reg                    tx_lookup_valid,
    output reg  [TX_IDX_W-1:0]    tx_lookup_vlid,
    input  wire                   tx_lookup_rdy,
    input  wire [2:0]             tx_bag_sel,
    input  wire [10:0]            tx_max_len,
    input  wire [47:0]            tx_dest_mac,
    input  wire                   tx_entry_valid,

    // ---- frame_encap grant ----
    output reg                    grant_valid,
    output reg  [TX_IDX_W-1:0]    grant_vlid,
    output reg  [47:0]            grant_dest_mac,
    output reg  [10:0]            grant_max_len,
    input  wire                   grant_ready,

    // ---- 由afdx_jitter_calculator后台扫描vl_table算出 ----
    input  wire [9:0]             jitter_max_us,

    output reg                    jitter_violation
);
    localparam integer CYCLES_PER_MS = CLK_FREQ_HZ / 1000;
    localparam integer CLK_MHZ       = (CLK_FREQ_HZ / 1000000);

    wire [31:0] jmax_raw    = {22'd0, jitter_max_us} * ((CLK_MHZ > 0) ? CLK_MHZ : 1);
    wire [31:0] jmax_cycles = (jmax_raw < 32'd1) ? 32'd1 : jmax_raw;

    localparam [2:0] S_INIT = 3'd0, S_IDLE = 3'd1, S_LOOKUP = 3'd2, S_GRANT = 3'd3, S_WAIT = 3'd4;
    reg [2:0]            state;
    reg [TX_IDX_W:0]     init_cnt;
    reg [31:0]           cyc;                 // 自由计数器
    reg [TX_IDX_W-1:0]   cand;
    reg [31:0]           wait_left;
    reg [31:0]           age;                 // 已具备授权条件后的等待周期数

    // 每VL "最近授权时刻" 与 "是否授权过"：无复位同步RAM
    (* ramstyle = "M9K", syn_ramstyle = "block_ram" *) reg [31:0] lt_ram [0:4095];   // 深度补到4096(综合工具只对深度>=4096推断块RAM)
    (* ramstyle = "M9K", syn_ramstyle = "block_ram" *) reg        lv_ram [0:4095];
    reg [31:0] lt_q;
    reg        lv_q;

    wire [TX_IDX_W-1:0] raddr = (state == S_IDLE) ? req_vlid : cand;
    wire                wr_init = (state == S_INIT);
    wire                wr_grant = (state == S_GRANT) && grant_ready;

    always @(posedge clk) begin
        if (wr_init) begin
            lv_ram[init_cnt[TX_IDX_W-1:0]] <= 1'b0;
            lt_ram[init_cnt[TX_IDX_W-1:0]] <= 32'd0;
        end else if (wr_grant) begin
            lv_ram[cand] <= 1'b1;
            lt_ram[cand] <= cyc;
        end
        lt_q <= lt_ram[raddr];
        lv_q <= lv_ram[raddr];
    end

    // synthesis translate_off
    initial begin lt_q = 32'd0; lv_q = 1'b0; end
    // synthesis translate_on

    wire [31:0] bag_cycles = CYCLES_PER_MS << tx_bag_sel;
    wire [31:0] elapsed    = cyc - lt_q;
    wire        bag_ok     = !lv_q || (elapsed >= bag_cycles);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_INIT; init_cnt <= {(TX_IDX_W+1){1'b0}}; cyc <= 32'd0;
            cand <= {TX_IDX_W{1'b0}}; wait_left <= 32'd0; age <= 32'd0;
            tx_lookup_valid <= 1'b0; tx_lookup_vlid <= {TX_IDX_W{1'b0}};
            grant_valid <= 1'b0; grant_vlid <= {TX_IDX_W{1'b0}};
            grant_dest_mac <= 48'd0; grant_max_len <= 11'd0;
            jitter_violation <= 1'b0;
        end else begin
            cyc <= cyc + 32'd1;
            tx_lookup_valid <= 1'b0;

            // Jitter窗口年龄：只在"可授权但尚未授权"的状态里累计
            if (req_valid && (state == S_IDLE || state == S_LOOKUP || state == S_GRANT)) begin
                if (age != 32'hFFFF_FFFF) age <= age + 32'd1;
                if (age > jmax_cycles) jitter_violation <= 1'b1;
            end else begin
                age <= 32'd0;
            end

            case (state)
                S_INIT: begin
                    init_cnt <= init_cnt + 1'b1;
                    if (init_cnt == NUM_TX_VL-1) state <= S_IDLE;
                end

                S_IDLE: if (req_valid) begin
                    cand            <= req_vlid;
                    tx_lookup_vlid  <= req_vlid;
                    tx_lookup_valid <= 1'b1;
                    state           <= S_LOOKUP;
                end

                S_LOOKUP: begin
                    if (!req_valid) state <= S_IDLE;
                    else if (tx_lookup_rdy) begin
                        if (!tx_entry_valid) begin
                            state <= S_IDLE;             // VL未配置：重试，age继续增长
                        end else if (bag_ok) begin
                            grant_valid    <= 1'b1;
                            grant_vlid     <= cand;
                            grant_dest_mac <= tx_dest_mac;
                            grant_max_len  <= tx_max_len;
                            state          <= S_GRANT;
                        end else begin
                            wait_left <= bag_cycles - elapsed;
                            age       <= 32'd0;
                            state     <= S_WAIT;         // BAG未到：休眠
                        end
                    end
                end

                S_GRANT: if (grant_ready) begin
                    grant_valid <= 1'b0;
                    age         <= 32'd0;
                    state       <= S_IDLE;
                end

                S_WAIT: begin
                    // 请求消失或换成别的VL就放弃本次等待，让新请求立即得到服务
                    if (!req_valid || (req_vlid != cand) || (wait_left <= 32'd1)) state <= S_IDLE;
                    else wait_left <= wait_left - 32'd1;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
