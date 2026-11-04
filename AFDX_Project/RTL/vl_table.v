//=====================================================================
// 模块名 : vl_table
// 功能   : AFDX端系统 VL(虚拟链路) 配置表存储与查表
//          - TX表: 最多NUM_TX_VL条VL, 每条含 {bag_sel, valid, lmax, dest_mac}
//          - RX表: 最多NUM_RX_VL条VL, 每条含 {lmin, lmax, valid}
//          - 长度字段统一采用"以太网线速帧长"口径(含FCS，64..1518)；
//            MAC数据流不含FCS，所以接收端用"MAC数据长度+4"与表项比较。
//
// 实现要点(可综合性)：
//   * TX/RX表都是"只带时钟、无复位"的同步读存储器，可映射为块RAM；
//   * 复位后由内部扫描状态机把全部表项清零(init_done拉高前
//     NUM_RX_VL个周期)，期间忽略配置写、所有查表一律返回"无效表项"；
//   * 配置总线读回与查表共用读口，查表优先，读回被推迟到读口空闲。
//
// 接口时序:
//   查表:  *_lookup_valid(周期n) -> *_lookup_rdy及结果(周期n+1，仅此拍有效)
//   写配置: reg_we(周期n) -> reg_rdy(周期n+1)
//   读配置: reg_re(周期n) -> reg_rdy+reg_rdata(1~2拍后)
//   reg_addr[15]=1 访问RX表(索引=reg_addr[RX_IDX_W-1:0])；
//   reg_addr[15]=0 访问TX表(索引=reg_addr[TX_IDX_W:1]，reg_addr[0]选
//   低/高32位)。
//
// TX表项:  lo[2:0]=BAG选择(1..128ms)  lo[3]=valid  lo[14:4]=LMAX
//          lo[31:15]+hi[30:0]=目的MAC(48位)
// RX表项:  [10:0]=LMIN  [21:11]=LMAX  [22]=valid
//=====================================================================
module vl_table #(
    parameter NUM_TX_VL = 128,
    parameter NUM_RX_VL = 4096,
    parameter TX_IDX_W  = 7,
    parameter RX_IDX_W  = 12
)(
    input  wire         clk,
    input  wire         rst_n,

    // ---- 配置/读出总线 ----
    input  wire [15:0]  reg_addr,
    input  wire [31:0]  reg_wdata,
    input  wire         reg_we,
    input  wire         reg_re,
    output reg  [31:0]  reg_rdata,
    output reg          reg_rdy,

    // ---- 发送侧查表 ----
    input  wire                  tx_lookup_valid,
    input  wire [TX_IDX_W-1:0]   tx_lookup_vlid,
    output reg                   tx_lookup_rdy,
    output wire [2:0]            tx_bag_sel,
    output wire [10:0]           tx_max_len,
    output wire [47:0]           tx_dest_mac,
    output wire                  tx_entry_valid,

    // ---- 接收侧查表 ----
    input  wire                  rx_lookup_valid,
    input  wire [RX_IDX_W-1:0]   rx_lookup_vlid,
    output reg                   rx_lookup_rdy,
    output wire [10:0]           rx_min_len,
    output wire [10:0]           rx_max_len,
    output wire                  rx_entry_valid,

    // ---- 复位后表清零完成标志 ----
    output reg                   init_done
);

    localparam integer INIT_MAX = (NUM_RX_VL > NUM_TX_VL) ? NUM_RX_VL : NUM_TX_VL;
    localparam integer RD_W     = (RX_IDX_W > TX_IDX_W) ? RX_IDX_W : TX_IDX_W;

    // 综合属性：要求映射为块RAM，不要因为深度小(128)被展开成寄存器
    (* ramstyle = "M9K", syn_ramstyle = "block_ram" *) reg [31:0] tx_lo  [0:4095];   // 深度补到4096(见afdx_fwft_fifo说明)
    (* ramstyle = "M9K", syn_ramstyle = "block_ram" *) reg [31:0] tx_hi  [0:4095];
    (* ramstyle = "M9K", syn_ramstyle = "block_ram" *) reg [31:0] rx_mem [0:NUM_RX_VL-1];
    reg [31:0] tx_lo_q, tx_hi_q, rx_q;

    reg [RD_W:0] init_cnt;
    wire init_busy = ~init_done;

    wire                 addr_is_rx = reg_addr[15];
    wire [TX_IDX_W-1:0]  tx_idx     = reg_addr[TX_IDX_W:1];
    wire                 tx_word_hi = reg_addr[0];
    wire [RX_IDX_W-1:0]  rx_idx     = reg_addr[RX_IDX_W-1:0];

    // ---------------- 配置读回请求(与查表共用读口) ----------------
    reg                  rd_pend;
    reg                  rd_is_rx;
    reg                  rd_hi;
    reg [RD_W-1:0]       rd_idx;
    reg                  rd_inflight;

    wire rd_blocked = rd_is_rx ? rx_lookup_valid : tx_lookup_valid;
    wire rd_grant   = rd_pend & init_done & ~rd_blocked;
    wire rd_grant_tx = rd_grant & ~rd_is_rx;
    wire rd_grant_rx = rd_grant &  rd_is_rx;

    // ---------------- 写口(初始化扫描 / 配置写) ----------------
    wire cfg_wr   = reg_we & init_done;
    wire tx_we_lo = init_busy ? (init_cnt < NUM_TX_VL) : (cfg_wr & ~addr_is_rx & ~tx_word_hi);
    wire tx_we_hi = init_busy ? (init_cnt < NUM_TX_VL) : (cfg_wr & ~addr_is_rx &  tx_word_hi);
    wire rx_we    = init_busy ? (init_cnt < NUM_RX_VL) : (cfg_wr &  addr_is_rx);
    wire [TX_IDX_W-1:0] tx_waddr = init_busy ? init_cnt[TX_IDX_W-1:0] : tx_idx;
    wire [RX_IDX_W-1:0] rx_waddr = init_busy ? init_cnt[RX_IDX_W-1:0] : rx_idx;
    wire [31:0]         wdata    = init_busy ? 32'd0 : reg_wdata;

    // ---------------- 读口 ----------------
    wire                tx_rd_en = tx_lookup_valid | rd_grant_tx;
    wire [TX_IDX_W-1:0] tx_raddr = tx_lookup_valid ? tx_lookup_vlid : rd_idx[TX_IDX_W-1:0];
    wire                rx_rd_en = rx_lookup_valid | rd_grant_rx;
    wire [RX_IDX_W-1:0] rx_raddr = rx_lookup_valid ? rx_lookup_vlid : rd_idx[RX_IDX_W-1:0];

    // 存储器：只带时钟、无复位，便于综合器推断块RAM
    always @(posedge clk) begin
        if (tx_we_lo) tx_lo[tx_waddr]  <= wdata;
        if (tx_we_hi) tx_hi[tx_waddr]  <= wdata;
        if (rx_we)    rx_mem[rx_waddr] <= wdata;
        if (tx_rd_en) begin
            tx_lo_q <= tx_lo[tx_raddr];
            tx_hi_q <= tx_hi[tx_raddr];
        end
        if (rx_rd_en) rx_q <= rx_mem[rx_raddr];
    end

    // synthesis translate_off
    initial begin tx_lo_q = 32'd0; tx_hi_q = 32'd0; rx_q = 32'd0; end
    // synthesis translate_on

    // 查表结果：RAM输出寄存器直接译码，与*_lookup_rdy同拍有效
    assign tx_bag_sel     = tx_lo_q[2:0];
    assign tx_entry_valid = tx_lo_q[3] & init_done;
    assign tx_max_len     = tx_lo_q[14:4];
    assign tx_dest_mac    = {tx_hi_q[30:0], tx_lo_q[31:15]};
    assign rx_min_len     = rx_q[10:0];
    assign rx_max_len     = rx_q[21:11];
    assign rx_entry_valid = rx_q[22] & init_done;

    // ---------------- 控制逻辑 ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            init_cnt      <= {(RD_W+1){1'b0}};
            init_done     <= 1'b0;
            tx_lookup_rdy <= 1'b0;
            rx_lookup_rdy <= 1'b0;
            reg_rdy       <= 1'b0;
            reg_rdata     <= 32'd0;
            rd_pend       <= 1'b0;
            rd_is_rx      <= 1'b0;
            rd_hi         <= 1'b0;
            rd_idx        <= {RD_W{1'b0}};
            rd_inflight   <= 1'b0;
        end else begin
            tx_lookup_rdy <= tx_lookup_valid;
            rx_lookup_rdy <= rx_lookup_valid;
            reg_rdy       <= 1'b0;

            if (!init_done) begin
                init_cnt <= init_cnt + 1'b1;
                if (init_cnt == INIT_MAX-1) init_done <= 1'b1;
            end

            // 写配置：下一拍应答
            if (cfg_wr) reg_rdy <= 1'b1;

            // 读配置：锁存请求 -> 读口空闲时发起 -> 下一拍取数
            if (reg_re && init_done && !rd_pend) begin
                rd_pend  <= 1'b1;
                rd_is_rx <= addr_is_rx;
                rd_hi    <= tx_word_hi;
                rd_idx   <= addr_is_rx ? {{RD_W{1'b0}}, rx_idx} : {{RD_W{1'b0}}, tx_idx};
            end
            if (rd_grant) begin
                rd_pend     <= 1'b0;
                rd_inflight <= 1'b1;
            end
            if (rd_inflight) begin
                rd_inflight <= 1'b0;
                reg_rdy     <= 1'b1;
                reg_rdata   <= rd_is_rx ? rx_q : (rd_hi ? tx_hi_q : tx_lo_q);
            end
        end
    end
endmodule
