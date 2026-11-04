//=====================================================================
// 模块名 : afdx_token_bucket_policer
// 功能   : AFDX接收侧按VL的Token Bucket流量监管(统计型入口监管单元)
//          - 每VL独立token余额，单位为字节
//          - 1ms时间基准，补充周期为1/2/4/8/16/32/64/128ms
//          - 一帧消耗frame_len个token，不足则判为超限并计数
//          - 采用"惰性补充"：只在该VL来帧时按流逝时间一次性补充，
//            因此不需要对4096条VL逐拍扫描
//
// 实现   : 每VL配置/状态/丢弃计数存放在"只带时钟、无复位"的同步读
//          存储器；复位后先用NUM_RX_VL拍扫成默认值(init_done拉高前
//          不处理帧事件、不接受配置写)。
//          帧事件2拍处理: 周期n frame_valid(发读) -> 周期n+1(计算写回)
//          -> 周期n+2 frame_allow/frame_policed。
//
// 配置总线 cfg_addr[15:12]: 0=bucket容量  1=refill token数
//                           2=refill周期选择(0..7)  3=使能(bit0)
// MIB读口 : mib_re后在空闲时读取，mib_rvalid脉冲时mib_drop_count有效。
//=====================================================================
module afdx_token_bucket_policer #(
    parameter NUM_RX_VL   = 4096,
    parameter RX_IDX_W    = 12,
    parameter CLK_FREQ_HZ = 125_000_000
)(
    input  wire                  clk,
    input  wire                  rst_n,

    output reg                   ms_tick,
    output reg  [31:0]           time_ms,

    input  wire                  frame_valid,
    input  wire [RX_IDX_W-1:0]   frame_vlid,
    input  wire [10:0]           frame_len,
    output reg                   frame_allow,
    output reg                   frame_policed,

    input  wire                  cfg_we,
    input  wire [15:0]           cfg_addr,
    input  wire [31:0]           cfg_wdata,

    input  wire                  mib_re,
    input  wire [RX_IDX_W-1:0]   mib_vlid,
    output reg                   mib_rvalid,
    output reg  [31:0]           mib_drop_count,

    output reg                   init_done
);

    localparam integer CYCLES_PER_MS = CLK_FREQ_HZ / 1000;
    localparam [31:0]  DEF_BYTES     = 32'd1518;

    // ---------------- 1ms时间基准 ----------------
    reg [31:0] clk_ms_counter;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_ms_counter <= 32'd0;
            time_ms        <= 32'd0;
            ms_tick        <= 1'b0;
        end else begin
            ms_tick <= 1'b0;
            if (clk_ms_counter == CYCLES_PER_MS-1) begin
                clk_ms_counter <= 32'd0;
                time_ms        <= time_ms + 32'd1;
                ms_tick        <= 1'b1;
            end else begin
                clk_ms_counter <= clk_ms_counter + 32'd1;
            end
        end
    end

    // ---------------- 每VL存储器(无复位) ----------------
    reg [31:0] cap_ram  [0:NUM_RX_VL-1];
    reg [31:0] ref_ram  [0:NUM_RX_VL-1];
    reg [2:0]  sel_ram  [0:NUM_RX_VL-1];
    reg        en_ram   [0:NUM_RX_VL-1];
    reg [31:0] bal_ram  [0:NUM_RX_VL-1];
    reg [31:0] tms_ram  [0:NUM_RX_VL-1];
    reg [31:0] drop_ram [0:NUM_RX_VL-1];
    reg [31:0] cap_q, ref_q, bal_q, tms_q, drop_q;
    reg [2:0]  sel_q;
    reg        en_q;

    reg [RX_IDX_W:0] init_cnt;
    wire init_busy = ~init_done;

    localparam [1:0] S_IDLE = 2'd0, S_EXEC = 2'd1, S_MIB = 2'd2;
    reg [1:0]          state;
    reg [RX_IDX_W-1:0] cur_vlid;
    reg [10:0]         cur_len;
    reg                mib_pend;
    reg [RX_IDX_W-1:0] mib_vl;
    // 忙碌(EXEC/MIB)期间到达的帧事件暂存一级(帧间隔>=64拍，一级足够)
    reg                frm_pend;
    reg [RX_IDX_W-1:0] pend_vlid;
    reg [10:0]         pend_len;

    wire                sel_pend = frm_pend;
    wire                sel_live = ~frm_pend & frame_valid;
    wire [RX_IDX_W-1:0] raddr = sel_pend ? pend_vlid : (sel_live ? frame_vlid : mib_vl);

    // ---------------- EXEC组合计算 ----------------
    wire [31:0] elapsed  = time_ms - tms_q;
    wire [31:0] quanta   = elapsed >> sel_q;                  // 周期为2^sel ms
    wire [63:0] add64    = quanta * ref_q;
    wire [63:0] sum64    = {32'd0, bal_q} + add64;
    wire [31:0] nbal_cap = (sum64 > {32'd0, cap_q}) ? cap_q : sum64[31:0];
    wire        enough   = (nbal_cap >= {21'd0, cur_len});
    wire [31:0] tms_new  = tms_q + (quanta << sel_q);

    reg         wr_state, wr_drop;
    reg  [31:0] n_bal, n_tms, n_drop;
    always @(*) begin
        wr_state = 1'b0; wr_drop = 1'b0;
        n_bal = bal_q; n_tms = tms_q; n_drop = drop_q;
        if (en_q) begin
            wr_state = 1'b1;
            if (quanta != 32'd0) n_tms = tms_new;
            if (enough) n_bal = nbal_cap - {21'd0, cur_len};
            else begin
                n_bal  = nbal_cap;
                n_drop = drop_q + 32'd1;
                wr_drop = 1'b1;
            end
        end
    end

    // ---------------- 写口 ----------------
    wire                cfg_hit  = cfg_we & init_done;
    wire [RX_IDX_W-1:0] cfg_idx  = cfg_addr[RX_IDX_W-1:0];
    wire [RX_IDX_W-1:0] init_idx = init_cnt[RX_IDX_W-1:0];
    wire                ex       = (state == S_EXEC);

    always @(posedge clk) begin
        if (init_busy) begin
            cap_ram[init_idx]  <= DEF_BYTES;
            ref_ram[init_idx]  <= DEF_BYTES;
            sel_ram[init_idx]  <= 3'd0;
            en_ram[init_idx]   <= 1'b1;
            bal_ram[init_idx]  <= DEF_BYTES;
            tms_ram[init_idx]  <= 32'd0;
            drop_ram[init_idx] <= 32'd0;
        end else begin
            if (cfg_hit && cfg_addr[15:12] == 4'd0) cap_ram[cfg_idx] <= cfg_wdata;
            if (cfg_hit && cfg_addr[15:12] == 4'd1) ref_ram[cfg_idx] <= cfg_wdata;
            if (cfg_hit && cfg_addr[15:12] == 4'd2) sel_ram[cfg_idx] <= cfg_wdata[2:0];
            if (cfg_hit && cfg_addr[15:12] == 4'd3) en_ram[cfg_idx]  <= cfg_wdata[0];
            if (ex && wr_state) begin
                bal_ram[cur_vlid] <= n_bal;
                tms_ram[cur_vlid] <= n_tms;
            end
            if (ex && wr_drop) drop_ram[cur_vlid] <= n_drop;
        end
        cap_q  <= cap_ram[raddr];
        ref_q  <= ref_ram[raddr];
        sel_q  <= sel_ram[raddr];
        en_q   <= en_ram[raddr];
        bal_q  <= bal_ram[raddr];
        tms_q  <= tms_ram[raddr];
        drop_q <= drop_ram[raddr];
    end

    // ---------------- 控制 ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            init_cnt <= {(RX_IDX_W+1){1'b0}}; init_done <= 1'b0;
            state <= S_IDLE; cur_vlid <= {RX_IDX_W{1'b0}}; cur_len <= 11'd0;
            frame_allow <= 1'b0; frame_policed <= 1'b0;
            mib_pend <= 1'b0; mib_vl <= {RX_IDX_W{1'b0}};
            frm_pend <= 1'b0; pend_vlid <= {RX_IDX_W{1'b0}}; pend_len <= 11'd0;
            mib_rvalid <= 1'b0; mib_drop_count <= 32'd0;
        end else begin
            frame_allow <= 1'b0; frame_policed <= 1'b0; mib_rvalid <= 1'b0;

            if (init_busy) begin
                init_cnt <= init_cnt + 1'b1;
                if (init_cnt == NUM_RX_VL-1) init_done <= 1'b1;
            end

            if (mib_re && init_done && !mib_pend) begin
                mib_pend <= 1'b1;
                mib_vl   <= mib_vlid;
            end

            if (state != S_IDLE && frame_valid && init_done) begin
                frm_pend <= 1'b1; pend_vlid <= frame_vlid; pend_len <= frame_len;
            end

            case (state)
                S_IDLE: begin
                    if (init_done && (frm_pend || frame_valid)) begin
                        cur_vlid <= sel_pend ? pend_vlid : frame_vlid;
                        cur_len  <= sel_pend ? pend_len  : frame_len;
                        frm_pend <= 1'b0;
                        state    <= S_EXEC;
                        // 暂存的事件被服务时，同拍新到的实时事件需再暂存
                        if (sel_pend && frame_valid) begin
                            frm_pend <= 1'b1; pend_vlid <= frame_vlid; pend_len <= frame_len;
                        end
                    end else if (mib_pend && init_done) begin
                        state <= S_MIB;
                    end
                end
                S_EXEC: begin
                    // 未使能的VL不受限，直接放行
                    if (!en_q)        frame_allow   <= 1'b1;
                    else if (enough)  frame_allow   <= 1'b1;
                    else              frame_policed <= 1'b1;
                    state <= S_IDLE;
                end
                S_MIB: begin
                    mib_drop_count <= drop_q;
                    mib_rvalid     <= 1'b1;
                    mib_pend       <= 1'b0;
                    state          <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    // synthesis translate_off
    initial begin cap_q = 0; ref_q = 0; bal_q = 0; tms_q = 0; drop_q = 0; sel_q = 0; en_q = 0; end
    // synthesis translate_on
endmodule
