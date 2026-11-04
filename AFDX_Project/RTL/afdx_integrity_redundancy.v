//=====================================================================
// 模块名 : afdx_integrity_redundancy
//
// 接收路径抽象: A/B两路帧摘要 -> 事件FIFO -> 完整性检查(Integrity Checker)
//              -> 冗余管理(Redundancy Manager)。所有统计均由实际接收到的
//              帧事件产生。
//
// Integrity Checker (每个网络独立维护该VL的前一个序号PSN)：
//   * 帧本身有错(MAC错/长度越界等)  -> 丢弃并报错，PSN仍按该帧SN更新；
//   * 复位后该VL该网络的第一帧      -> 接受，任意SN可建立PSN；
//   * SN=0                          -> 始终接受(发送端复位)，下一期望SN=1；
//   * 其余: SN必须等于 PSN+1 或 PSN+2(回绕 255->1)，允许单帧丢失；
//   * 检查失败则丢帧，并把该帧SN写入PSN，使后续流可恢复；
//   * integrity_enable 按VL配置，关闭时跳过SN检查(帧错误仍丢弃)。
//
// Redundancy Manager：
//   * 与最近放行帧同VL、同SN、来自另一网络、且距上次放行不超过
//     SkewMAX -> 冗余副本，丢弃并计数；
//   * 同SN但已超过SkewMAX -> 视为新帧放行；
//   * SN相对最近放行帧"向前"(1..127) -> 放行并更新；其余 -> 丢弃报错。
//
// 实现方式(可综合、支持4096条RX VL)：
//   每VL状态存放在三组"只带时钟、无复位"的同步读存储器里
//     cf_*_ram : 配置(integrity_enable, SkewMAX周期数)
//     st_ram   : 序号状态(A/B期望SN、冗余管理最近放行帧、放行时间戳)
//     cn_ram   : 4个统计计数器
//   不再为每条VL保留一个自增计时器；SkewMAX用"放行时刻的自由计数器
//   时间戳"比较实现: elapsed = cyc - rm_ts。时间戳为32位，
//   在125MHz下约34秒回绕。
//   事件逐个处理，每个事件2拍: S_IDLE(出队+发读) -> S_EXEC(计算+写回)。
//   复位后先用NUM_RX_VL拍把三组存储器扫成默认值，期间(init_done=0)
//   不处理事件、不接受配置写。
//
// MIB读口：mib_re后在事件空闲时读存储器，mib_rvalid脉冲时mib_rdata有效。
//
// cfg_addr[15:12]: 0=integrity enable(bit0)  1=SkewMAX(时钟周期数)
// mib_addr[15:12]: 0=rx_frames 1=rx_errors 2=duplicates 3=seq_errors
//                  4=integrity_last_seq
//=====================================================================
module afdx_integrity_redundancy #(
    parameter NUM_RX_VL           = 4096,
    parameter RX_IDX_W            = 12,
    parameter FIFO_DEPTH          = 8,
    parameter [31:0] DEFAULT_SKEW_CYCLES = 32'd625   // 默认5us@125MHz(<=5ms上限)
)(
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire                  a_valid,
    input  wire [RX_IDX_W-1:0]   a_vlid,
    input  wire [7:0]            a_seq_num,
    input  wire [10:0]           a_frame_len,
    input  wire                  a_error,

    input  wire                  b_valid,
    input  wire [RX_IDX_W-1:0]   b_vlid,
    input  wire [7:0]            b_seq_num,
    input  wire [10:0]           b_frame_len,
    input  wire                  b_error,

    input  wire                  cfg_we,
    input  wire [15:0]           cfg_addr,
    input  wire [31:0]           cfg_wdata,

    output reg                   out_valid,
    output reg  [RX_IDX_W-1:0]   out_vlid,
    output reg  [7:0]            out_seq_num,
    output reg  [10:0]           out_frame_len,
    output reg                   out_network,

    output reg                   out_error,
    output reg                   sequence_error,
    output reg                   duplicate_drop,

    // 每个事件处理完毕(无论放行/冗余丢弃/校验失败)都脉冲一拍，
    // 供下游载荷交付模块确定性地释放或丢弃对应缓冲。
    output reg                   event_valid,
    output reg  [RX_IDX_W-1:0]   event_vlid,
    output reg  [7:0]            event_seq_num,
    output reg                   event_network,

    input  wire                  mib_re,
    input  wire [15:0]           mib_addr,
    output reg                   mib_rvalid,
    output reg  [31:0]           mib_rdata,

    output reg                   init_done
);

    // ------------------------------------------------------------
    // 事件FIFO
    // ------------------------------------------------------------
    localparam integer FIFO_W = 1 + RX_IDX_W + 8 + 11 + 1;  // {net,vlid,seq,len,err}
    localparam integer PTR_W  = 4;                          // FIFO_DEPTH <= 16

    reg [FIFO_W-1:0] event_fifo [0:FIFO_DEPTH-1];
    reg [PTR_W-1:0]  fifo_count, fifo_wr_ptr, fifo_rd_ptr;

    function [PTR_W-1:0] ptr_inc;
        input [PTR_W-1:0] ptr;
        begin
            if (ptr == FIFO_DEPTH-1) ptr_inc = {PTR_W{1'b0}};
            else                     ptr_inc = ptr + 1'b1;
        end
    endfunction

    function [7:0] next_seq;      // 255->1, 0->1, 其余+1
        input [7:0] s;
        begin
            if ((s == 8'd0) || (s == 8'd255)) next_seq = 8'd1;
            else next_seq = s + 8'd1;
        end
    endfunction

    function [8:0] seq_forward_distance; // new相对base向前的距离(0..254)
        input [7:0] base_seq;
        input [7:0] new_seq;
        begin
            if (new_seq >= base_seq) seq_forward_distance = {1'b0, new_seq} - {1'b0, base_seq};
            else                     seq_forward_distance = {1'b0, new_seq} + 9'd255 - {1'b0, base_seq};
        end
    endfunction

    wire [FIFO_W-1:0] fifo_head = event_fifo[fifo_rd_ptr];
    wire [RX_IDX_W-1:0] head_vlid = fifo_head[FIFO_W-2 -: RX_IDX_W];

    reg [2:0] state;
    localparam [2:0] S_IDLE = 3'd0, S_EXEC = 3'd1, S_MIB = 3'd2;

    wire pop_do  = (state == S_IDLE) && init_done && (fifo_count != 0);
    wire push_a  = a_valid && (fifo_count < FIFO_DEPTH);
    wire push_b  = b_valid && ((fifo_count + push_a) < FIFO_DEPTH);
    wire [PTR_W-1:0] wr_ptr_a = push_a ? ptr_inc(fifo_wr_ptr) : fifo_wr_ptr;
    wire [PTR_W-1:0] wr_ptr_b = push_b ? ptr_inc(wr_ptr_a)    : wr_ptr_a;

    always @(posedge clk) begin   // FIFO存储体: 无复位
        if (push_a) event_fifo[fifo_wr_ptr] <= {1'b0, a_vlid, a_seq_num, a_frame_len, a_error};
        if (push_b) event_fifo[wr_ptr_a]    <= {1'b1, b_vlid, b_seq_num, b_frame_len, b_error};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_count  <= {PTR_W{1'b0}};
            fifo_wr_ptr <= {PTR_W{1'b0}};
            fifo_rd_ptr <= {PTR_W{1'b0}};
        end else begin
            fifo_wr_ptr <= wr_ptr_b;
            if (pop_do) fifo_rd_ptr <= ptr_inc(fifo_rd_ptr);
            fifo_count  <= fifo_count + push_a + push_b - pop_do;
        end
    end

    // ------------------------------------------------------------
    // 当前处理事件 / MIB请求
    // ------------------------------------------------------------
    reg                cur_net, cur_error;
    reg [RX_IDX_W-1:0] cur_vlid;
    reg [7:0]          cur_seq;
    reg [10:0]         cur_len;

    reg                  mib_pend;
    reg [3:0]            mib_obj;
    reg [RX_IDX_W-1:0]   mib_vl;

    reg [31:0] cyc;                 // 自由计数器(时间戳基准)
    always @(posedge clk or negedge rst_n)
        if (!rst_n) cyc <= 32'd0; else cyc <= cyc + 32'd1;

    // ------------------------------------------------------------
    // 每VL存储器
    // ------------------------------------------------------------
    // 状态字: [7:0]expA [8]initA [16:9]expB [17]initB [25:18]intLastSeq
    //         [26]rmInit [34:27]rmLastSeq [35]rmLastNet [67:36]rmTs
    localparam integer ST_W = 68;
    localparam [ST_W-1:0] ST_DEFAULT = {32'd0, 1'b0, 8'd0, 1'b0, 8'd0, 1'b0, 8'd1, 1'b0, 8'd1};
    // 计数字: {seq_errors, duplicates, rx_errors, rx_frames}
    localparam integer CN_W = 128;

    reg [ST_W-1:0] st_ram [0:NUM_RX_VL-1];
    reg [CN_W-1:0] cn_ram [0:NUM_RX_VL-1];
    reg            cf_en_ram   [0:NUM_RX_VL-1];
    reg [31:0]     cf_skew_ram [0:NUM_RX_VL-1];
    reg [ST_W-1:0] st_q;
    reg [CN_W-1:0] cn_q;
    reg            cf_en_q;
    reg [31:0]     cf_skew_q;

    reg [RX_IDX_W:0] init_cnt;
    wire init_busy = ~init_done;

    // 读地址: S_IDLE拍发出(出队事件的VL，否则MIB请求的VL)
    wire [RX_IDX_W-1:0] raddr = (fifo_count != 0) ? head_vlid : mib_vl;

    // 写口
    reg                  exec_wr;
    reg  [ST_W-1:0]      n_st;
    reg  [CN_W-1:0]      n_cn;
    wire [RX_IDX_W-1:0]  waddr   = init_busy ? init_cnt[RX_IDX_W-1:0] : cur_vlid;
    wire                 wr_en   = init_busy | exec_wr;
    wire [ST_W-1:0]      st_wd   = init_busy ? ST_DEFAULT : n_st;
    wire [CN_W-1:0]      cn_wd   = init_busy ? {CN_W{1'b0}} : n_cn;

    wire                 cfg_hit = cfg_we & init_done;
    wire                 cf_en_we   = init_busy | (cfg_hit & (cfg_addr[15:12] == 4'd0));
    wire                 cf_skew_we = init_busy | (cfg_hit & (cfg_addr[15:12] == 4'd1));
    wire [RX_IDX_W-1:0]  cf_waddr   = init_busy ? init_cnt[RX_IDX_W-1:0] : cfg_addr[RX_IDX_W-1:0];
    wire                 cf_en_wd   = init_busy ? 1'b1 : cfg_wdata[0];
    wire [31:0]          cf_skew_wd = init_busy ? DEFAULT_SKEW_CYCLES : cfg_wdata;

    always @(posedge clk) begin   // 存储器：只带时钟、无复位
        if (wr_en) begin
            st_ram[waddr] <= st_wd;
            cn_ram[waddr] <= cn_wd;
        end
        if (cf_en_we)   cf_en_ram[cf_waddr]   <= cf_en_wd;
        if (cf_skew_we) cf_skew_ram[cf_waddr] <= cf_skew_wd;
        st_q      <= st_ram[raddr];
        cn_q      <= cn_ram[raddr];
        cf_en_q   <= cf_en_ram[raddr];
        cf_skew_q <= cf_skew_ram[raddr];
    end

    // ------------------------------------------------------------
    // S_EXEC 组合计算 (cur_*, st_q, cn_q, cf_*_q -> 下一状态/标志)
    // ------------------------------------------------------------
    wire [7:0] q_exp_a   = st_q[7:0];
    wire       q_init_a  = st_q[8];
    wire [7:0] q_exp_b   = st_q[16:9];
    wire       q_init_b  = st_q[17];
    wire [7:0] q_ilast   = st_q[25:18];
    wire       q_rm_init = st_q[26];
    wire [7:0] q_rm_seq  = st_q[34:27];
    wire       q_rm_net  = st_q[35];
    wire [31:0] q_rm_ts  = st_q[67:36];

    wire [31:0] elapsed   = cyc - q_rm_ts;
    wire        in_skew   = (elapsed <= cf_skew_q);
    wire [7:0]  exp_cur   = cur_net ? q_exp_b  : q_exp_a;
    wire        init_cur  = cur_net ? q_init_b : q_init_a;
    wire [8:0]  fwd       = seq_forward_distance(q_rm_seq, cur_seq);

    reg f_fail, f_accept, f_dup, f_rm_err;

    always @(*) begin
        f_fail = 1'b0; f_accept = 1'b0; f_dup = 1'b0; f_rm_err = 1'b0;
        n_st = st_q;
        n_cn = cn_q;

        // ---------------- Integrity Checker ----------------
        if (cur_error) begin
            f_fail = 1'b1;
        end else if (cf_en_q && init_cur && (cur_seq != 8'd0) &&
                     (cur_seq != exp_cur) && (cur_seq != next_seq(exp_cur))) begin
            f_fail = 1'b1;
        end

        // PSN更新：被丢弃的帧也更新(XAPP1130: 允许被丢弃的数据流恢复)
        if (cf_en_q) begin
            n_st[25:18] = cur_seq;
            if (!cur_net) begin n_st[8]  = 1'b1; n_st[7:0]  = next_seq(cur_seq); end
            else          begin n_st[17] = 1'b1; n_st[16:9] = next_seq(cur_seq); end
        end

        if (f_fail) begin
            n_cn[63:32] = cn_q[63:32] + 32'd1;                    // rx_errors
            if (!cur_error) n_cn[127:96] = cn_q[127:96] + 32'd1;  // seq_errors
        end else begin
            // ---------------- Redundancy Manager ----------------
            if (!q_rm_init) begin
                f_accept = 1'b1;
                n_st[26] = 1'b1;
            end else if ((cur_seq == q_rm_seq) && (cur_net != q_rm_net) && in_skew) begin
                f_dup = 1'b1;
                n_cn[95:64] = cn_q[95:64] + 32'd1;                // duplicates
            end else if ((cur_seq == q_rm_seq) && (cur_net == q_rm_net)) begin
                f_rm_err = 1'b1;
            end else if (cur_seq == 8'd0) begin
                f_accept = 1'b1;                                   // 发送端复位
            end else if ((fwd > 9'd0) && (fwd <= 9'd127)) begin
                f_accept = 1'b1;
            end else if ((cur_seq == q_rm_seq) && (cur_net != q_rm_net)) begin
                f_accept = 1'b1;                                   // 超过SkewMAX的同SN帧
            end else begin
                f_rm_err = 1'b1;
            end

            if (f_accept) begin
                n_st[34:27] = cur_seq;
                n_st[35]    = cur_net;
                n_st[67:36] = cyc;
                n_cn[31:0]  = cn_q[31:0] + 32'd1;                 // rx_frames
            end
            if (f_rm_err) n_cn[63:32] = cn_q[63:32] + 32'd1;      // rx_errors
        end
    end

    // ------------------------------------------------------------
    // 主状态机
    // ------------------------------------------------------------
    always @(*) exec_wr = (state == S_EXEC);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            init_cnt       <= {(RX_IDX_W+1){1'b0}};
            init_done      <= 1'b0;
            cur_net        <= 1'b0; cur_error <= 1'b0;
            cur_vlid       <= {RX_IDX_W{1'b0}};
            cur_seq        <= 8'd0; cur_len <= 11'd0;
            out_valid      <= 1'b0; out_vlid <= {RX_IDX_W{1'b0}};
            out_seq_num    <= 8'd0; out_frame_len <= 11'd0; out_network <= 1'b0;
            out_error      <= 1'b0; sequence_error <= 1'b0; duplicate_drop <= 1'b0;
            event_valid    <= 1'b0; event_vlid <= {RX_IDX_W{1'b0}};
            event_seq_num  <= 8'd0; event_network <= 1'b0;
            mib_pend       <= 1'b0; mib_obj <= 4'd0; mib_vl <= {RX_IDX_W{1'b0}};
            mib_rvalid     <= 1'b0; mib_rdata <= 32'd0;
        end else begin
            out_valid      <= 1'b0;
            out_error      <= 1'b0;
            sequence_error <= 1'b0;
            duplicate_drop <= 1'b0;
            event_valid    <= 1'b0;
            mib_rvalid     <= 1'b0;

            if (init_busy) begin
                init_cnt <= init_cnt + 1'b1;
                if (init_cnt == NUM_RX_VL-1) init_done <= 1'b1;
            end

            if (mib_re && init_done && !mib_pend) begin
                mib_pend <= 1'b1;
                mib_obj  <= mib_addr[15:12];
                mib_vl   <= mib_addr[RX_IDX_W-1:0];
            end

            case (state)
                S_IDLE: begin
                    if (pop_do) begin
                        {cur_net, cur_vlid, cur_seq, cur_len, cur_error} <= fifo_head;
                        state <= S_EXEC;
                    end else if (mib_pend && init_done) begin
                        state <= S_MIB;      // raddr已选中mib_vl，下一拍q有效
                    end
                end

                S_EXEC: begin
                    event_valid   <= 1'b1;
                    event_vlid    <= cur_vlid;
                    event_seq_num <= cur_seq;
                    event_network <= cur_net;
                    if (f_fail) begin out_error <= 1'b1; sequence_error <= 1'b1; end
                    if (f_rm_err) out_error <= 1'b1;
                    if (f_dup)  duplicate_drop <= 1'b1;
                    if (f_accept) begin
                        out_valid     <= 1'b1;
                        out_vlid      <= cur_vlid;
                        out_seq_num   <= cur_seq;
                        out_frame_len <= cur_len;
                        out_network   <= cur_net;
                    end
                    state <= S_IDLE;
                end

                S_MIB: begin
                    case (mib_obj)
                        4'd0:    mib_rdata <= cn_q[31:0];
                        4'd1:    mib_rdata <= cn_q[63:32];
                        4'd2:    mib_rdata <= cn_q[95:64];
                        4'd3:    mib_rdata <= cn_q[127:96];
                        4'd4:    mib_rdata <= {24'd0, q_ilast};
                        default: mib_rdata <= 32'd0;
                    endcase
                    mib_rvalid <= 1'b1;
                    mib_pend   <= 1'b0;
                    state      <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // synthesis translate_off
    initial begin
        st_q = {ST_W{1'b0}}; cn_q = {CN_W{1'b0}}; cf_en_q = 1'b0; cf_skew_q = 32'd0;
    end
    // synthesis translate_on
endmodule
