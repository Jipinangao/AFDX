//=====================================================================
// 模块名 : ip_reasm
// 功能   : AFDX接收侧IP分片重组(RX侧)，与ip_frag.v(TX侧分片)对应，
//          共同实现选题指南"UDP/IP协议处理"里的分片/重组要求。
//
// 输入契约：
//   in_*来自afdx_rx_payload_deliver的deliver_*输出，即"已经过冗余
//   管理去重、已过滤错误帧"的单一分片载荷字节流。每个deliver突发
//   (in_valid连续为1直到in_last)对应恰好一个以太网帧(=一个IP分片)，
//   in_mf/in_frag_offset_8b/in_ip_id在整个突发期间保持不变。
//
// 架构决策：
//   1) 重组键用(VLID, IP.Identification)，而不是标准IP完整4元组。
//      AFDX每条VL本身唯一标识一条源-目的关系，Protocol恒为UDP，
//      所以(VLID,ID)足以消歧义。这是明确的简化，非标准原文写法。
//   2) 未分片单帧消息(FragOffset==0 且 MF==0)完全旁路，直接透传，
//      不占用重组上下文、不增加缓冲时延。
//   3) IC/RM已在上游以太网帧级别完成，本模块收到的每个分片都是
//      确定要收下的唯一副本，不需要再去重。
//   4) 分片重叠(同一字节位置被写两次)按"整条消息丢弃"处理并计数。
//   5) 输出端口由"透传旁路"和"重组完成后drain"共享，旁路享有绝对
//      优先级(它转发的是实时到来、无处缓冲的数据；drain的数据已安全
//      存于上下文缓冲，可以等)。已知限制：若单帧消息背靠背且中间
//      零空拍，drain会被推迟；真实以太网帧间必有IFG，不会出现。
//   8) 分片顺序：不再用逐块位图判重叠。同一VL的帧经完整性检查后按序到达，
//      所以分片必按偏移递增；延续分片的起点必须等于已收数据的末尾(ctx_next)，
//      否则(重叠或中间丢片)整条消息丢弃并计入reasm_overlap_count。
//   7) 存储结构按FPGA综合通用模板重做：
//      - 重组数据缓冲(4x8192x8=256Kbit)放进独立的afdx_sdp_ram(无复位、
//        同步读)，这样综合工具才能把它推断成块RAM，而不是展开成26万个
//        触发器(yosys实测：缓冲和带异步复位的状态机混在一个always块里
//        会被展开)。写口晚1拍生效，读口1拍延迟，drain改为"发地址/取数据"
//        两拍一字节——输出速率减半，换来可综合的存储结构。
//      - 位图由"逐字节"(8192位/上下文)改为"逐8字节块"(1024位/上下文)：
//        IP分片边界天然是8字节对齐的，块粒度足以判重叠，触发器数量降
//        到原来的1/8。每进入一个新的8字节块都检查一次，所以"起点没重叠
//        但中途压到别的分片"这类异常同样能抓住。
//   6) 输入侧无反压端口(上游deliver_*本身没有ready)，所以本模块必须
//      每拍都能接收，写入上下文缓冲是1拍1字节，满足这一点。
//=====================================================================
module ip_reasm #(
    parameter RX_IDX_W       = 12,
    parameter NUM_CTX        = 4,
    parameter MAX_MSG_BYTES  = 8192,
    parameter TIMEOUT_CYCLES = 200000
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        in_valid,
    input  wire [7:0]  in_data,
    input  wire        in_last,
    input  wire [RX_IDX_W-1:0] in_vlid,
    input  wire [15:0] in_ip_id,
    input  wire        in_mf,
    input  wire [12:0] in_frag_offset_8b,

    output reg          out_valid,
    output reg  [7:0]   out_data,
    output reg          out_last,
    output reg  [RX_IDX_W-1:0] out_vlid,

    output reg [31:0] reasm_timeout_count,
    output reg [31:0] reasm_overlap_count,
    output reg [31:0] reasm_overflow_count,
    output reg [31:0] reasm_orphan_count,
    output reg [31:0] reasm_complete_count
);

    localparam OFF_W = 13;

    // ==================== 重组上下文表 ====================
    reg                    ctx_valid        [0:NUM_CTX-1];
    reg [RX_IDX_W-1:0]     ctx_vlid         [0:NUM_CTX-1];
    reg [15:0]             ctx_ipid         [0:NUM_CTX-1];
    reg [OFF_W:0]          ctx_recv_cnt     [0:NUM_CTX-1];
    reg [OFF_W:0]          ctx_total_len    [0:NUM_CTX-1];
    reg                    ctx_total_known  [0:NUM_CTX-1];
    reg                    ctx_overlap_flag [0:NUM_CTX-1];
    reg                    ctx_pending_drain[0:NUM_CTX-1];
    reg [31:0]             ctx_timer        [0:NUM_CTX-1];
    // 下一个应到的用户数据偏移(=已写入的最后一个字节偏移+1)。
    // 同一VL的帧在完整性检查之后保证按序号递增到达，因此一条消息的分片必然按偏移顺序
    // 到达；延续分片的起点必须恰等于该值。小于它=重叠，大于它=中间有分片丢失，两种情形
    // 都丢弃整条消息(不再为逐块位图付出4x1024位寄存器和大型译码器)。
    reg [OFF_W:0]          ctx_next         [0:NUM_CTX-1];

    // ---- 重组数据缓冲：独立同步RAM(地址={上下文号,上下文内偏移}) ----
    // 约束：MAX_MSG_BYTES固定为8192(OFF_W=13)，NUM_CTX<=4，地址共15位。
    reg         ram_we;
    reg  [14:0] ram_waddr;
    reg  [7:0]  ram_wdata;
    wire [14:0] ram_raddr;
    wire [7:0]  ram_rdata;
    afdx_sdp_ram #(.AW(15), .DW(8), .DEPTH(NUM_CTX*MAX_MSG_BYTES)) u_ctx_ram (
        .clk(clk), .we(ram_we), .waddr(ram_waddr), .wdata(ram_wdata),
        .raddr(ram_raddr), .rdata(ram_rdata)
    );

    // ==================== 突发捕获状态 ====================
    reg                 bursting;
    reg                 cap_is_bypass;
    reg                 cap_drop;
    reg [1:0]           cap_ctx;
    reg [OFF_W-1:0]      cap_ptr;
    reg                 cap_mf;

    // ==================== drain FSM ====================
    localparam DR_IDLE = 2'd0;   // 找已完成的上下文
    localparam DR_ADDR = 2'd1;   // 读地址送入RAM(本拍采样)
    localparam DR_DATA = 2'd2;   // RAM数据已出，输出一个字节
    reg [1:0]           dr_state;
    reg [1:0]           dr_ctx;
    reg [OFF_W:0]        dr_ptr;
    reg [OFF_W:0]        dr_len;
    reg [RX_IDX_W-1:0]   dr_vlid;

    integer i;
    // 提升为模块级声明以保持纯Verilog-2001(不依赖SystemVerilog的块内局部声明)
    reg        found_free, found_ctx, found_pending;
    reg [1:0]  free_idx, match_idx, pend_idx;
    // 本拍被第一部分写入(需要重装超时)的上下文。Verilog里同一always块内对
    // 同一寄存器的多次非阻塞赋值，后者覆盖前者；如果第二部分的"超时递减"
    // 不避开本拍刚被重装的上下文，重装就会被覆盖而失效(大消息会被误判超时)。
    reg        touch_valid;
    reg [1:0]  touch_idx;

    // 用户数据位置换算(关键)：IP.FragOffset是按"IP载荷"(首片含8字节UDP头)
    // 计的偏移，但首片送进本模块的字节流已经不含UDP头(被frame_decap_filter
    // 吃掉)，所以延续分片的用户数据起始位置 = FragOffset*8 - 8。
    wire [15:0] cont_base = {in_frag_offset_8b,3'b000} - 16'd8;

    assign ram_raddr = {dr_ctx, dr_ptr[OFF_W-1:0]};

    // 是否本拍由"透传旁路"驱动输出(旁路优先于drain)
    wire new_burst_is_bypass = (in_frag_offset_8b == 13'd0) && (in_mf == 1'b0);
    wire bypass_drives_output = in_valid && (bursting ? cap_is_bypass : new_burst_is_bypass);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bursting      <= 1'b0;
            cap_is_bypass <= 1'b0;
            cap_drop      <= 1'b0;
            cap_ctx       <= 2'd0;
            cap_ptr       <= {OFF_W{1'b0}};
            cap_mf        <= 1'b0;

            ram_we    <= 1'b0;
            ram_waddr <= 15'd0;
            ram_wdata <= 8'd0;

            dr_state <= DR_IDLE;
            dr_ctx   <= 2'd0;
            dr_ptr   <= {(OFF_W+1){1'b0}};
            dr_len   <= {(OFF_W+1){1'b0}};
            dr_vlid  <= {RX_IDX_W{1'b0}};

            out_valid <= 1'b0;
            out_data  <= 8'd0;
            out_last  <= 1'b0;
            out_vlid  <= {RX_IDX_W{1'b0}};

            reasm_timeout_count  <= 32'd0;
            reasm_overlap_count  <= 32'd0;
            reasm_overflow_count <= 32'd0;
            reasm_orphan_count   <= 32'd0;
            reasm_complete_count <= 32'd0;

            for (i = 0; i < NUM_CTX; i = i + 1) begin
                ctx_valid[i]         <= 1'b0;
                ctx_vlid[i]          <= {RX_IDX_W{1'b0}};
                ctx_ipid[i]          <= 16'd0;
                ctx_recv_cnt[i]      <= {(OFF_W+1){1'b0}};
                ctx_total_len[i]     <= {(OFF_W+1){1'b0}};
                ctx_total_known[i]   <= 1'b0;
                ctx_overlap_flag[i]  <= 1'b0;
                ctx_pending_drain[i] <= 1'b0;
                ctx_timer[i]         <= 32'd0;
                ctx_next[i]          <= {(OFF_W+1){1'b0}};
            end
        end else begin
            out_valid <= 1'b0;
            out_last  <= 1'b0;
            ram_we <= 1'b0;
            touch_valid = 1'b0;
            touch_idx   = 2'd0;

            // =============================================================
            // 第一部分：输入突发的捕获/写入
            // =============================================================
            if (!bursting) begin
                if (in_valid) begin
                    if (new_burst_is_bypass) begin
                        // 未分片完整消息：旁路透传
                        out_valid <= 1'b1;
                        out_data  <= in_data;
                        out_vlid  <= in_vlid;
                        out_last  <= in_last;
                        cap_is_bypass <= 1'b1;
                        bursting      <= !in_last;
                    end else begin
                        cap_is_bypass <= 1'b0;
                        cap_mf        <= in_mf;
                        if (in_frag_offset_8b == 13'd0) begin
                            // 首片(MF必为1)：分配新上下文
                            found_free = 1'b0;
                            free_idx   = 2'd0;
                            for (i = 0; i < NUM_CTX; i = i + 1) begin
                                if (!found_free && !ctx_valid[i]) begin
                                    found_free = 1'b1;
                                    free_idx   = i[1:0];
                                end
                            end
                            if (found_free) begin
                                ctx_valid[free_idx]         <= 1'b1;
                                ctx_vlid[free_idx]          <= in_vlid;
                                ctx_ipid[free_idx]          <= in_ip_id;
                                ctx_total_known[free_idx]   <= 1'b0;
                                ctx_overlap_flag[free_idx]  <= 1'b0;
                                ctx_pending_drain[free_idx] <= 1'b0;
                                ctx_timer[free_idx]         <= TIMEOUT_CYCLES;
                                touch_valid = 1'b1; touch_idx = free_idx;
                                ctx_next[free_idx]          <= {{OFF_W{1'b0}},1'b1};
                                ctx_recv_cnt[free_idx]      <= {{OFF_W{1'b0}},1'b1};
                                ram_we    <= 1'b1;
                                ram_waddr <= {free_idx, {OFF_W{1'b0}}};
                                ram_wdata <= in_data;
                                cap_ctx  <= free_idx;
                                cap_ptr  <= 1;
                                cap_drop <= 1'b0;
                                bursting <= !in_last;
                            end else begin
                                reasm_overflow_count <= reasm_overflow_count + 32'd1;
                                cap_drop <= 1'b1;
                                bursting <= !in_last;
                            end
                        end else begin
                            // 延续分片：必须找到匹配上下文
                            found_ctx = 1'b0;
                            match_idx = 2'd0;
                            for (i = 0; i < NUM_CTX; i = i + 1) begin
                                if (!found_ctx && ctx_valid[i] &&
                                    ctx_vlid[i] == in_vlid && ctx_ipid[i] == in_ip_id) begin
                                    found_ctx = 1'b1;
                                    match_idx = i[1:0];
                                end
                            end
                            if (found_ctx) begin
                                ctx_timer[match_idx] <= TIMEOUT_CYCLES;
                                touch_valid = 1'b1; touch_idx = match_idx;
                                // 延续片必须恰好接在已收数据之后，否则整条消息作废
                                if (cont_base[OFF_W:0] != ctx_next[match_idx])
                                    ctx_overlap_flag[match_idx] <= 1'b1;
                                ctx_next[match_idx] <= cont_base[OFF_W:0] + 1'b1;
                                ctx_recv_cnt[match_idx] <= ctx_recv_cnt[match_idx] + 1;
                                ram_we    <= 1'b1;
                                ram_waddr <= {match_idx, cont_base[OFF_W-1:0]};
                                ram_wdata <= in_data;
                                cap_ctx  <= match_idx;
                                cap_ptr  <= cont_base[OFF_W-1:0] + 1;
                                cap_drop <= 1'b0;
                                bursting <= !in_last;
                                if (in_last && in_mf == 1'b0) begin
                                    ctx_total_len[match_idx]    <= cont_base[OFF_W:0] + 1;
                                    ctx_total_known[match_idx]  <= 1'b1;
                                end
                            end else begin
                                reasm_orphan_count <= reasm_orphan_count + 32'd1;
                                cap_drop <= 1'b1;
                                bursting <= !in_last;
                            end
                        end
                    end
                end
            end else begin
                // 同一突发内的后续字节
                if (in_valid) begin
                    if (cap_is_bypass) begin
                        out_valid <= 1'b1;
                        out_data  <= in_data;
                        out_vlid  <= in_vlid;
                        out_last  <= in_last;
                    end else if (!cap_drop) begin
                        ctx_next[cap_ctx] <= {1'b0, cap_ptr} + 1'b1;
                        ctx_recv_cnt[cap_ctx] <= ctx_recv_cnt[cap_ctx] + 1;
                        ram_we    <= 1'b1;
                        ram_waddr <= {cap_ctx, cap_ptr};
                        ram_wdata <= in_data;
                        ctx_timer[cap_ctx] <= TIMEOUT_CYCLES;
                        touch_valid = 1'b1; touch_idx = cap_ctx;
                        cap_ptr <= cap_ptr + 1;
                        if (in_last && cap_mf == 1'b0) begin
                            ctx_total_len[cap_ctx]   <= cap_ptr + 1;
                            ctx_total_known[cap_ctx] <= 1'b1;
                        end
                    end
                    if (in_last) bursting <= 1'b0;
                end
            end

            // =============================================================
            // 第二部分：后台扫描——完整性判定 / 重叠丢弃 / 超时释放
            // =============================================================
            for (i = 0; i < NUM_CTX; i = i + 1) begin
                if (ctx_valid[i] && !ctx_pending_drain[i]) begin
                    if (ctx_overlap_flag[i]) begin
                        reasm_overlap_count <= reasm_overlap_count + 32'd1;
                        ctx_valid[i] <= 1'b0;
                    end else if (ctx_total_known[i] && ctx_recv_cnt[i] == ctx_total_len[i]) begin
                        ctx_pending_drain[i] <= 1'b1;
                    end else if (touch_valid && touch_idx == i[1:0]) begin
                        // 本拍刚收到该上下文的字节，超时已在第一部分重装，这里不递减
                    end else if (ctx_timer[i] == 32'd0) begin
                        reasm_timeout_count <= reasm_timeout_count + 32'd1;
                        ctx_valid[i] <= 1'b0;
                    end else begin
                        ctx_timer[i] <= ctx_timer[i] - 32'd1;
                    end
                end
            end

            // =============================================================
            // 第三部分：drain FSM——旁路优先，drain让路
            // =============================================================
            if (bypass_drives_output) begin
                // 旁路抢占输出：流水线在途的那个字节会丢，退回一个地址，让drain恢复时重新读它
                if (dr_state == DR_DATA) begin
                    dr_ptr   <= dr_ptr - 1;
                    dr_state <= DR_ADDR;
                end
            end else begin
                case (dr_state)
                    DR_IDLE: begin
                        found_pending = 1'b0;
                        pend_idx      = 2'd0;
                        for (i = 0; i < NUM_CTX; i = i + 1) begin
                            if (!found_pending && ctx_pending_drain[i]) begin
                                found_pending = 1'b1;
                                pend_idx      = i[1:0];
                            end
                        end
                        if (found_pending) begin
                            dr_ctx   <= pend_idx;
                            dr_ptr   <= {(OFF_W+1){1'b0}};
                            dr_len   <= ctx_total_len[pend_idx];
                            dr_vlid  <= ctx_vlid[pend_idx];
                            dr_state <= DR_ADDR;
                        end
                    end
                    DR_ADDR: begin
                        // 送出第0个地址(ram_raddr={dr_ctx,dr_ptr}在本拍时钟沿被RAM采样)，
                        // 同时把指针推进到1：此后每拍同时"输出上一拍读出的字节"和"送下一个地址"，
                        // 流水线每拍1字节，一条消息的out_valid连续。
                        dr_ptr   <= dr_ptr + 1;
                        dr_state <= DR_DATA;
                    end
                    DR_DATA: begin
                        // 本拍rdata是字节(dr_ptr-1)
                        out_valid <= 1'b1;
                        out_data  <= ram_rdata;
                        out_vlid  <= dr_vlid;
                        if (dr_ptr == dr_len) begin
                            out_last <= 1'b1;
                            dr_state <= DR_IDLE;
                            ctx_valid[dr_ctx]         <= 1'b0;
                            ctx_pending_drain[dr_ctx] <= 1'b0;
                            reasm_complete_count      <= reasm_complete_count + 32'd1;
                        end else begin
                            dr_ptr   <= dr_ptr + 1;
                        end
                    end
                    default: dr_state <= DR_IDLE;
                endcase
            end
        end
    end

endmodule
