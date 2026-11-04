//=====================================================================
// 模块名 : afdx_615a_loader
// 功能   : AFDX端系统 数据加卸载
//
// 设计范围与简化说明：
//   ARINC 615A《Software Data Loader Using Ethernet》规定的是一整套基于
//   以太网+FTP(或类似的可靠传输)的软件件加载协议，包含LUH(Load Upload
//   Header)/LUP(Load Upload Part)等文件级描述、目标硬件描述文件、多文件
//   会话管理等完整语义，属于应用层软件规程，并不是单一的硬件状态机可以
//   完整覆盖的范畴。
//   本模块实现的是"数据加卸载"这一功能在端系统硬件侧最核心、且可由数字
//   逻辑真实验证的子集：
//     1) LOAD REQUEST/ACCEPT/BUSY/REFUSE 会话建立握手
//        （对应615A中Load流程开始前的目标身份确认与忙-闲判定）；
//     2) 分块(Block)数据传输 + 每块CRC16校验 + 顺序号校验 + 逐块ACK
//        （对应615A要求的数据完整性校验与传输状态回报）；
//     3) 加载完成状态查询/回报（对应615A的加载状态通知）；
//     4) 数据回读(Upload/卸载方向)，用于验证已加载内容，对应"加卸载"
//        中的"卸载"（数据回传/校验）语义。
//   本模块不实现基于文件系统的LUH/LUP解析、FTP/TFTP传输层、多目标硬件
//   描述文件等内容；报文格式为本项目按上述核心语义自定义的简化封装
//   （通过UDP承载），端口号UDP_LOAD_PORT为项目自选，不代表某具体标准
//   端口。
//=====================================================================
module afdx_615a_loader #(
    parameter integer MEM_DEPTH      = 4096,
    parameter [31:0]  TARGET_HW_ID   = 32'hA5A5_0001,
    parameter [15:0]  UDP_LOAD_PORT  = 16'd6150,
    parameter integer MAX_BLOCK_LEN  = 64,   // 每个LOAD_DATA块的最大数据字节数
    parameter integer MAX_UPLOAD_LEN = 64    // 每次UPLOAD_REQUEST最大回读字节数
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  udp_rx_data,
    input  wire        udp_rx_valid,
    input  wire        udp_rx_last,
    input  wire [15:0] udp_rx_src_port,
    input  wire [15:0] udp_rx_dst_port,

    output reg  [7:0]  udp_tx_data,
    output reg         udp_tx_valid,
    output reg         udp_tx_last,
    output reg  [15:0] udp_tx_src_port,
    output reg  [15:0] udp_tx_dst_port,
    input  wire        udp_tx_ready,

    // ---- 状态/诊断，供MIB/上层监控读取 ----
    output reg  [2:0]  session_state,   // 0=IDLE 1=OPEN(加载中) 2=COMPLETE 3=FAILED
    output reg  [31:0] bytes_loaded,
    output reg  [31:0] image_len,
    output reg  [31:0] load_crc_errors,
    output reg  [31:0] load_seq_errors,
    output reg  [31:0] load_complete_count,
    output reg  [31:0] load_failed_count,
    output reg  [31:0] upload_count
);

    // ==================== 简化"目标存储器"(等效Flash/应用镜像区) ====================
    // 同步读/写块RAM(无复位)。写口只在"数据块落盘"那一拍有效，读口供UPLOAD回读，
    // 数据比地址晚一拍(见P_UP_COPY)。
    wire        mem_we;
    wire [11:0] mem_waddr;
    wire [7:0]  mem_wdata;
    wire [11:0] mem_raddr;
    wire [7:0]  mem_q;
    afdx_sdp_ram #(.AW(12), .DW(8), .DEPTH(MEM_DEPTH)) u_image_mem (
        .clk(clk), .we(mem_we), .waddr(mem_waddr), .wdata(mem_wdata), .raddr(mem_raddr), .rdata(mem_q));

    // ==================== 操作码定义 ====================
    localparam [7:0] OP_LOAD_REQUEST      = 8'h01;
    localparam [7:0] OP_LOAD_DATA         = 8'h02;
    localparam [7:0] OP_LOAD_COMPLETE_QRY = 8'h03;
    localparam [7:0] OP_UPLOAD_REQUEST    = 8'h10;

    localparam [7:0] RSP_LOAD_ACCEPTED    = 8'h81;
    localparam [7:0] RSP_LOAD_BUSY        = 8'h82;
    localparam [7:0] RSP_LOAD_REFUSED     = 8'h83;
    localparam [7:0] RSP_LOAD_DATA_ACK    = 8'h84;
    localparam [7:0] RSP_LOAD_STATUS      = 8'h85;
    localparam [7:0] RSP_UPLOAD_DATA      = 8'h90;

    localparam [7:0] ACK_OK          = 8'd0;
    localparam [7:0] ACK_CRC_ERR     = 8'd1;
    localparam [7:0] ACK_SEQ_ERR     = 8'd2;
    localparam [7:0] ACK_OVERFLOW    = 8'd3;

    localparam S_IDLE   = 3'd0;
    localparam S_OPEN   = 3'd1;
    localparam S_DONE   = 3'd2;
    localparam S_FAILED = 3'd3;

    // ==================== RX/TX 缓冲与解析状态机 ====================
    localparam RX_BUF_DEPTH = 8 + MAX_BLOCK_LEN + 2; // opcode+seq+len+data+crc16
    localparam TX_BUF_DEPTH = 8 + MAX_UPLOAD_LEN;

    localparam P_RX       = 4'd0;
    localparam P_PARSE    = 4'd1;
    localparam P_CRC      = 4'd2;
    localparam P_WRITE    = 4'd3;
    localparam P_UP_SETUP = 4'd4;
    localparam P_UP_COPY  = 4'd5;
    localparam P_BUILD    = 4'd6;
    localparam P_TX       = 4'd7;

    reg [3:0] pstate;
    reg [7:0] rx_buf [0:RX_BUF_DEPTH-1];
    reg [7:0] tx_buf [0:TX_BUF_DEPTH-1];
    reg [10:0] rx_cnt;
    reg [10:0] rx_len;
    reg [10:0] tx_len;
    reg [10:0] tx_idx;
    reg [15:0] reply_port;

    reg [7:0]  opcode;
    reg [31:0] req_hw_id;
    reg [15:0] req_image_len;
    reg [15:0] blk_seq;
    reg [7:0]  blk_len;
    reg [15:0] blk_crc_rx;
    reg [15:0] up_offset;
    reg [15:0] up_len;
    reg [15:0] up_len_clamped;

    reg [15:0] crc_acc;
    reg [7:0]  copy_idx;

    reg [15:0] expected_seq;

    // 显式锁存本次处理结果，供P_BUILD直接使用，避免事后从会话状态反推
    // 造成的歧义（例如expected_seq在P_WRITE中已经被修改，若在P_BUILD里
    // 用它反推"这一块是否成功"会不可靠）。
    reg [1:0] req_result;    // 0=ACCEPTED 1=BUSY 2=REFUSED
    reg [1:0] blk_result;    // 0=OK 1=CRC_ERR 2=SEQ_ERR 3=OVERFLOW

    integer i;

    function [15:0] crc16_update;
        input [15:0] crc_in;
        input [7:0]  data_in;
        integer k;
        reg [15:0] c;
        begin
            c = crc_in ^ ({data_in, 8'h00});
            for (k = 0; k < 8; k = k + 1) begin
                if (c[15]) c = (c << 1) ^ 16'h1021;
                else       c = (c << 1);
            end
            crc16_update = c;
        end
    endfunction

    // 落盘写口：与P_WRITE里"通过全部检查、逐字节搬移"的分支条件一致
    wire wr_data_path = (pstate == P_WRITE) && (opcode != OP_LOAD_REQUEST) &&
                        (session_state == S_OPEN) && (blk_crc_rx == crc_acc) && (blk_seq == expected_seq) &&
                        !(bytes_loaded + {24'd0, blk_len} > MEM_DEPTH) && (copy_idx < blk_len);
    assign mem_we    = wr_data_path;
    assign mem_waddr = bytes_loaded[11:0] + {4'd0, copy_idx};
    assign mem_wdata = rx_buf[4 + copy_idx];
    assign mem_raddr = up_offset[11:0] + {4'd0, copy_idx};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pstate              <= P_RX;
            rx_cnt              <= 11'd0;
            rx_len              <= 11'd0;
            tx_len              <= 11'd0;
            tx_idx              <= 11'd0;
            reply_port          <= 16'd0;
            opcode              <= 8'd0;
            req_hw_id           <= 32'd0;
            req_image_len       <= 16'd0;
            blk_seq             <= 16'd0;
            blk_len             <= 8'd0;
            blk_crc_rx          <= 16'd0;
            up_offset           <= 16'd0;
            up_len              <= 16'd0;
            up_len_clamped      <= 16'd0;
            crc_acc             <= 16'hFFFF;
            copy_idx            <= 8'd0;
            expected_seq        <= 16'd0;
            req_result          <= 2'd0;
            blk_result          <= 2'd0;

            session_state       <= S_IDLE;
            bytes_loaded        <= 32'd0;
            image_len           <= 32'd0;
            load_crc_errors     <= 32'd0;
            load_seq_errors     <= 32'd0;
            load_complete_count <= 32'd0;
            load_failed_count   <= 32'd0;
            upload_count        <= 32'd0;

            udp_tx_data         <= 8'd0;
            udp_tx_valid        <= 1'b0;
            udp_tx_last         <= 1'b0;
            udp_tx_src_port     <= UDP_LOAD_PORT;
            udp_tx_dst_port     <= 16'd0;

        end else begin
            udp_tx_valid <= 1'b0;
            udp_tx_last  <= 1'b0;

            case (pstate)
                // ---------------- 接收一个完整UDP报文 ----------------
                P_RX: begin
                    if (udp_rx_valid && udp_rx_dst_port == UDP_LOAD_PORT) begin
                        if (rx_cnt < RX_BUF_DEPTH)
                            rx_buf[rx_cnt] <= udp_rx_data;
                        if (udp_rx_last) begin
                            rx_len     <= rx_cnt + 11'd1;
                            reply_port <= udp_rx_src_port;
                            pstate     <= P_PARSE;
                        end else begin
                            rx_cnt <= rx_cnt + 11'd1;
                        end
                    end
                end

                // ---------------- 解析报文头，分发到相应处理 ----------------
                P_PARSE: begin
                    opcode <= rx_buf[0];
                    case (rx_buf[0])
                        OP_LOAD_REQUEST: begin
                            req_hw_id     <= {rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]};
                            req_image_len <= {rx_buf[5], rx_buf[6]};
                            pstate <= P_WRITE; // 复用WRITE状态做会话建立判定(无需CRC/搬移)
                        end

                        OP_LOAD_DATA: begin
                            blk_seq    <= {rx_buf[1], rx_buf[2]};
                            blk_len    <= rx_buf[3];
                            crc_acc    <= 16'hFFFF;
                            copy_idx   <= 8'd0;
                            pstate     <= P_CRC;
                        end

                        OP_LOAD_COMPLETE_QRY: begin
                            pstate <= P_BUILD;
                        end

                        OP_UPLOAD_REQUEST: begin
                            up_offset <= {rx_buf[1], rx_buf[2]};
                            up_len    <= {rx_buf[3], rx_buf[4]};
                            copy_idx  <= 8'd0;
                            pstate    <= P_UP_SETUP;
                        end

                        default: begin
                            rx_cnt <= 11'd0;
                            pstate <= P_RX;
                        end
                    endcase
                end

                // ---------------- LOAD_DATA: 逐字节计算CRC16(仅覆盖数据区) ----------------
                P_CRC: begin
                    if (copy_idx < blk_len) begin
                        crc_acc  <= crc16_update(crc_acc, rx_buf[4 + copy_idx]);
                        copy_idx <= copy_idx + 8'd1;
                    end else begin
                        blk_crc_rx <= {rx_buf[4 + blk_len], rx_buf[5 + blk_len]};
                        copy_idx   <= 8'd0;
                        pstate     <= P_WRITE;
                    end
                end

                // ---------------- 会话建立 / 数据落盘 ----------------
                P_WRITE: begin
                    if (opcode == OP_LOAD_REQUEST) begin
                        if (session_state == S_OPEN) begin
                            // 已有会话进行中：忙-拒绝新的加载请求，不打断当前会话。
                            req_result <= 2'd1; // BUSY
                            pstate <= P_BUILD;
                        end else if (req_hw_id != TARGET_HW_ID) begin
                            req_result <= 2'd2; // REFUSED
                            pstate <= P_BUILD;
                        end else begin
                            session_state <= S_OPEN;
                            image_len     <= {16'd0, req_image_len};
                            bytes_loaded  <= 32'd0;
                            expected_seq  <= 16'd0;
                            req_result    <= 2'd0; // ACCEPTED
                            pstate        <= P_BUILD;
                        end
                    end else begin
                        // OP_LOAD_DATA落盘路径
                        if (session_state != S_OPEN) begin
                            // 未建立会话时收到的数据块，直接按序号错误应答，不写入。
                            load_seq_errors <= load_seq_errors + 32'd1;
                            blk_result <= 2'd2; // SEQ_ERR
                            pstate <= P_BUILD;
                        end else if (blk_crc_rx != crc_acc) begin
                            load_crc_errors <= load_crc_errors + 32'd1;
                            blk_result <= 2'd1; // CRC_ERR
                            pstate <= P_BUILD;
                        end else if (blk_seq != expected_seq) begin
                            load_seq_errors <= load_seq_errors + 32'd1;
                            blk_result <= 2'd2; // SEQ_ERR
                            pstate <= P_BUILD;
                        end else if (bytes_loaded + {24'd0, blk_len} > MEM_DEPTH) begin
                            // 落盘越界：判定会话失败，不写入。
                            session_state     <= S_FAILED;
                            load_failed_count <= load_failed_count + 32'd1;
                            blk_result        <= 2'd3; // OVERFLOW
                            pstate <= P_BUILD;
                        end else begin
                            if (copy_idx < blk_len) begin
                                copy_idx <= copy_idx + 8'd1;
                            end else begin
                                bytes_loaded <= bytes_loaded + {24'd0, blk_len};
                                expected_seq <= expected_seq + 16'd1;
                                blk_result   <= 2'd0; // OK
                                // 达到声明的镜像长度即视为加载完成。
                                if (bytes_loaded + {24'd0, blk_len} >= image_len)
                                    session_state <= S_DONE;
                                pstate <= P_BUILD;
                            end
                        end
                    end
                end

                // ---------------- UPLOAD_REQUEST: 先算出实际可回读长度(1拍) ----------------
                // 越界(offset超出MEM_DEPTH)或请求长度超过MAX_UPLOAD_LEN/剩余空间时静默裁剪，
                // 不作为错误上报（简化处理，已在模块头部注释中说明）。
                P_UP_SETUP: begin
                    if (up_offset >= MEM_DEPTH[15:0])
                        up_len_clamped <= 16'd0;
                    else if (up_len > MAX_UPLOAD_LEN[15:0])
                        up_len_clamped <= MAX_UPLOAD_LEN[15:0];
                    else if (up_offset + up_len > MEM_DEPTH[15:0])
                        up_len_clamped <= MEM_DEPTH[15:0] - up_offset;
                    else
                        up_len_clamped <= up_len;
                    copy_idx <= 8'd0;
                    pstate   <= P_UP_COPY;
                end

                // ---------------- UPLOAD_REQUEST: 用已稳定的up_len_clamped逐字节搬移 ----------------
                P_UP_COPY: begin
                    // 周期k发出读地址up_offset+k，数据在周期k+1从mem_q到达，
                    // 因此第k+1拍写入tx_buf[5+k]；copy_idx==len的那一拍收最后一个字节。
                    if (copy_idx != 8'd0)
                        tx_buf[4 + copy_idx] <= mem_q;
                    if (copy_idx < up_len_clamped[7:0]) begin
                        copy_idx <= copy_idx + 8'd1;
                    end else begin
                        pstate <= P_BUILD;
                    end
                end

                // ---------------- 组装响应 ----------------
                P_BUILD: begin
                    case (opcode)
                        OP_LOAD_REQUEST: begin
                            tx_buf[0] <= (req_result == 2'd0) ? RSP_LOAD_ACCEPTED :
                                         (req_result == 2'd1) ? RSP_LOAD_BUSY :
                                         RSP_LOAD_REFUSED;
                            tx_len <= 11'd1;
                        end

                        OP_LOAD_DATA: begin
                            tx_buf[0] <= RSP_LOAD_DATA_ACK;
                            tx_buf[1] <= blk_seq[15:8];
                            tx_buf[2] <= blk_seq[7:0];
                            tx_buf[3] <= (blk_result == 2'd0) ? ACK_OK :
                                         (blk_result == 2'd1) ? ACK_CRC_ERR :
                                         (blk_result == 2'd2) ? ACK_SEQ_ERR :
                                         ACK_OVERFLOW;
                            tx_len <= 11'd4;
                        end

                        OP_LOAD_COMPLETE_QRY: begin
                            tx_buf[0] <= RSP_LOAD_STATUS;
                            tx_buf[1] <= (session_state == S_DONE) ? 8'd0 :
                                         (session_state == S_FAILED) ? 8'd2 : 8'd1;
                            tx_buf[2] <= bytes_loaded[31:24];
                            tx_buf[3] <= bytes_loaded[23:16];
                            tx_buf[4] <= bytes_loaded[15:8];
                            tx_buf[5] <= bytes_loaded[7:0];
                            tx_len <= 11'd6;
                            if (session_state == S_DONE)
                                load_complete_count <= load_complete_count + 32'd1;
                        end

                        OP_UPLOAD_REQUEST: begin
                            tx_buf[0] <= RSP_UPLOAD_DATA;
                            tx_buf[1] <= up_offset[15:8];
                            tx_buf[2] <= up_offset[7:0];
                            tx_buf[3] <= up_len_clamped[15:8];
                            tx_buf[4] <= up_len_clamped[7:0];
                            tx_len <= 11'd5 + {3'd0, up_len_clamped[7:0]};
                            upload_count <= upload_count + 32'd1;
                        end

                        default: tx_len <= 11'd0;
                    endcase
                    tx_idx          <= 11'd0;
                    udp_tx_src_port <= UDP_LOAD_PORT;
                    udp_tx_dst_port <= reply_port;
                    pstate          <= P_TX;
                end

                // ---------------- 发送响应，随后回到RX等待下一个报文 ----------------
                P_TX: begin
                    if (udp_tx_ready && tx_idx < tx_len) begin
                        udp_tx_data  <= tx_buf[tx_idx];
                        udp_tx_valid <= 1'b1;
                        udp_tx_last  <= (tx_idx == tx_len - 11'd1);
                        if (tx_idx == tx_len - 11'd1) begin
                            tx_idx <= 11'd0;
                            rx_cnt <= 11'd0;
                            pstate <= P_RX;
                        end else begin
                            tx_idx <= tx_idx + 11'd1;
                        end
                    end else if (tx_len == 11'd0) begin
                        rx_cnt <= 11'd0;
                        pstate <= P_RX;
                    end
                end

                default: pstate <= P_RX;
            endcase
        end
    end

endmodule
