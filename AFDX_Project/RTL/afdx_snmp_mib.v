//=====================================================================
// 模块名 : afdx_snmp_mib
// 功能   : AFDX SNMP MIB统一读接口
//          把完整性/冗余统计、Policing统计和链路/运行状态统一映射到
//          SNMP Agent可访问的32bit MIB地址空间。
//
// MIB地址定义：
//   [15:12]
//     0x0 : sysUpTimeMs          0x6 : policingDrops(VL)
//     0x1 : linkStatus           0x7 : lastSeq(VL)
//     0x2 : rxFrames(VL)         0x8 : txFrames
//     0x3 : rxErrors(VL)         0x9 : txErrors
//     0x4 : duplicates(VL)
//     0x5 : seqErrors(VL)
//   [11:0] : VL ID
//
// 读时序：mib_re脉冲后，mib_rvalid脉冲时mib_rdata有效。对每VL对象，
//   本模块向完整性/Policer模块发起读请求并等待其*_rvalid握手(读延迟
//   由被读模块决定，不再假设固定拍数)。同一时刻只处理一个请求。
//=====================================================================
module afdx_snmp_mib (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         mib_re,
    input  wire [15:0]  mib_addr,
    output reg          mib_rvalid,
    output reg  [31:0]  mib_rdata,

    input  wire [31:0]  sys_uptime_ms,
    input  wire         link_status,
    input  wire [31:0]  tx_frames,
    input  wire [31:0]  tx_errors,

    output reg          int_mib_re,
    output reg  [15:0]  int_mib_addr,
    input  wire         int_mib_rvalid,
    input  wire [31:0]  int_mib_rdata,

    output reg          pol_mib_re,
    output reg  [11:0]  pol_mib_vlid,
    input  wire         pol_mib_rvalid,
    input  wire [31:0]  pol_mib_drop_count
);

    localparam [1:0] S_IDLE = 2'd0, S_WAIT_INT = 2'd1, S_WAIT_POL = 2'd2;
    reg [1:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            mib_rvalid   <= 1'b0;
            mib_rdata    <= 32'd0;
            int_mib_re   <= 1'b0;
            int_mib_addr <= 16'd0;
            pol_mib_re   <= 1'b0;
            pol_mib_vlid <= 12'd0;
        end else begin
            mib_rvalid <= 1'b0;
            int_mib_re <= 1'b0;
            pol_mib_re <= 1'b0;

            case (state)
                S_IDLE: if (mib_re) begin
                    case (mib_addr[15:12])
                        4'h0: begin mib_rdata <= sys_uptime_ms;        mib_rvalid <= 1'b1; end
                        4'h1: begin mib_rdata <= {31'd0, link_status}; mib_rvalid <= 1'b1; end
                        4'h8: begin mib_rdata <= tx_frames;            mib_rvalid <= 1'b1; end
                        4'h9: begin mib_rdata <= tx_errors;            mib_rvalid <= 1'b1; end
                        4'h2: begin int_mib_re <= 1'b1; int_mib_addr <= {4'd0, mib_addr[11:0]}; state <= S_WAIT_INT; end
                        4'h3: begin int_mib_re <= 1'b1; int_mib_addr <= {4'd1, mib_addr[11:0]}; state <= S_WAIT_INT; end
                        4'h4: begin int_mib_re <= 1'b1; int_mib_addr <= {4'd2, mib_addr[11:0]}; state <= S_WAIT_INT; end
                        4'h5: begin int_mib_re <= 1'b1; int_mib_addr <= {4'd3, mib_addr[11:0]}; state <= S_WAIT_INT; end
                        4'h7: begin int_mib_re <= 1'b1; int_mib_addr <= {4'd4, mib_addr[11:0]}; state <= S_WAIT_INT; end
                        4'h6: begin pol_mib_re <= 1'b1; pol_mib_vlid <= mib_addr[11:0];         state <= S_WAIT_POL; end
                        default: begin mib_rdata <= 32'd0; mib_rvalid <= 1'b1; end
                    endcase
                end

                S_WAIT_INT: if (int_mib_rvalid) begin
                    mib_rdata  <= int_mib_rdata;
                    mib_rvalid <= 1'b1;
                    state      <= S_IDLE;
                end

                S_WAIT_POL: if (pol_mib_rvalid) begin
                    mib_rdata  <= pol_mib_drop_count;
                    mib_rvalid <= 1'b1;
                    state      <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
