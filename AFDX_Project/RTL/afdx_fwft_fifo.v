//=====================================================================
// 模块名 : afdx_fwft_fifo
// 功能   : 以同步读RAM实现的"先出字预读"(show-ahead)FIFO。
//          队首数据在rd_valid=1时由rd_data直接给出，rd_en弹出一项。
//          存储体是afdx_sdp_ram(无复位、读口寄存)，可映射为块RAM；
//          队首/在途各占一个寄存器级，所以总容量=2^AW+2。
// 时序   : wr_en后约3拍队首可见；弹出后约3拍下一项可见(每项最多每3拍弹一次)。
//=====================================================================
module afdx_fwft_fifo #(
    parameter DW = 8,
    parameter AW = 5
)(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          wr_en,
    input  wire [DW-1:0] wr_data,
    output wire          full,
    input  wire          rd_en,
    output reg  [DW-1:0] rd_data,
    output reg           rd_valid
);
    // 综合工具只把深度>=4096的存储体推断为块RAM，更浅的会被展开成逻辑单元，
    // 所以物理RAM深度补到2^12；FIFO只使用低2^AW个地址。
    localparam RAW = (AW < 12) ? 12 : AW;
    reg [AW:0] wr_ptr, fetch_ptr;
    reg        s_valid;                         // 一次RAM读在途
    wire [DW-1:0] ram_q;

    assign full = ((wr_ptr - fetch_ptr) == (1 << AW));
    wire do_wr   = wr_en & ~full;
    wire can_fetch = ((!rd_valid) || rd_en) && !s_valid && (fetch_ptr != wr_ptr);

    afdx_sdp_ram #(.AW(RAW), .DW(DW), .DEPTH(1 << RAW)) u_ram (
        .clk(clk), .we(do_wr), .waddr({{(RAW-AW){1'b0}}, wr_ptr[AW-1:0]}), .wdata(wr_data),
        .raddr({{(RAW-AW){1'b0}}, fetch_ptr[AW-1:0]}), .rdata(ram_q));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= {(AW+1){1'b0}}; fetch_ptr <= {(AW+1){1'b0}};
            s_valid <= 1'b0; rd_valid <= 1'b0; rd_data <= {DW{1'b0}};
        end else begin
            if (do_wr) wr_ptr <= wr_ptr + 1'b1;
            if (rd_en && rd_valid) rd_valid <= 1'b0;
            s_valid <= 1'b0;
            if (can_fetch) begin
                fetch_ptr <= fetch_ptr + 1'b1;
                s_valid   <= 1'b1;
            end
            if (s_valid) begin
                rd_data  <= ram_q;
                rd_valid <= 1'b1;
            end
        end
    end
endmodule
