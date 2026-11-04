//=====================================================================
// 模块名 : afdx_sdp_ram
// 功能   : 简单双端口同步RAM(1写口+1读口，读口寄存输出，先读后写)
//
// 读写时序：读地址在时钟沿采样，数据在下一拍从rdata出现(1拍读延迟)。
//   同一地址同拍读写时，读到的是写入之前的旧值(read-first)。
//=====================================================================
module afdx_sdp_ram #(
    parameter AW    = 15,
    parameter DW    = 8,
    parameter DEPTH = (1 << AW)
)(
    input  wire          clk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [DW-1:0] wdata,
    input  wire [AW-1:0] raddr,
    output reg  [DW-1:0] rdata
);
    (* ramstyle = "M9K", syn_ramstyle = "block_ram" *) reg [DW-1:0] mem [0:DEPTH-1];

    always @(posedge clk) begin
        if (we)
            mem[waddr] <= wdata;
        rdata <= mem[raddr];
    end

    // 仿真用初值，避免读未写地址时出X；综合时忽略(上电内容由器件决定)。
    // synthesis translate_off
    initial rdata = {DW{1'b0}};
    // synthesis translate_on
endmodule
