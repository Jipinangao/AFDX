//=====================================================================
// 模块名 : afdx_snmp_agent
// 功能   : 精简SNMPv1 Agent
//          - UDP/161入口由上层UDP/IP模块提供，仅处理UDP payload
//          - 支持 GET(0xA0) / GetNext(0xA1)
//          - 只读 community = "public"
//          - 单次请求支持1个VarBind
//          - 支持短BER长度编码(<128)，足够覆盖本阶段MIB查询
//
// MIB OID：私有实验子树
//   1.3.6.1.4.1.99999.1.1.1.0   sysUpTimeMs
//   1.3.6.1.4.1.99999.1.1.2.0   linkStatus
//   1.3.6.1.4.1.99999.1.2.1.H.L rxFrames(VL)
//   1.3.6.1.4.1.99999.1.2.2.H.L rxErrors(VL)
//   1.3.6.1.4.1.99999.1.2.3.H.L duplicates(VL)
//   1.3.6.1.4.1.99999.1.2.4.H.L seqErrors(VL)
//   1.3.6.1.4.1.99999.1.2.5.H.L policingDrops(VL)
//   1.3.6.1.4.1.99999.1.2.6.H.L lastSeq(VL)
//
// H/L为VL[11:8]/VL[7:0]两个OID子标识，因此可覆盖4096个RX VL。
//=====================================================================
module afdx_snmp_agent #(
    parameter RX_BUF_DEPTH = 256,
    parameter TX_BUF_DEPTH = 256
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
    output reg [15:0]  udp_tx_src_port,
    output reg [15:0]  udp_tx_dst_port,
    input  wire        udp_tx_ready,

    output reg         mib_re,
    output reg [15:0]  mib_addr,
    input  wire        mib_rvalid,
    input  wire [31:0] mib_rdata
);

    localparam S_RX      = 3'd0;
    localparam S_PARSE   = 3'd1;
    localparam S_MIBWAIT = 3'd2;
    localparam S_BUILD   = 3'd3;
    localparam S_TX      = 3'd4;

    reg [2:0] state;
    reg [7:0] rx_buf [0:RX_BUF_DEPTH-1];
    reg [7:0] tx_buf [0:TX_BUF_DEPTH-1];
    reg [7:0] rx_count;
    reg [7:0] rx_len;
    reg [7:0] parse_index;
    reg [7:0] tx_len;
    reg [7:0] tx_index;

    reg [31:0] request_id;
    reg [15:0] reply_dst_port;
    reg [7:0] request_pdu_type;
    reg [7:0] req_oid [0:15];
    reg [4:0] req_oid_len;
    reg req_oid_valid;
    reg req_is_getnext;

    reg [3:0] mib_obj;
    reg [11:0] mib_vlid;
    reg [31:0] mib_value;
    reg [7:0] rsp_oid [0:15];
    reg [4:0] rsp_oid_len;
    reg rsp_valid;
    reg [7:0] rsp_value_type;

    integer i;

    function [7:0] oid_expected_byte;
        input [3:0] obj;
        input [11:0] vl;
        input [4:0] pos;
        begin
            case (pos)
                0:  oid_expected_byte = 8'h2B; // 1.3
                1:  oid_expected_byte = 8'h06;
                2:  oid_expected_byte = 8'h01;
                3:  oid_expected_byte = 8'h04;
                4:  oid_expected_byte = 8'h01;
                5:  oid_expected_byte = 8'h86;
                6:  oid_expected_byte = 8'h8D;
                7:  oid_expected_byte = 8'h1F;
                8:  oid_expected_byte = 8'h01;
                9:  oid_expected_byte = 8'h02;
                10: oid_expected_byte = obj;
                11: oid_expected_byte = vl[11:8];
                12: oid_expected_byte = vl[7:0];
                default: oid_expected_byte = 8'h00;
            endcase
        end
    endfunction

    function [7:0] scalar_expected_byte;
        input [1:0] leaf;
        input [4:0] pos;
        begin
            case (pos)
                0:  scalar_expected_byte = 8'h2B;
                1:  scalar_expected_byte = 8'h06;
                2:  scalar_expected_byte = 8'h01;
                3:  scalar_expected_byte = 8'h04;
                4:  scalar_expected_byte = 8'h01;
                5:  scalar_expected_byte = 8'h86;
                6:  scalar_expected_byte = 8'h8D;
                7:  scalar_expected_byte = 8'h1F;
                8:  scalar_expected_byte = 8'h01;
                9:  scalar_expected_byte = 8'h01;
                10: scalar_expected_byte = {6'd0, leaf};
                11: scalar_expected_byte = 8'h00;
                default: scalar_expected_byte = 8'h00;
            endcase
        end
    endfunction

    function integer match_oid;
        input dummy;
        integer k;
        reg same;
        begin
            match_oid = 0;
            // sysUpTime
            if (req_oid_len == 12) begin
                same = 1'b1;
                for (k = 0; k < 12; k = k + 1)
                    if (req_oid[k] != scalar_expected_byte(2'd1, k)) same = 1'b0;
                if (same) match_oid = 1;

                same = 1'b1;
                for (k = 0; k < 12; k = k + 1)
                    if (req_oid[k] != scalar_expected_byte(2'd2, k)) same = 1'b0;
                if (same) match_oid = 2;
            end

            // VL table OIDs: obj=1..6, VL=1..4095
            if (req_oid_len == 13) begin
                for (k = 1; k <= 6; k = k + 1) begin
                    same = 1'b1;
                    if (req_oid[0] != 8'h2B || req_oid[1] != 8'h06 || req_oid[2] != 8'h01 ||
                        req_oid[3] != 8'h04 || req_oid[4] != 8'h01 || req_oid[5] != 8'h86 ||
                        req_oid[6] != 8'h8D || req_oid[7] != 8'h1F || req_oid[8] != 8'h01 ||
                        req_oid[9] != 8'h02 || req_oid[10] != k[7:0]) same = 1'b0;
                    if (req_oid[11] == 8'd0 && req_oid[12] == 8'd0) same = 1'b0;
                    if (same) begin
                        match_oid = 10 + k;
                    end
                end
            end
        end
    endfunction

    task automatic set_response_oid;
        input integer oid_sel;
        input [11:0] vl;
        integer q;
        begin
            if (oid_sel == 1) begin
                rsp_oid_len = 12;
                for (q = 0; q < 12; q = q + 1)
                    rsp_oid[q] = scalar_expected_byte(2'd1, q);
            end else if (oid_sel == 2) begin
                rsp_oid_len = 12;
                for (q = 0; q < 12; q = q + 1)
                    rsp_oid[q] = scalar_expected_byte(2'd2, q);
            end else begin
                rsp_oid_len = 13;
                rsp_oid[0]=8'h2B; rsp_oid[1]=8'h06; rsp_oid[2]=8'h01; rsp_oid[3]=8'h04;
                rsp_oid[4]=8'h01; rsp_oid[5]=8'h86; rsp_oid[6]=8'h8D; rsp_oid[7]=8'h1F;
                rsp_oid[8]=8'h01; rsp_oid[9]=8'h02; rsp_oid[10]=oid_sel-10;
                rsp_oid[11]=vl[11:8]; rsp_oid[12]=vl[7:0];
            end
        end
    endtask

    task automatic load_tx;
        integer p;
        integer content_len;
        integer pdu_len;
        integer vb_len;
        integer vbl_len;
        integer msg_content_len;
        begin
            // 只返回一条VarBind，全部使用短BER长度编码。
            vb_len          = 8 + rsp_oid_len;  // VB内容长度：OID TLV + Value TLV
            vbl_len         = 2 + vb_len;        // VarBindList内容长度：一个VB序列
            pdu_len         = 24 + rsp_oid_len;  // Response-PDU内容长度
            msg_content_len = 37 + rsp_oid_len;
            content_len     = msg_content_len;

            tx_buf[0]  = 8'h30;
            tx_buf[1]  = content_len[7:0];
            tx_buf[2]  = 8'h02; tx_buf[3] = 8'h01; tx_buf[4] = 8'h00;
            tx_buf[5]  = 8'h04; tx_buf[6] = 8'h06;
            tx_buf[7]  = "p"; tx_buf[8] = "u"; tx_buf[9] = "b";
            tx_buf[10] = "l"; tx_buf[11] = "i"; tx_buf[12] = "c";
            tx_buf[13] = 8'hA2; tx_buf[14] = pdu_len[7:0];
            tx_buf[15] = 8'h02; tx_buf[16] = 8'h04;
            tx_buf[17] = request_id[31:24]; tx_buf[18] = request_id[23:16];
            tx_buf[19] = request_id[15:8];  tx_buf[20] = request_id[7:0];
            tx_buf[21] = 8'h02; tx_buf[22] = 8'h01; tx_buf[23] = 8'h00;
            tx_buf[24] = 8'h02; tx_buf[25] = 8'h01; tx_buf[26] = 8'h00;
            tx_buf[27] = 8'h30; tx_buf[28] = vbl_len[7:0];
            tx_buf[29] = 8'h30; tx_buf[30] = vb_len[7:0];
            tx_buf[31] = 8'h06; tx_buf[32] = rsp_oid_len;
            for (p = 0; p < 13; p = p + 1)
                if (p < rsp_oid_len) tx_buf[33+p] = rsp_oid[p];
            tx_buf[33+rsp_oid_len] = rsp_value_type;
            tx_buf[34+rsp_oid_len] = 8'h04;
            tx_buf[35+rsp_oid_len] = mib_value[31:24];
            tx_buf[36+rsp_oid_len] = mib_value[23:16];
            tx_buf[37+rsp_oid_len] = mib_value[15:8];
            tx_buf[38+rsp_oid_len] = mib_value[7:0];
            tx_len = 39 + rsp_oid_len;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_RX;
            rx_count         <= 8'd0;
            rx_len           <= 8'd0;
            parse_index      <= 8'd0;
            tx_len           <= 8'd0;
            tx_index         <= 8'd0;
            request_id       <= 16'd0;
            reply_dst_port   <= 16'd0;
            request_pdu_type <= 8'd0;
            req_oid_len      <= 5'd0;
            req_oid_valid    <= 1'b0;
            req_is_getnext   <= 1'b0;
            mib_obj          <= 4'd0;
            mib_vlid         <= 12'd0;
            mib_value        <= 32'd0;
            rsp_oid_len      <= 5'd0;
            rsp_valid        <= 1'b0;
            rsp_value_type   <= 8'h41;
            mib_re           <= 1'b0;
            mib_addr         <= 16'd0;
            udp_tx_data      <= 8'd0;
            udp_tx_valid     <= 1'b0;
            udp_tx_last      <= 1'b0;
            udp_tx_src_port  <= 16'd161;
            udp_tx_dst_port  <= 16'd0;
            for (i = 0; i < 16; i = i + 1) begin
                req_oid[i] <= 8'd0;
                rsp_oid[i] <= 8'd0;
            end
        end else begin
            udp_tx_valid <= 1'b0;
            udp_tx_last  <= 1'b0;
            mib_re       <= 1'b0;

            case (state)
                S_RX: begin
                    if (udp_rx_valid && udp_rx_dst_port == 16'd161) begin
                        if (rx_count < RX_BUF_DEPTH) begin
                            rx_buf[rx_count] <= udp_rx_data;
                            rx_count <= rx_count + 8'd1;
                        end
                        if (udp_rx_last) begin
                            rx_len <= rx_count + 8'd1;
                            parse_index <= 8'd0;
                            state <= S_PARSE;
                            reply_dst_port <= udp_rx_src_port;
                        end
                    end
                end

                S_PARSE: begin
                    if (rx_len >= 40 && rx_buf[0] == 8'h30 &&
                        rx_buf[2] == 8'h02 && rx_buf[3] == 8'h01 && rx_buf[4] == 8'h00 &&
                        rx_buf[5] == 8'h04 && rx_buf[6] == 8'h06 &&
                        rx_buf[7] == "p" && rx_buf[8] == "u" && rx_buf[9] == "b" &&
                        rx_buf[10] == "l" && rx_buf[11] == "i" && rx_buf[12] == "c" &&
                        (rx_buf[13] == 8'hA0 || rx_buf[13] == 8'hA1)) begin
                        request_pdu_type <= rx_buf[13];
                        req_is_getnext   <= (rx_buf[13] == 8'hA1);
                        request_id       <= {rx_buf[17], rx_buf[18], rx_buf[19], rx_buf[20]};
                        req_oid_len      <= rx_buf[32];
                        for (i = 0; i < 13; i = i + 1)
                            if (i < rx_buf[32]) req_oid[i] <= rx_buf[33+i];
                        req_oid_valid <= 1'b1;

                        // 识别目标OID：标量或VL统计项。
                        if (rx_buf[32] == 12 && rx_buf[43] == 8'h01) begin
                            // sysUpTime
                            if (rx_buf[13] == 8'hA1) begin
                                mib_obj <= 4'h1; // GETNEXT(sysUpTime) -> linkStatus
                            end else begin
                                mib_obj <= 4'h0;
                            end
                            mib_vlid <= 12'd0;
                            // linkStatus的GETNEXT继续在下一轮按固定顺序推进。
                            if (rx_buf[13] == 8'hA1) begin
                                set_response_oid(2, 12'd0);
                            end
                        end else if (rx_buf[32] == 12 && rx_buf[43] == 8'h02) begin
                            // linkStatus
                            if (rx_buf[13] == 8'hA1) begin
                                mib_obj <= 4'h2; mib_vlid <= 12'd1;
                            end else begin
                                mib_obj <= 4'h1; mib_vlid <= 12'd0;
                            end
                        end else if (rx_buf[32] == 13 &&
                                     rx_buf[0] == 8'h30 && rx_buf[33] == 8'h2B && rx_buf[34] == 8'h06 &&
                                     rx_buf[35] == 8'h01 && rx_buf[36] == 8'h04 && rx_buf[37] == 8'h01 &&
                                     rx_buf[38] == 8'h86 && rx_buf[39] == 8'h8D && rx_buf[40] == 8'h1F &&
                                     rx_buf[41] == 8'h01 && rx_buf[42] == 8'h02 &&
                                     rx_buf[43] >= 8'd1 && rx_buf[43] <= 8'd6) begin
                            if (rx_buf[13] == 8'hA1) begin
                                if (rx_buf[43] < 8'd6) begin
                                    mib_obj  <= rx_buf[43] + 8'd2;
                                    mib_vlid <= {rx_buf[44][3:0], rx_buf[45]};
                                end else if ({rx_buf[44][3:0],rx_buf[45]} < 12'd4095) begin
                                    mib_obj  <= 4'h2;
                                    mib_vlid <= {rx_buf[44][3:0],rx_buf[45]} + 12'd1;
                                end else begin
                                    mib_obj  <= 4'hF;
                                    mib_vlid <= 12'd0;
                                end
                            end else begin
                                mib_obj  <= rx_buf[43] + 8'd1;
                                mib_vlid <= {rx_buf[44][3:0], rx_buf[45]};
                            end
                        end else begin
                            mib_obj  <= 4'hF;
                            mib_vlid <= 12'd0;
                        end
                        state <= S_MIBWAIT;
                    end else begin
                        state <= S_RX;
                        rx_count <= 8'd0;
                    end
                end

                S_MIBWAIT: begin
                    // 解析阶段已经将GET/GETNEXT最终目标放入mib_obj/mib_vlid。
                    // 本状态只负责向统一MIB桥发起一次读请求。
                    if (mib_obj != 4'hF) begin
                        mib_re   <= 1'b1;
                        mib_addr <= {mib_obj, mib_vlid};
                        state <= S_BUILD;
                    end else begin
                        // 非法OID：本阶段不开放写操作，直接返回空值。
                        mib_value <= 32'd0;
                        rsp_value_type <= 8'h02;
                        rsp_valid <= 1'b1;
                        rsp_oid_len <= req_oid_len;
                        for (i = 0; i < 13; i = i + 1)
                            if (i < req_oid_len) rsp_oid[i] <= req_oid[i];
                        load_tx();
                        tx_index <= 8'd0;
                        udp_tx_src_port <= 16'd161;
                        udp_tx_dst_port <= reply_dst_port;
                        state <= S_TX;
                    end
                end

                S_BUILD: begin
                    if (mib_rvalid) begin
                        // 使用已返回的MIB数据进行响应组包。
                        mib_value = mib_rdata;
                        if (mib_obj == 4'h0) begin
                            set_response_oid(1, 12'd0);
                            rsp_value_type = 8'h43; // TimeTicks
                        end else if (mib_obj == 4'h1) begin
                            set_response_oid(2, 12'd0);
                            rsp_value_type = 8'h02; // INTEGER
                        end else begin
                            set_response_oid(10 + mib_obj - 1, mib_vlid);
                            if (mib_obj == 4'h7)
                                rsp_value_type = 8'h02;
                            else
                                rsp_value_type = 8'h41; // Counter32
                        end
                        load_tx();
                        tx_index <= 8'd0;
                        udp_tx_src_port <= 16'd161;
                        udp_tx_dst_port <= reply_dst_port;
                        state <= S_TX;
                    end
                end

                S_TX: begin
                    if (udp_tx_ready && tx_index < tx_len) begin
                        udp_tx_data  <= tx_buf[tx_index];
                        udp_tx_valid <= 1'b1;
                        udp_tx_last  <= (tx_index == tx_len - 1);
                        if (tx_index == tx_len - 1) begin
                            tx_index <= 8'd0;
                            rx_count <= 8'd0;
                            state <= S_RX;
                        end else begin
                            tx_index <= tx_index + 8'd1;
                        end
                    end
                end

                default: state <= S_RX;
            endcase
        end
    end
endmodule
