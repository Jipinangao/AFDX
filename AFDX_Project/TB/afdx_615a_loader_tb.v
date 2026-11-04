//=====================================================================
// afdx_615a_loader_tb
// 覆盖：
//   1) LOAD_REQUEST: 正确硬件ID -> ACCEPTED；错误硬件ID -> REFUSED；
//      会话进行中重复请求 -> BUSY
//   2) LOAD_DATA: 正常分块加载(含CRC16校验)全部成功，落盘内容正确
//   3) LOAD_DATA: CRC错误块被检测并计数，不落盘、不推进序号
//   4) LOAD_DATA: 乱序块被检测为SEQ_ERR
//   5) LOAD_COMPLETE_QUERY: 全部数据加载完毕后返回COMPLETE状态及总字节数
//   6) UPLOAD_REQUEST: 从已加载区域回读，内容与写入一致(卸载路径)
//=====================================================================
`timescale 1ns/1ps
module afdx_615a_loader_tb;
    reg clk, rst_n;
    reg [7:0] udp_rx_data;
    reg udp_rx_valid, udp_rx_last;
    reg [15:0] udp_rx_src_port, udp_rx_dst_port;
    wire [7:0] udp_tx_data;
    wire udp_tx_valid, udp_tx_last;
    wire [15:0] udp_tx_src_port, udp_tx_dst_port;
    reg udp_tx_ready;

    wire [2:0] session_state;
    wire [31:0] bytes_loaded, image_len, load_crc_errors, load_seq_errors;
    wire [31:0] load_complete_count, load_failed_count, upload_count;

    integer pass_count, fail_count;
    reg [7:0] req[0:255];
    reg [7:0] resp[0:255];
    integer req_len, resp_len, j;

    localparam [31:0] MY_HW_ID = 32'hA5A5_0001;

    afdx_615a_loader #(
        .MEM_DEPTH(4096), .TARGET_HW_ID(MY_HW_ID), .UDP_LOAD_PORT(16'd6150),
        .MAX_BLOCK_LEN(64), .MAX_UPLOAD_LEN(64)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .udp_rx_data(udp_rx_data), .udp_rx_valid(udp_rx_valid), .udp_rx_last(udp_rx_last),
        .udp_rx_src_port(udp_rx_src_port), .udp_rx_dst_port(udp_rx_dst_port),
        .udp_tx_data(udp_tx_data), .udp_tx_valid(udp_tx_valid), .udp_tx_last(udp_tx_last),
        .udp_tx_src_port(udp_tx_src_port), .udp_tx_dst_port(udp_tx_dst_port), .udp_tx_ready(udp_tx_ready),
        .session_state(session_state), .bytes_loaded(bytes_loaded), .image_len(image_len),
        .load_crc_errors(load_crc_errors), .load_seq_errors(load_seq_errors),
        .load_complete_count(load_complete_count), .load_failed_count(load_failed_count),
        .upload_count(upload_count)
    );

    initial begin clk=0; forever #5 clk=~clk; end

    task automatic check;
        input cond; input [8*128-1:0] name;
        begin
            if (cond) begin $display("[PASS] %0s", name); pass_count=pass_count+1; end
            else begin $display("[FAIL] %0s", name); fail_count=fail_count+1; end
        end
    endtask

    // CRC16-CCITT(poly=0x1021, init=0xFFFF)，与DUT算法保持一致，
    // 用于测试平台独立计算期望值(不复用DUT内部function，避免"self-check")。
    function [15:0] crc16_ref;
        input [8*64-1:0] data; // 最多64字节，按大端打包
        input integer len;
        integer b, k;
        reg [15:0] c;
        reg [7:0] byte_v;
        begin
            c = 16'hFFFF;
            for (b = 0; b < len; b = b + 1) begin
                byte_v = data[8*(len-1-b) +: 8];
                c = c ^ {byte_v, 8'h00};
                for (k = 0; k < 8; k = k + 1) begin
                    if (c[15]) c = (c << 1) ^ 16'h1021;
                    else       c = (c << 1);
                end
            end
            crc16_ref = c;
        end
    endfunction

    task automatic send_and_recv;
        begin
            j = 0;
            while (j < req_len) begin
                @(negedge clk);
                udp_rx_data = req[j]; udp_rx_valid = 1; udp_rx_last = (j == req_len-1);
                udp_rx_dst_port = 16'd6150; udp_rx_src_port = 16'd40000;
                @(posedge clk);
                j = j + 1;
            end
            @(negedge clk); udp_rx_valid = 0; udp_rx_last = 0;
            resp_len = 0;
            begin : wl
                integer wd; wd = 0;
                while (1) begin
                    @(posedge clk); #1;
                    wd = wd + 1;
                    if (wd > 500) begin $display("WATCHDOG in send_and_recv"); disable wl; end
                    if (udp_tx_valid && udp_tx_ready) begin
                        resp[resp_len] = udp_tx_data;
                        resp_len = resp_len + 1;
                        if (udp_tx_last) disable wl;
                    end
                end
            end
        end
    endtask

    task automatic build_load_request;
        input [31:0] hw_id;
        input [15:0] img_len;
        begin
            req[0]=8'h01;
            req[1]=hw_id[31:24]; req[2]=hw_id[23:16]; req[3]=hw_id[15:8]; req[4]=hw_id[7:0];
            req[5]=img_len[15:8]; req[6]=img_len[7:0];
            req_len = 7;
        end
    endtask

    task automatic build_load_data;
        input [15:0] seq;
        input [7:0] len8;
        input [15:0] crc_override; // 若为16'hFFFF_FFFF不适用；用CRC_BAD标志单独控制
        input bad_crc;
        integer k;
        reg [15:0] crc_calc;
        reg [8*64-1:0] packed_data;
        begin
            req[0]=8'h02;
            req[1]=seq[15:8]; req[2]=seq[7:0]; req[3]=len8;
            packed_data = 0;
            for (k=0;k<len8;k=k+1) begin
                req[4+k] = 8'h30 + k[7:0] + seq[7:0]; // 简单的可辨识测试数据
                packed_data[8*(len8-1-k) +: 8] = req[4+k];
            end
            crc_calc = crc16_ref(packed_data, len8);
            if (bad_crc) crc_calc = crc_calc ^ 16'hFFFF;
            req[4+len8]   = crc_calc[15:8];
            req[4+len8+1] = crc_calc[7:0];
            req_len = 4 + len8 + 2;
        end
    endtask

    task automatic build_complete_query;
        begin
            req[0] = 8'h03;
            req_len = 1;
        end
    endtask

    task automatic build_upload_request;
        input [15:0] offset;
        input [15:0] len;
        begin
            req[0]=8'h10;
            req[1]=offset[15:8]; req[2]=offset[7:0];
            req[3]=len[15:8]; req[4]=len[7:0];
            req_len = 5;
        end
    endtask

    initial begin
        pass_count=0; fail_count=0;
        rst_n=0; udp_rx_data=0; udp_rx_valid=0; udp_rx_last=0;
        udp_rx_src_port=0; udp_rx_dst_port=0; udp_tx_ready=1;
        repeat(5) @(posedge clk); rst_n=1;

        // ---- 1a) 错误硬件ID -> REFUSED ----
        build_load_request(32'hDEAD_BEEF, 16'd20);
        send_and_recv;
        check(resp_len==1 && resp[0]==8'h83, "wrong hardware ID -> LOAD_REQUEST_REFUSED");
        check(session_state == 3'd0, "session state stays IDLE after a refusal");

        // ---- 1b) 正确硬件ID -> ACCEPTED ----
        build_load_request(MY_HW_ID, 16'd20); // 镜像总长度20字节(2个块: 8+12)
        send_and_recv;
        check(resp_len==1 && resp[0]==8'h81, "correct hardware ID -> LOAD_REQUEST_ACCEPTED");
        check(session_state == 3'd1, "session state is OPEN(1) after acceptance");

        // ---- 1c) 会话进行中重复LOAD_REQUEST -> BUSY ----
        build_load_request(MY_HW_ID, 16'd99);
        send_and_recv;
        check(resp_len==1 && resp[0]==8'h82, "new request during a session -> LOAD_REQUEST_BUSY");
        check(image_len == 32'd20, "BUSY answer did not disturb image_len of the running session");

        // ---- 2) 正常分块加载: block0(8B, seq0), block1(12B, seq1) ----
        build_load_data(16'd0, 8'd8, 16'd0, 1'b0);
        send_and_recv;
        check(resp_len==4 && resp[0]==8'h84 && resp[1]==8'h00 && resp[2]==8'h00 && resp[3]==8'h00,
              "block0 loaded -> LOAD_DATA_ACK(OK)");
        check(bytes_loaded == 32'd8, "bytes_loaded=8 after block0");

        // ---- 3) CRC错误块(seq1, 故意错误CRC): 应报CRC_ERR, 不落盘、不推进 ----
        build_load_data(16'd1, 8'd12, 16'd0, 1'b1);
        send_and_recv;
        check(resp[0]==8'h84 && resp[1]==8'h00 && resp[2]==8'h01 && resp[3]==8'h01,
              "block1 with bad CRC -> LOAD_DATA_ACK(CRC_ERR=1)");
        check(bytes_loaded == 32'd8, "CRC-error block does not advance bytes_loaded (still 8)");
        check(load_crc_errors == 32'd1, "load_crc_errors counter correct");

        // ---- 4) 乱序块(声明seq5, 期望seq仍为1) -> SEQ_ERR ----
        build_load_data(16'd5, 8'd4, 16'd0, 1'b0);
        send_and_recv;
        check(resp[3]==8'h02, "out-of-order block (seq5) -> LOAD_DATA_ACK(SEQ_ERR=2)");
        check(load_seq_errors == 32'd1, "load_seq_errors counter correct");
        check(bytes_loaded == 32'd8, "out-of-order block is not stored and does not advance bytes_loaded");

        // ---- 正确的block1(12B, seq1) 补发 ----
        build_load_data(16'd1, 8'd12, 16'd0, 1'b0);
        send_and_recv;
        check(resp[3]==8'h00, "resent block1 (good CRC/sequence) -> OK");
        check(bytes_loaded == 32'd20, "bytes_loaded=20 after block1, image_len reached");
        check(session_state == 3'd2, "session state COMPLETE(2) once the declared length is reached");

        // ---- 5) LOAD_COMPLETE_QUERY ----
        build_complete_query;
        send_and_recv;
        check(resp_len==6 && resp[0]==8'h85 && resp[1]==8'h00, "COMPLETE_QUERY -> status = success(0)");
        check({resp[2],resp[3],resp[4],resp[5]}==32'd20, "COMPLETE_QUERY -> total bytes = 20");
        check(load_complete_count == 32'd1, "load_complete_count incremented");

        // ---- 6) UPLOAD_REQUEST: 回读前8字节，核对与block0写入内容一致 ----
        build_upload_request(16'd0, 16'd8);
        send_and_recv;
        check(resp_len==13 && resp[0]==8'h90, "UPLOAD_DATA response header correct");
        check({resp[1],resp[2]}==16'd0 && {resp[3],resp[4]}==16'd8, "UPLOAD response echoes offset/length");
        check(resp[5]==8'h30 && resp[6]==8'h31 && resp[7]==8'h32 && resp[12]==8'h37,
              "UPLOAD read-back equals the data written in block0 (unload path)");
        check(upload_count == 32'd1, "upload_count incremented");

        // ---- 6b) 回读block1区域(偏移8, 12字节)，核对内容 ----
        build_upload_request(16'd8, 16'd12);
        send_and_recv;
        check(resp[5]==(8'h30+8'd0+8'd1) && resp[16]==(8'h30+8'd11+8'd1),
              "UPLOAD read-back of the block1 area correct");

        $display("============================================");
        $display("615A-LOADER RESULT: PASS=%0d FAIL=%0d", pass_count, fail_count);
        if (fail_count==0) $display("========== 615A-LOADER ALL PASS ==========");
        else $display("========== 615A-LOADER FAIL ==========");
        $display("============================================");
        $finish;
    end
endmodule
