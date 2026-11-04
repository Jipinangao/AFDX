//=====================================================================
// afdx_mgmt_plane_tb : management plane at UDP level (SNMP / 615A / RTC)
// The DUT is wired to the real integrity/redundancy and policer modules so that
// SNMP reads return statistics produced by real frame events.
// (End-to-end reachability over Ethernet frames is tested in AFDX_top_tb.)
//=====================================================================
`timescale 1ns/1ps
module afdx_mgmt_plane_tb;
    localparam W = 12;
    reg clk = 0, rst_n = 0;

    // frame events for integrity / policer
    reg a_valid = 0; reg [W-1:0] a_vlid = 0; reg [7:0] a_seq = 0;
    reg pol_valid = 0; reg [W-1:0] pol_vlid = 0; reg [10:0] pol_len = 0;
    reg pol_cfg_we = 0; reg [15:0] pol_cfg_addr = 0; reg [31:0] pol_cfg_wdata = 0;

    // UDP side
    reg [7:0] udp_rx_data = 0; reg udp_rx_valid = 0, udp_rx_last = 0; reg [15:0] udp_rx_src_port = 0, udp_rx_dst_port = 0;
    wire [7:0] udp_tx_data; wire udp_tx_valid, udp_tx_last; wire [15:0] udp_tx_src_port, udp_tx_dst_port;
    reg udp_tx_ready = 1;

    wire int_mib_re, int_mib_rvalid, pol_mib_re, pol_mib_rvalid;
    wire [15:0] int_mib_addr; wire [31:0] int_mib_rdata; wire [11:0] pol_mib_vlid; wire [31:0] pol_mib_drop;
    wire [31:0] rtc_seconds; wire [9:0] rtc_ms; wire rtc_locked; wire [2:0] ld_state; wire [31:0] ld_bytes;
    wire int_init, pol_init;

    afdx_integrity_redundancy #(.NUM_RX_VL(4096), .RX_IDX_W(W)) u_int (
        .clk(clk), .rst_n(rst_n),
        .a_valid(a_valid), .a_vlid(a_vlid), .a_seq_num(a_seq), .a_frame_len(11'd64), .a_error(1'b0),
        .b_valid(1'b0), .b_vlid(12'd0), .b_seq_num(8'd0), .b_frame_len(11'd0), .b_error(1'b0),
        .cfg_we(1'b0), .cfg_addr(16'd0), .cfg_wdata(32'd0),
        .out_valid(), .out_vlid(), .out_seq_num(), .out_frame_len(), .out_network(), .out_error(),
        .sequence_error(), .duplicate_drop(), .event_valid(), .event_vlid(), .event_seq_num(), .event_network(),
        .mib_re(int_mib_re), .mib_addr(int_mib_addr), .mib_rvalid(int_mib_rvalid), .mib_rdata(int_mib_rdata),
        .init_done(int_init));

    afdx_token_bucket_policer #(.NUM_RX_VL(4096), .RX_IDX_W(W), .CLK_FREQ_HZ(125_000_000)) u_pol (
        .clk(clk), .rst_n(rst_n), .ms_tick(), .time_ms(),
        .frame_valid(pol_valid), .frame_vlid(pol_vlid), .frame_len(pol_len), .frame_allow(), .frame_policed(),
        .cfg_we(pol_cfg_we), .cfg_addr(pol_cfg_addr), .cfg_wdata(pol_cfg_wdata),
        .mib_re(pol_mib_re), .mib_vlid(pol_mib_vlid), .mib_rvalid(pol_mib_rvalid), .mib_drop_count(pol_mib_drop),
        .init_done(pol_init));

    afdx_mgmt_plane #(.CLK_FREQ_HZ(125_000_000)) dut (
        .clk(clk), .rst_n(rst_n),
        .udp_rx_data(udp_rx_data), .udp_rx_valid(udp_rx_valid), .udp_rx_last(udp_rx_last),
        .udp_rx_src_port(udp_rx_src_port), .udp_rx_dst_port(udp_rx_dst_port),
        .udp_tx_data(udp_tx_data), .udp_tx_valid(udp_tx_valid), .udp_tx_last(udp_tx_last),
        .udp_tx_src_port(udp_tx_src_port), .udp_tx_dst_port(udp_tx_dst_port), .udp_tx_ready(udp_tx_ready),
        .rtc_cfg_we(1'b0), .rtc_cfg_addr(16'd0), .rtc_cfg_wdata(32'd0),
        .link_status(1'b1), .tx_frames(32'd7), .tx_errors(32'd0),
        .int_mib_re(int_mib_re), .int_mib_addr(int_mib_addr), .int_mib_rvalid(int_mib_rvalid), .int_mib_rdata(int_mib_rdata),
        .pol_mib_re(pol_mib_re), .pol_mib_vlid(pol_mib_vlid), .pol_mib_rvalid(pol_mib_rvalid), .pol_mib_drop_count(pol_mib_drop),
        .rtc_seconds_o(rtc_seconds), .rtc_ms_o(rtc_ms), .rtc_locked_o(rtc_locked),
        .loader_session_state_o(ld_state), .loader_bytes_loaded_o(ld_bytes));

    always #4 clk = ~clk;

    integer pass = 0, fail = 0;
    task check; input c; input [8*72-1:0] n;
        begin if (c) begin pass = pass + 1; $display("[PASS] %0s", n); end
              else begin fail = fail + 1; $display("[FAIL] %0s", n); end end
    endtask

    reg [7:0] req [0:127]; reg [7:0] resp [0:255];
    integer req_len, resp_len, j, req_id = 0;

    // send one UDP message and collect the answer (if any)
    task send_pkt; input [15:0] dport; input expect_resp;
        integer wd;
        begin
            for (j = 0; j < req_len; j = j + 1) begin
                @(negedge clk);
                udp_rx_data = req[j]; udp_rx_valid = 1; udp_rx_last = (j == req_len-1);
                udp_rx_dst_port = dport; udp_rx_src_port = 16'd40000;
            end
            @(negedge clk); udp_rx_valid = 0; udp_rx_last = 0;
            resp_len = 0; wd = 0;
            if (expect_resp) begin
                while (wd < 2000) begin
                    @(posedge clk); #1; wd = wd + 1;
                    if (udp_tx_valid && udp_tx_ready) begin
                        resp[resp_len] = udp_tx_data; resp_len = resp_len + 1;
                        if (udp_tx_last) wd = 99999;
                    end
                end
            end else repeat (10) @(posedge clk);
        end
    endtask

    task build_snmp; input [7:0] pdu; input integer oid_len; input [103:0] oid; // oid[103:96]=first byte
        integer k;
        begin
            req_id = req_id + 1;
            req[0]=8'h30; req[1]=oid_len+33; req[2]=8'h02; req[3]=8'h01; req[4]=8'h00;
            req[5]=8'h04; req[6]=8'h06; req[7]="p"; req[8]="u"; req[9]="b"; req[10]="l"; req[11]="i"; req[12]="c";
            req[13]=pdu; req[14]=oid_len+20; req[15]=8'h02; req[16]=8'h04; req[17]=0; req[18]=0; req[19]=0; req[20]=req_id[7:0];
            req[21]=8'h02; req[22]=8'h01; req[23]=0; req[24]=8'h02; req[25]=8'h01; req[26]=0;
            req[27]=8'h30; req[28]=oid_len+6; req[29]=8'h30; req[30]=oid_len+4; req[31]=8'h06; req[32]=oid_len;
            for (k = 0; k < oid_len; k = k + 1) req[33+k] = oid[103 - 8*k -: 8];
            req[33+oid_len]=8'h05; req[34+oid_len]=8'h00;
            req_len = oid_len + 35;
        end
    endtask

    localparam [103:0] OID_UPTIME = 104'h2B06010401868D1F01010100_00;     // ...1.1.1.0 (12 bytes used)
    function [31:0] val; input integer base; begin val = {resp[base],resp[base+1],resp[base+2],resp[base+3]}; end endfunction

    initial begin
        repeat (5) @(posedge clk); rst_n = 1;
        wait (int_init && pol_init); repeat (2) @(posedge clk);

        // statistics source: one accepted frame on VL1, and one policed frame on VL0 (capacity 32 < 64 bytes)
        @(negedge clk); a_valid = 1; a_vlid = 12'd1; a_seq = 8'd1; @(negedge clk); a_valid = 0;
        @(negedge clk); pol_cfg_we = 1; pol_cfg_addr = {4'd0, 12'd0}; pol_cfg_wdata = 32'd32; @(negedge clk); pol_cfg_we = 0;
        @(negedge clk); pol_cfg_we = 1; pol_cfg_addr = {4'd1, 12'd0}; pol_cfg_wdata = 32'd0;  @(negedge clk); pol_cfg_we = 0; // no refill
        @(negedge clk); pol_cfg_we = 1; pol_cfg_addr = {4'd0, 12'd0}; pol_cfg_wdata = 32'd32; @(negedge clk); pol_cfg_we = 0;
        @(negedge clk); pol_vlid = 0; pol_len = 11'd64; pol_valid = 1; @(negedge clk); pol_valid = 0;
        repeat (10) @(posedge clk);

        // ---- RTC sync over UDP/8319: jump to 500000 s ----
        req[0]=8'h00; req[1]=8'h07; req[2]=8'hA1; req[3]=8'h20; req[4]=8'h00; req[5]=8'h00; req_len = 6;   // 500000 = 0x0007A120
        send_pkt(16'd8319, 0);
        check(rtc_seconds == 32'd500000 && rtc_locked, "RTC sync message aligns RTC to 500000 s and locks");

        // ---- SNMP GET sysUpTime (taken from the RTC uptime base) ----
        build_snmp(8'hA0, 12, {8'h2B,8'h06,8'h01,8'h04,8'h01,8'h86,8'h8D,8'h1F,8'h01,8'h01,8'h01,8'h00,8'h00});
        send_pkt(16'd161, 1);
        check(resp_len == 51 && resp[0] == 8'h30 && resp[13] == 8'hA2, "SNMP GET sysUpTime: well-formed GetResponse");
        check(resp[17]==0 && resp[20]==req_id[7:0], "SNMP response echoes request-id");
        check(val(47) < 32'd100000, "sysUpTime is uptime (ms), not the synchronised absolute time");

        // ---- SNMP GET rxFrames(VL1) : 1.3.6.1.4.1.99999.1.2.1.0.1 ----
        build_snmp(8'hA0, 13, {8'h2B,8'h06,8'h01,8'h04,8'h01,8'h86,8'h8D,8'h1F,8'h01,8'h02,8'h01,8'h00,8'h01});
        send_pkt(16'd161, 1);
        check(resp_len == 52 && resp[13] == 8'hA2 && val(48) == 32'd1, "SNMP GET rxFrames(VL1) = 1 (real integrity statistics)");

        // ---- SNMP GET policingDrops(VL0) ----
        build_snmp(8'hA0, 13, {8'h2B,8'h06,8'h01,8'h04,8'h01,8'h86,8'h8D,8'h1F,8'h01,8'h02,8'h05,8'h00,8'h00});
        send_pkt(16'd161, 1);
        check(resp_len == 52 && val(48) == 32'd1, "SNMP GET policingDrops(VL0) = 1 (real policer statistics)");

        // ---- SNMP GETNEXT rxFrames(VL1) -> rxErrors(VL1) ----
        build_snmp(8'hA1, 13, {8'h2B,8'h06,8'h01,8'h04,8'h01,8'h86,8'h8D,8'h1F,8'h01,8'h02,8'h01,8'h00,8'h01});
        send_pkt(16'd161, 1);
        check(resp_len == 52 && resp[33+10] == 8'h02 && val(48) == 32'd0, "SNMP GETNEXT moves rxFrames -> rxErrors(VL1) = 0");

        // ---- 615A load request, then alternate with SNMP (arbiter must not mix answers) ----
        req[0]=8'h01; req[1]=8'hA5; req[2]=8'hA5; req[3]=8'h00; req[4]=8'h01; req[5]=8'h00; req[6]=8'h08; req_len = 7;
        send_pkt(16'd6150, 1);
        check(resp_len == 1 && resp[0] == 8'h81 && ld_state == 3'd1, "615A LOAD_REQUEST accepted (session OPEN)");
        build_snmp(8'hA0, 12, {8'h2B,8'h06,8'h01,8'h04,8'h01,8'h86,8'h8D,8'h1F,8'h01,8'h01,8'h01,8'h00,8'h00});
        send_pkt(16'd161, 1);
        check(resp_len == 51 && resp[13] == 8'hA2, "interleaved: SNMP answer not corrupted by 615A");
        req[0]=8'h01; req[1]=8'hA5; req[2]=8'hA5; req[3]=8'h00; req[4]=8'h01; req[5]=8'h00; req[6]=8'h08; req_len = 7;
        send_pkt(16'd6150, 1);
        check(resp_len == 1 && resp[0] == 8'h82, "interleaved: second LOAD_REQUEST answered BUSY");

        $display("MGMT-PLANE RESULT: PASS=%0d FAIL=%0d", pass, fail);
        if (fail == 0) $display("========== MGMT-PLANE ALL PASS =========="); else $display("========== MGMT-PLANE FAIL ==========");
        $finish;
    end
endmodule
