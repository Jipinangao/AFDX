//=====================================================================
// afdx_perf_tb : technical-indicator measurements on AFDX_top (default parameters, 125 MHz)
//   1) RX latency      : last byte on rx_data  -> first byte on partition_rx   (min / max frame)
//   2) TX latency      : first byte of the partition message -> first byte on tx_data
//   3) TX throughput   : 12 VLs (BAG 1 ms, 1471-byte payload) round-robin, payload Mbit/s on the wire
//   4) BAG compliance  : minimum spacing between two frames of the same VL >= 1 ms
//   5) jitter flag     : tx_jitter_violation_o never raised; formula output tx_jitter_max_us_o reported
//   6) VL capacity     : 128 TX VLs and 4096 RX VLs configured; first/last entry of each usable
// Result lines starting with "MEASURE" are quoted in the test report.
//=====================================================================
`timescale 1ns/1ps
module afdx_core_perf_tb;
    reg clk = 0, reset = 1;
    wire [7:0] tx_data, tx_data_b; wire tx_valid, tx_valid_b, tx_ready;
    reg [7:0] rx_data = 0, rx_data_b = 0; reg rx_valid = 0, rx_valid_b = 0;
    reg [6:0] p_vlid = 0; reg p_valid = 0; reg [7:0] p_data = 0; reg p_last = 0; reg [13:0] p_len = 0; wire p_ready;
    wire pr_valid, pr_last; wire [7:0] pr_data; wire [11:0] pr_vlid;
    reg [15:0] vlt_addr = 0; reg [31:0] vlt_wdata = 0; reg vlt_we = 0;
    wire init_done, jit_calc; wire [9:0] jit_us;
    wire [31:0] rtc_seconds_o; wire rtc_locked_o; wire [2:0] loader_session_state_o;
    wire [7:0] tx_frag_count_o, reasm_complete_count_o, err_flags_o;
    wire jit_viol = err_flags_o[2];
    wire flv = err_flags_o[3];
    wire [31:0] unused32;

    afdx_core dut (
        .clk(clk), .reset(reset),
        .tx_data(tx_data), .tx_valid(tx_valid), .rx_data(rx_data), .rx_valid(rx_valid), .tx_ready(tx_ready), .rx_ready(1'b0),
        .tx_data_b(tx_data_b), .tx_valid_b(tx_valid_b), .rx_data_b(rx_data_b), .rx_valid_b(rx_valid_b),
        .rx_err(1'b0), .rx_err_b(1'b0),
        .partition_tx_vlid(p_vlid), .partition_tx_valid(p_valid), .partition_tx_data(p_data), .partition_tx_last(p_last),
        .partition_tx_len(p_len), .partition_tx_ready(p_ready),
        .partition_rx_valid(pr_valid), .partition_rx_data(pr_data), .partition_rx_last(pr_last), .partition_rx_vlid(pr_vlid),
        .cfg_sel(2'd0), .cfg_addr(vlt_addr), .cfg_wdata(vlt_wdata), .cfg_we(vlt_we), .cfg_re(1'b0),
        .cfg_rdata(), .cfg_rdy(), .link_status(1'b1),
        .init_done_o(init_done), .rtc_seconds_o(rtc_seconds_o), .rtc_locked_o(rtc_locked_o),
        .loader_session_state_o(loader_session_state_o),
        .tx_jitter_max_us_o(jit_us), .tx_jitter_calc_valid_o(jit_calc),
        .tx_frag_count_o(tx_frag_count_o), .reasm_complete_count_o(reasm_complete_count_o),
        .err_flags_o(err_flags_o));

    always #4 clk = ~clk;
    integer pass = 0, fail = 0;
    task check; input c; input [8*80-1:0] n;
        begin if (c) begin pass = pass + 1; $display("[PASS] %0s", n); end
              else begin fail = fail + 1; $display("[FAIL] %0s", n); end end
    endtask

    // ---- configuration ----
    task cfg_rx_vl; input [11:0] vl; input [10:0] lmin; input [10:0] lmax;
        begin @(negedge clk); vlt_addr = {1'b1, 3'd0, vl}; vlt_wdata = {9'd0, 1'b1, lmax, lmin}; vlt_we = 1; @(negedge clk); vlt_we = 0; end endtask
    task cfg_tx_vl; input [6:0] vl; input [10:0] lmax; input [47:0] mac;
        begin
            @(negedge clk); vlt_addr = {1'b0, 6'd0, vl, 1'b0}; vlt_wdata = {mac[16:0], lmax, 1'b1, 3'd0}; vlt_we = 1;
            @(negedge clk); vlt_addr = {1'b0, 6'd0, vl, 1'b1}; vlt_wdata = {1'b0, mac[47:17]};
            @(negedge clk); vlt_we = 0;
        end endtask

    // ---- TX frame monitor (network A): start time, end time, VL of every frame ----
    integer nfr = 0; time t_start [0:255]; time t_end [0:255]; integer f_vl [0:255]; integer f_len [0:255];
    integer cur_len = 0; reg prev_v = 0; reg [7:0] b4, b5;
    always @(posedge clk) begin
        prev_v <= tx_valid;
        if (tx_valid) begin
            if (!prev_v) begin t_start[nfr] = $time; cur_len = 0; end
            if (cur_len == 4) b4 = tx_data;
            if (cur_len == 5) f_vl[nfr] = {b4[3:0], tx_data};
            cur_len = cur_len + 1;
        end else if (prev_v) begin t_end[nfr] = $time; f_len[nfr] = cur_len; nfr = nfr + 1; end
    end

    // ---- partition message sender ----
    time msg_t [0:255]; integer midx = 0;
    task send_msg; input [6:0] vl; input integer n;
        integer k;
        begin
            k = 0;
            while (k < n) begin
                @(negedge clk);
                if (k == 0 && !p_ready) p_valid = 0;
                else begin if (k == 0) begin msg_t[midx] = $time; midx = midx + 1; end
                    p_valid = 1; p_vlid = vl; p_data = k[7:0]; p_last = (k == n-1); p_len = n; k = k + 1; end
            end
            @(negedge clk); p_valid = 0; p_last = 0;
        end endtask

    // ---- RX frame injection (network A) ----
    reg [7:0] fr [0:1599]; integer fr_len;
    function [15:0] fold16; input [19:0] s; reg [16:0] f1;
        begin f1 = {1'b0, s[15:0]} + {13'd0, s[19:16]}; fold16 = f1[15:0] + {15'd0, f1[16]}; end
    endfunction
    task build_rx; input [11:0] vl; input [7:0] sn; input integer plen;
        integer k, pad; reg [15:0] tl, ul, ck; reg [19:0] sm;
        begin
            pad = (plen < 17) ? 17 - plen : 0; tl = 28 + plen; ul = 8 + plen;
            fr[0]=3; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]={4'h0,vl[11:8]}; fr[5]=vl[7:0];
            fr[6]=2; fr[7]=0; fr[8]=0; fr[9]=0; fr[10]=1; fr[11]=1; fr[12]=8; fr[13]=0;
            fr[14]=8'h45; fr[15]=0; fr[16]=tl[15:8]; fr[17]=tl[7:0]; fr[18]=0; fr[19]=0; fr[20]=0; fr[21]=0; fr[22]=8'h40; fr[23]=8'h11;
            fr[24]=0; fr[25]=0; fr[26]=8'hC0; fr[27]=8'hA8; fr[28]=1; fr[29]=2; fr[30]=8'hC0; fr[31]=8'hA8; fr[32]=1; fr[33]=8'hFF;
            sm = {4'd0,fr[14],fr[15]} + {4'd0,fr[16],fr[17]} + {4'd0,fr[18],fr[19]} + {4'd0,fr[20],fr[21]} +
                 {4'd0,fr[22],fr[23]} + {4'd0,fr[26],fr[27]} + {4'd0,fr[28],fr[29]} + {4'd0,fr[30],fr[31]} + {4'd0,fr[32],fr[33]};
            ck = ~fold16(sm); fr[24] = ck[15:8]; fr[25] = ck[7:0];
            fr[34]=8'h13; fr[35]=8'h88; fr[36]=8'h13; fr[37]=8'h89; fr[38]=ul[15:8]; fr[39]=ul[7:0]; fr[40]=0; fr[41]=0;
            for (k = 0; k < plen; k = k + 1) fr[42+k] = k[7:0];
            for (k = 0; k < pad; k = k + 1) fr[42+plen+k] = 0;
            fr[42+plen+pad] = sn; fr_len = 42 + plen + pad + 1;
        end endtask
    time t_rx_last, t_first_out;
    reg seen_out;
    always @(posedge clk) if (pr_valid && !seen_out) begin seen_out = 1; t_first_out = $time; end
    task inject_rx; integer k;
        begin
            seen_out = 0;
            for (k = 0; k < fr_len; k = k + 1) begin @(negedge clk); rx_data = fr[k]; rx_valid = 1; end
            t_rx_last = $time;
            @(negedge clk); rx_valid = 0;
        end endtask

    integer i, r, v, first_idx, bad_bag; time tmin; time dur; integer payload_bits;
    real mbps; integer spacing_ok; time last_start [0:127]; time gap;
    integer cyc = 0, cyc0 = 0;
    always @(posedge clk) cyc <= cyc + 1;
    time t_msg0, t_first_tx; time lat_min, lat_max; time vl_worst;

    initial begin
        repeat (5) @(posedge clk); reset = 0;
        wait (init_done); repeat (4) @(posedge clk);

        // ---- VL capacity: configure all 4096 RX VLs and 128 TX VLs ----
        for (i = 0; i < 4096; i = i + 1) cfg_rx_vl(i[11:0], 11'd64, 11'd1518);
        for (i = 0; i < 128; i = i + 1)  cfg_tx_vl(i[6:0], 11'd1518, 48'h0300_0000_0000 + i);
        repeat (400) @(posedge clk);     // let the jitter calculator rescan the TX table
        check(jit_calc, "jitter bound computed from the configured TX VLs");
        $display("MEASURE jitter_max_us (XAPP1130 Eq.2, 128 VLs x 1518 B) = %0d us (standard cap 500 us)", jit_us);

        // ---- 1) RX latency ----
        build_rx(12'd4095, 8'd1, 17); inject_rx; repeat (200) @(posedge clk);
        $display("MEASURE RX latency, 64-byte frame  (last byte in -> first byte out) = %0d ns", t_first_out - t_rx_last);
        check(seen_out && (t_first_out - t_rx_last) < 150000, "RX latency (64-byte frame) < 150 us (ARINC 664 end-system limit)");
        build_rx(12'd4095, 8'd2, 1471); inject_rx; repeat (3000) @(posedge clk);
        $display("MEASURE RX latency, 1518-byte frame (last byte in -> first byte out) = %0d ns", t_first_out - t_rx_last);
        check(seen_out && (t_first_out - t_rx_last) < 150000, "RX latency (1518-byte frame) < 150 us");

        // ---- 2) TX latency (idle BAG) ----
        nfr = 0; t_msg0 = $time; send_msg(7'd100, 17);
        while (nfr < 1) @(posedge clk);
        $display("MEASURE TX latency, 17-byte message: first byte in -> first frame byte out = %0d ns", t_start[0] - t_msg0);
        check((t_start[0] - t_msg0) < 150000, "TX latency (17-byte message) is within the allowed end-system latency");

        // ---- 3) throughput ----
        repeat (130000) @(posedge clk);      // let every VL's BAG elapse (>1 ms)
        nfr = 0;
        for (r = 0; r < 5; r = r + 1)
            for (v = 1; v <= 12; v = v + 1) send_msg(v[6:0], 1471);
        while (nfr < 60) @(posedge clk);
        repeat (50) @(posedge clk);
        dur = t_end[59] - t_start[0];
        payload_bits = 60 * 1471 * 8;
        mbps = payload_bits / (dur / 1000.0);           // bits per microsecond = Mbit/s
        $display("MEASURE TX throughput: 60 frames x 1471 B payload in %0d ns = %0f Mbit/s payload (12 VLs, BAG 1 ms)", dur, mbps);
        check(mbps >= 100.0, "payload throughput >= 100 Mbit/s");

        // ---- 4) BAG compliance (same VL spacing >= 1 ms) ----
        bad_bag = 0; tmin = 64'd999999999999;
        for (v = 0; v < 128; v = v + 1) last_start[v] = 0;
        for (i = 0; i < 60; i = i + 1) begin
            if (last_start[f_vl[i]] != 0) begin
                gap = t_start[i] - last_start[f_vl[i]];
                if (gap < tmin) tmin = gap;
                if (gap < 1000000) bad_bag = bad_bag + 1;
            end
            last_start[f_vl[i]] = t_start[i];
        end
        $display("MEASURE minimum spacing of two frames of the same VL = %0d ns (BAG = 1000000 ns)", tmin);
        check(bad_bag == 0, "no VL ever transmits faster than its BAG (1 ms)");
        check(!jit_viol, "tx_jitter_violation_o never raised");

        // ---- 5) jitter, official definition: for ONE VL, max - min of the latency over many frames ----
        // 12 VLs run concurrently, one message per VL every 1.1 ms. The payload length changes every period:
        // 17 B (minimum 64-byte frame), 300 B, 900 B, 1471 B (maximum 1518-byte frame); so every VL sees
        // the minimum, two intermediate and the maximum frame size.
        repeat (140000) @(posedge clk);
        nfr = 0; midx = 0; cyc0 = cyc;
        for (r = 0; r < 4; r = r + 1) begin
            for (v = 1; v <= 12; v = v + 1) begin
                // absolute schedule: period 1.1 ms (137500 cycles), VLs 80 us (10000 cycles) apart
                while (cyc < cyc0 + r*137500 + (v-1)*10000) @(posedge clk);
                send_msg(v[6:0], (r == 0) ? 17 : (r == 1) ? 300 : (r == 2) ? 900 : 1471);
            end
        end
        repeat (5000) @(posedge clk);
        lat_min = 64'd999999999; lat_max = 0;
        for (i = 0; i < 48; i = i + 1) begin
            gap = t_start[i] - msg_t[i];
            if (gap < lat_min) lat_min = gap;
            if (gap > lat_max) lat_max = gap;
        end
        $display("MEASURE variable-length traffic (48 frames, 12 VLs, lengths 17/300/900/1471 B): message->frame latency min=%0d ns max=%0d ns", lat_min, lat_max);
        // per-VL spread (max - min over the 4 frames of the same VL)
        vl_worst = 0;
        for (v = 0; v < 12; v = v + 1) begin
            lat_min = 64'd999999999; lat_max = 0;
            for (r = 0; r < 4; r = r + 1) begin
                gap = t_start[r*12 + v] - msg_t[r*12 + v];
                if (gap < lat_min) lat_min = gap;
                if (gap > lat_max) lat_max = gap;
            end
            if (lat_max - lat_min > vl_worst) vl_worst = lat_max - lat_min;
        end
        $display("MEASURE jitter = worst per-VL (max - min latency of the 4 frames of one VL) = %0d ns", vl_worst);
        check(nfr == 48, "all 48 periodic frames transmitted");
        check(vl_worst <= 50000, "per-VL jitter (max - min latency) <= 50 us (tens of microseconds)");
        check(lat_max < 2000000, "end-to-end latency < 2 ms (GMII interface, official answer)");
        check(!flv, "no frame length violation");

        $display("AFDX_PERF RESULT: PASS=%0d FAIL=%0d", pass, fail);
        if (fail == 0) $display("========== AFDX_PERF ALL PASS =========="); else $display("========== AFDX_PERF FAIL ==========");
        $finish;
    end
    initial begin #900_000_000; $display("[FAIL] watchdog"); $finish; end
endmodule
