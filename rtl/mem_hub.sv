// =============================================================================
// Memory hub: ROM download and SDRAM arbitration
// =============================================================================
//
// Sits between the HPS download stream, the core's ROM clients and channel 3
// of sdram.sv (16-bit read/write, request pulse, one-clock ready pulse).
//
// SDRAM holds the ROM image exactly as the MRA streams it (byte addresses
// 0x000000-0x3805FF; see rom_map.svh), except that the wrapper interleaves the
// two playfield bitplanes as they arrive so one burst returns both.
//
// Word format: the stream is written as {byte[2n+1], byte[2n]} = {MSB, LSB}.
// With the MRA's interleave that gives
//   program : the 68000's {even, odd} byte pair directly
//   RLE     : the word directly
//   tiles   : the tilemap engine wants {byte[a], byte[a+1]}, so its data is
//             byte-swapped here
//   OKI     : one byte, chosen by address bit 0 (0 = low byte)
//
// During the download every byte is also copied to the core (dl_*) so the
// character ROM and the 6502 ROM block RAMs fill at the same time, and the
// 16-bit word sum of each 128KB chunk of RLE ROM is accumulated for the RLE
// engine's checksum command.
//
// SDRAM read sample point: after each download the hub writes a test
// pattern to a spare corner of SDRAM and reads it back with each of sdram.sv's
// eight read sample points (half a clock apart), then uses the middle of the
// run that read it back correctly (see "Read calibration" below). rom_loaded
// rises when that is done.
//
// Arbitration: one SDRAM transaction at a time, in fixed priority
//   calibration > download > program > playfield > RLE > OKI.
// Clients hold their request line high, often across a run of different
// addresses, so each client is locked out from repeating the address it has
// just been served until its address changes or its request drops. Without
// that lock the hub re-issued a request on the clock after completing it
// (before the client had seen ready), and the duplicate's late completion
// answered the client's NEXT request with stale data.
// =============================================================================

module mem_hub (
    input  logic        clk,
    input  logic        reset,

    // ---- HPS download ----
    input  logic        ioctl_download,
    input  logic        ioctl_wr,
    input  logic [26:0] ioctl_addr,
    input  logic [7:0]  ioctl_dout,
    input  logic [15:0] ioctl_index,
    output logic        ioctl_wait,

    output logic        rom_loaded,
    output logic [255:0] checksums,     // 16 x 16-bit, chunk N at [N*16 +: 16]

    // SDRAM read sample point (sdram.sv ch3_rdsel)
    input  logic [3:0]  rdsel_force,    // 0 = calibrate; n = always use point n-1
    output logic [2:0]  rdsel,
    output logic [7:0]  cal_mask,       // the points that read the pattern back

    // byte copy to the core's block-RAM ROMs
    output logic        dl_wr,
    output logic [24:0] dl_addr,
    output logic [7:0]  dl_data,

    // ---- core clients ----
    input  logic [24:0] prog_addr,   input logic prog_rd,        // byte address
    output logic [15:0] prog_data,   output logic prog_ready,

    input  logic [19:0] pfrom_addr,  input logic pfrom_rd,       // byte in the tile region
    output logic [15:0] pfrom_data,  output logic pfrom_ready,
    output logic [15:0] pfrom_data2,                             // next word of the burst

    input  logic [19:0] rle_waddr,   input logic rle_rd,         // word in the RLE region
    output logic [15:0] rle_data,    output logic rle_ready,

    input  logic [17:0] oki_addr,    input logic oki_rd,         // byte in the OKI region
    output logic [7:0]  oki_data,    output logic oki_ready,     // a level, see below

    // ---- sdram.sv channel 3 ----
    output logic [26:1] ch3_addr,
    output logic [15:0] ch3_din,
    output logic [1:0]  ch3_be,
    output logic        ch3_req,
    output logic        ch3_rnw,
    input  logic [15:0] ch3_dout,
    input  logic [15:0] ch3_dout2,      // second word of the burst, valid a clock later
    input  logic        ch3_ready
);

    localparam [24:0] TILES_BASE = 25'h0E0000;
    localparam [24:0] RLE_BASE   = 25'h180000;
    localparam [24:0] OKI_BASE   = 25'h090000;

    // =========================================================================
    // Download: pair bytes into word writes, copy bytes to the core, checksum
    // =========================================================================
    logic [7:0]  dl_lo;
    logic        dl_word_pend;          // a word is waiting to be written
    logic [23:0] dl_word_addr;
    logic [15:0] dl_word;
    logic        dl_write_done;
    logic        pf_grab2;
    logic        downloading;

    assign downloading = ioctl_download && (ioctl_index == 16'd0);
    assign ioctl_wait  = dl_word_pend;      // hold the HPS until the word is in SDRAM

    always_ff @(posedge clk) begin
        dl_wr <= 1'b0;
        if (downloading && ioctl_wr) begin
            dl_wr   <= 1'b1;
            dl_addr <= ioctl_addr[24:0];
            dl_data <= ioctl_dout;

            if (!ioctl_addr[0]) begin
                dl_lo <= ioctl_dout;
            end else begin
                dl_word      <= {ioctl_dout, dl_lo};
                dl_word_addr <= ioctl_addr[24:1];
                dl_word_pend <= 1'b1;
                // RLE checksums: word sum per 128KB chunk
                if (ioctl_addr[24:0] >= RLE_BASE
                 && ioctl_addr[24:0] <  RLE_BASE + 25'h200000) begin
                    checksums[{4'((ioctl_addr[24:0] - RLE_BASE) >> 17), 4'd0} +: 16]
                        <= checksums[{4'((ioctl_addr[24:0] - RLE_BASE) >> 17), 4'd0} +: 16]
                         + {ioctl_dout, dl_lo};
                end
            end
        end
        if (dl_write_done) dl_word_pend <= 1'b0;

        if (reset) begin
            dl_word_pend <= 1'b0;
            rom_loaded   <= 1'b0;
            checksums    <= '0;
        end else if (downloading)  rom_loaded <= 1'b0;   // a new ROM is arriving
        else if (cal_finish)       rom_loaded <= 1'b1;   // after read calibration
    end

    logic downloading_q;
    always_ff @(posedge clk) downloading_q <= downloading;
    wire rom_load_end = downloading_q && !downloading;

    // =========================================================================
    // Arbiter
    // =========================================================================
    typedef enum logic [1:0] { H_IDLE, H_WAIT } hstate_t;
    hstate_t hs;

    typedef enum logic [2:0] { C_DL, C_PROG, C_PF, C_RLE, C_OKI, C_CAL } client_t;
    client_t cur;

    // =========================================================================
    // Read calibration
    // =========================================================================
    // Pattern: 8 words at word address CAL_WADDR (byte 0x3E0000, past every
    // ROM image), each differing from the next in many bits. Each read checks
    // the addressed word and the burst's next word (ch3_dout2), so offsets
    // 3 and 7 are skipped (a 4-word burst wraps there). Each point is tried 16
    // times over; a single wrong word fails it.
    localparam [23:0] CAL_WADDR = 24'h1F_0000;
    function automatic logic [15:0] pat(input logic [2:0] i);
        case (i)
            3'd0: return 16'h0000;  3'd1: return 16'hFFFF;
            3'd2: return 16'h5555;  3'd3: return 16'hAAAA;
            3'd4: return 16'h3333;  3'd5: return 16'hCCCC;
            3'd6: return 16'h0F0F;  default: return 16'hF0F0;
        endcase
    endfunction

    typedef enum logic [2:0] { CP_WRITE, CP_SETTLE, CP_READ, CP_CHOOSE, CP_IDLE } calphase_t;
    calphase_t   cal_ph = CP_IDLE;
    logic        cal_req;             // the next calibration access is wanted
    logic [2:0]  cal_i;               // pattern offset
    logic [2:0]  cal_pt;              // sample point under test
    logic [3:0]  cal_rep;
    logic [4:0]  cal_wait;
    logic        cal_ok;              // this point has read everything right so far
    logic        cal_chk;             // compare on the clock after ready (dout2)
    logic [15:0] cal_d1;
    logic        cal_finish;          // one-clock pulse: calibration done
    logic [2:0]  rdsel_auto = 3'd3;   // until calibrated

    // Choose the middle of the longest run of passing points; of the two
    // middle points of an even run, the falling-edge (odd) one. With the
    // inverted SDRAM clock each word's window is centred near a falling edge
    // of this clock, so a rising-edge point that passes may sit close to a
    // word change. Nothing passing: point 3 (3.5T), the likeliest centre.
    logic [2:0] cal_choice;
    logic [3:0] best_len, best_start, run_len, run_start, m;
    always_comb begin
        best_len = 4'd0;  best_start = 4'd0;  run_len = 4'd0;  run_start = 4'd0;  m = 4'd0;
        for (int k = 0; k < 8; k++) begin
            if (cal_mask[k]) begin
                if (run_len == 4'd0) run_start = 4'(k);
                run_len = run_len + 4'd1;
                if (run_len > best_len) begin best_len = run_len; best_start = run_start; end
            end else run_len = 4'd0;
        end
        if (best_len == 4'd0)   cal_choice = 3'd3;
        else if (best_len[0])   cal_choice = 3'(best_start + (best_len >> 1));
        else begin
            m = best_start + (best_len >> 1) - 4'd1;
            cal_choice = m[0] ? 3'(m) : 3'(m + 4'd1);
        end
    end

    // Calibration always sweeps every point (so cal_mask means the same
    // whether or not a point is forced); afterwards the OSD can force one.
    wire [2:0] rdsel_want = (cal_ph != CP_IDLE)   ? cal_pt
                          : (rdsel_force != 4'd0) ? 3'(rdsel_force - 4'd1) : rdsel_auto;

    // The point may only change with no read in flight: sdram.sv takes ready
    // from a tap of its read delay line that depends on the point, so moving
    // it under a read could lose that read's ready (the hub would wait for
    // ever) or give it a second one. When the wanted point differs, the hub
    // issues nothing until it has been idle for 15 clocks (a read's token
    // leaves the delay line within 9), then switches.
    logic       rs_change;
    logic [3:0] rs_quiet;
    assign rs_change = (rdsel_want != rdsel);
    always_ff @(posedge clk) begin
        if (reset) begin
            rdsel <= 3'd3;  rs_quiet <= 4'd0;
        end else begin
            if (hs != H_IDLE || !rs_change) rs_quiet <= 4'd0;
            else if (rs_quiet != 4'd15)     rs_quiet <= rs_quiet + 4'd1;
            if (rs_change && rs_quiet == 4'd15) rdsel <= rdsel_want;
        end
    end

    wire cal_active = (cal_ph != CP_IDLE);
    wire cal_issue    = (hs == H_IDLE) && !rs_change && cal_active && cal_req;
    wire cal_done_acc = (hs == H_WAIT) && ch3_ready && (cur == C_CAL);
    // this read's two words were right (and all before it at this point)
    wire cal_ok_next  = cal_ok && (cal_d1 == pat(cal_i)) && (ch3_dout2 == pat(cal_i + 3'd1));

    always_ff @(posedge clk) begin
        cal_finish <= 1'b0;
        cal_chk    <= 1'b0;
        if (reset) begin
            cal_ph <= CP_IDLE;  cal_req <= 1'b0;  cal_mask <= 8'd0;  rdsel_auto <= 3'd3;
        end else if (rom_load_end) begin
            // a new ROM is in SDRAM: write the pattern, then try every point
            cal_ph <= CP_WRITE;  cal_i <= 3'd0;  cal_pt <= 3'd0;  cal_req <= 1'b1;
        end else begin
            if (cal_issue) cal_req <= 1'b0;
            unique case (cal_ph)
                CP_WRITE: if (cal_done_acc) begin
                    if (cal_i == 3'd7) begin
                        cal_ph <= CP_SETTLE;  cal_wait <= 5'd31;  cal_mask <= 8'd0;
                    end else begin
                        cal_i <= cal_i + 3'd1;  cal_req <= 1'b1;
                    end
                end
                // After changing the point, let any read still in sdram.sv's
                // delay line drain before the next read.
                CP_SETTLE: if (cal_wait != 5'd0) cal_wait <= cal_wait - 5'd1;
                    else begin
                        cal_ph <= CP_READ;  cal_i <= 3'd0;  cal_rep <= 4'd0;
                        cal_ok <= 1'b1;     cal_req <= 1'b1;
                    end
                CP_READ: begin
                    if (cal_done_acc) cal_chk <= 1'b1;
                    if (cal_chk) begin
                        cal_ok <= cal_ok_next;
                        if (cal_i == 3'd6) begin                // offsets 0,1,2,4,5,6
                            if (cal_rep == 4'd15) begin
                                cal_mask[cal_pt] <= cal_ok_next;
                                if (cal_pt == 3'd7) cal_ph <= CP_CHOOSE;
                                else begin
                                    cal_pt <= cal_pt + 3'd1;
                                    cal_ph <= CP_SETTLE;  cal_wait <= 5'd31;
                                end
                            end else begin
                                cal_rep <= cal_rep + 4'd1;  cal_i <= 3'd0;  cal_req <= 1'b1;
                            end
                        end else begin
                            cal_i   <= (cal_i == 3'd2) ? 3'd4 : cal_i + 3'd1;
                            cal_req <= 1'b1;
                        end
                    end
                end
                CP_CHOOSE: begin
                    rdsel_auto <= cal_choice;
                    cal_ph     <= CP_IDLE;
                    cal_finish <= 1'b1;
                end
                default: ;
            endcase
        end
    end

    // per client: locked out, address issued, address last served
    logic prog_lock, pf_lock, rle_lock, oki_lock, dl_lock;
    logic [24:1] prog_iss, prog_srv;
    logic [19:0] pf_iss,   pf_srv;
    logic [19:0] rle_iss,  rle_srv;
    logic [17:0] oki_iss,  oki_srv;
    logic [23:0] dl_iss,   dl_srv;
    logic        oki_byte_sel;

    always_ff @(posedge clk) begin
        ch3_req       <= 1'b0;
        prog_ready    <= 1'b0;
        pfrom_ready   <= 1'b0;
        rle_ready     <= 1'b0;
        dl_write_done <= 1'b0;

        if (!prog_rd      || prog_addr[24:1] != prog_srv) prog_lock <= 1'b0;
        if (!pfrom_rd     || pfrom_addr      != pf_srv)   pf_lock   <= 1'b0;
        if (!rle_rd       || rle_waddr       != rle_srv)  rle_lock  <= 1'b0;
        if (!oki_rd       || oki_addr        != oki_srv)  oki_lock  <= 1'b0;
        if (!dl_word_pend || dl_word_addr    != dl_srv)   dl_lock   <= 1'b0;

        // OKI ready is a LEVEL: high for as long as oki_data belongs to the
        // address being asked for. jt6295 samples it in one particular time
        // slot of its own schedule, which a one-clock pulse would usually miss.
        oki_ready <= oki_lock && oki_rd && (oki_addr == oki_srv);

        // the playfield's second word arrives the clock after ready
        if (pf_grab2) pfrom_data2 <= {ch3_dout2[7:0], ch3_dout2[15:8]};
        pf_grab2 <= 1'b0;

        if (reset) begin
            hs <= H_IDLE; pf_grab2 <= 1'b0; pfrom_data2 <= 16'd0;
            prog_lock <= 1'b0; pf_lock <= 1'b0; rle_lock <= 1'b0;
            oki_lock  <= 1'b0; dl_lock <= 1'b0;
        end else begin
            unique case (hs)
            H_IDLE: begin
                if (rs_change) begin
                    // changing the read point: wait (see rdsel above)
                end else if (cal_active) begin
                    if (cal_req) begin
                        cur <= C_CAL;
                        ch3_addr <= {2'd0, CAL_WADDR + {21'd0, cal_i}};
                        ch3_din  <= pat(cal_i);  ch3_be <= 2'b11;
                        ch3_rnw  <= (cal_ph != CP_WRITE);
                        ch3_req  <= 1'b1;  hs <= H_WAIT;
                    end
                end else if (dl_word_pend && !dl_lock) begin
                    cur <= C_DL;  dl_iss <= dl_word_addr;
                    ch3_addr <= {2'd0, dl_word_addr};
                    ch3_din  <= dl_word;  ch3_be <= 2'b11; ch3_rnw <= 1'b0;
                    ch3_req  <= 1'b1;  hs <= H_WAIT;
                end else if (prog_rd && !prog_lock) begin
                    cur <= C_PROG;  prog_iss <= prog_addr[24:1];
                    ch3_addr <= {2'd0, prog_addr[24:1]};
                    ch3_rnw  <= 1'b1;  ch3_req <= 1'b1;  hs <= H_WAIT;
                end else if (pfrom_rd && !pf_lock) begin
                    cur <= C_PF;  pf_iss <= pfrom_addr;
                    ch3_addr <= {2'd0, (TILES_BASE + {5'd0, pfrom_addr}) >> 1};
                    ch3_rnw  <= 1'b1;  ch3_req <= 1'b1;  hs <= H_WAIT;
                end else if (rle_rd && !rle_lock) begin
                    cur <= C_RLE;  rle_iss <= rle_waddr;
                    ch3_addr <= {2'd0, (RLE_BASE >> 1) + {4'd0, rle_waddr}};
                    ch3_rnw  <= 1'b1;  ch3_req <= 1'b1;  hs <= H_WAIT;
                end else if (oki_rd && !oki_lock) begin
                    cur <= C_OKI;  oki_iss <= oki_addr;
                    oki_byte_sel <= oki_addr[0];
                    ch3_addr <= {2'd0, (OKI_BASE + {7'd0, oki_addr}) >> 1};
                    ch3_rnw  <= 1'b1;  ch3_req <= 1'b1;  hs <= H_WAIT;
                end
            end

            H_WAIT: if (ch3_ready) begin
                unique case (cur)
                    C_DL:   begin dl_write_done <= 1'b1;
                                  dl_lock <= 1'b1; dl_srv <= dl_iss; end
                    C_PROG: begin prog_data  <= ch3_dout;
                                  prog_ready <= 1'b1;
                                  prog_lock  <= 1'b1; prog_srv <= prog_iss; end
                    C_PF:   begin pfrom_data <= {ch3_dout[7:0], ch3_dout[15:8]};
                                  pf_grab2   <= 1'b1;
                                  pfrom_ready<= 1'b1;
                                  pf_lock    <= 1'b1; pf_srv <= pf_iss; end
                    C_RLE:  begin rle_data   <= ch3_dout;
                                  rle_ready  <= 1'b1;
                                  rle_lock   <= 1'b1; rle_srv <= rle_iss; end
                    C_OKI:  begin oki_data   <= oki_byte_sel ? ch3_dout[15:8]
                                                             : ch3_dout[7:0];
                                  // oki_ready follows from the lock next clock
                                  oki_lock   <= 1'b1; oki_srv <= oki_iss; end
                    C_CAL:  cal_d1 <= ch3_dout;     // the calibration FSM takes it from here
                    default: ;
                endcase
                hs <= H_IDLE;
            end

            default: hs <= H_IDLE;
            endcase
        end
    end

endmodule
