// =============================================================================
// Atari RLE motion-object engine ("growth renderer"), Pit-Fighter and Hydra
// =============================================================================
//
// Behaviour follows MAME's atarirle.cpp (Aaron Giles, BSD-3-Clause), with one
// deliberate difference in horizontal scaling that matches the real board
// (see "Horizontal scaling" below). The three microcode PROMs on the board are
// not executed; the algorithm they implement is.
//
// The engine draws the 256-entry object list into one of two framebuffers
// while the other is displayed, as the real hardware draws into its own VRAM.
//
// ---- Object list entry (8 words; MAME modesc_pitfight / modesc_hydra) ----
//   word 0  [15] hflip, [14:0] object code
//   word 1  [7:4] colour
//   word 2  [15:6] X, 10-bit signed
//   word 3  [15:6] Y, 10-bit signed
//   word 4  scale, 4.12 fixed point (0x1000 = 1:1; 0 = not drawn)
//   word 6  [7:0] priority order on Pit-Fighter; word 5 on Hydra
//           (0 = never drawn). Orders are drawn 1..255; within one order,
//           objects are drawn in ascending index.
// Clip window: x 40..295 on Pit-Fighter, 0..255 on Hydra (a 256-pixel strip
// either way, which is the width of the framebuffer), y 0..239. The X
// position is relative to the left edge of the window.
// Pixel value written = palette base + colour + pixel, where the base is 0x200
// aligned down to the object's bit depth (see palettebase below).
//
// ---- RLE data in ROM (16-bit words) ----
// Header for each code at word code*4:
//   +0 x offset (s16)   +1 y offset (s16)
//   +2 [10:8] decode table, [7:0] data offset bits 23:16
//   +3 data offset bits 15:0 (in words)
// Object data is a list of rows. Each row starts with a count word N (if bit
// 15 is set, the count is ~N), followed by N words each holding two chunks,
// low byte first. N = 0 ends the object. Each chunk byte encodes a run length
// and a pixel value (rle_decode below); value 0 is transparent.
//
// ---- Commands ----
// FF2000 (mo_command) holds the command; a rising edge of MOGO (bit 0 of the
// control byte at FA0001) runs it. On Pit-Fighter command 0 is CHECKSUM and
// anything else is DRAW; Hydra always draws. Control byte: bit 0 MOGO, bit 1 ERASE, bit 2 FRAME.
// DRAW renders into buffer ~FRAME while buffer FRAME is displayed; with ERASE
// set, scan-out clears each line of the displayed buffer after reading it.
// CHECKSUM writes the ROM checksums (supplied by the memory hub, which sums
// each 128KB of RLE ROM during the download) into object RAM words 0..n-1,
// where n = object RAM word 0 + 1.
//
// ---- Object sizes (start-up prescan) ----
// Scaling and horizontal flipping need each object's width and height. The
// board's microcode finds the edges as it draws; MAME instead scans the whole
// ROM once at start-up, and so does this engine, after each ROM download:
//   1. count the objects: the lowest data offset in any header, / 4
//      (MAME count_objects);
//   2. for every object, walk its rows: height = rows before the terminating
//      zero count (at most 1023), width = the widest row in pixels
//      (MAME prescan_rle). A header whose data offset lies below its own
//      header or beyond the end of the ROM is an empty object.
// The results go into u_info. `ready` stays low until this is done and the
// board holds both CPUs in reset meanwhile (about 0.2 s after the download).
// The scan survives an OSD reset; it is repeated only after a new download.
// tools/gen_rle_info.py computes the same table offline, for checking.
//
// A note on timing: every block RAM here has a registered output, and all
// addresses are set with non-blocking assignments, so read data appears TWO
// clocks after the state that sets the address. Each read below has explicit
// wait states for that; reading a clock early was the most common bug found
// while bringing this engine up.
// =============================================================================

module rle_objects #(
    parameter int MAX_OBJ_BITS = 11     // 2048 codes (Pit-Fighter uses 1312)
)(
    input  logic        clk,
    input  logic        reset,
    input  logic        is_hydra,       // Hydra; otherwise Pit-Fighter

    // ---- control / command ----
    input  logic [15:0] mo_command,     // FF2000
    input  logic        control_wr,
    input  logic [7:0]  control_in,     // FA0001
    input  logic        rom_loaded,     // level: the ROM download has finished
    input  logic [255:0] checksums,     // 16 x 16-bit sums of the 128KB ROM chunks
    output logic        busy,
    output logic        ready,          // start-up prescan finished (see above)

    // ---- object list RAM, port B (2048 x 16) ----
    output logic [10:0] objram_addr,
    input  logic [15:0] objram_q,
    output logic [15:0] objram_d,
    output logic        objram_we,

    // ---- RLE ROM in SDRAM (word addressed, 1M x 16) ----
    output logic [19:0] rle_rom_waddr,
    output logic        rle_rom_rd,
    input  logic [15:0] rle_rom_data,
    input  logic        rle_rom_ready,

    // ---- framebuffer (block RAM in atarig1_top): 2 buffers x 256 x 240 ----
    // render port: one pixel write per clock
    output logic [16:0] fb_addr,        // {buffer, y[7:0], x[7:0]}
    output logic [15:0] fb_wdata,
    output logic        fb_we,
    // scan-out port: line-buffer fill reads and erase writes
    output logic [16:0] fbs_addr,
    input  logic [15:0] fbs_rdata,
    output logic [15:0] fbs_wdata,
    output logic        fbs_we,

    // ---- display ----
    input  logic [8:0]  hcnt, vcnt,
    output logic [9:0]  scan_pix        // palette index, 0 = transparent
);

    // =========================================================================
    // RLE chunk decode: {run[4:0], value[5:0]}, run = pixel count 1..16
    // =========================================================================
    // The table select from the object header chooses the bit depth. Tables 1,
    // 4 and 6 have a "special" form: a zero low nibble means a transparent run
    // of up to 16.
    function automatic logic [10:0] rle_decode(input logic [7:0] b,
                                               input logic [2:0] tsel);
        logic [4:0] run; logic [5:0] value;
        unique case (tsel)
            3'd0: begin run = {1'b0, b[7:4]} + 5'd1; value = {2'b00, b[3:0]}; end     // 4bpp
            3'd1: if (b[3:0] == 4'h0)
                       begin run = {1'b0, b[7:4]} + 5'd1; value = 6'd0; end            // 5bpp special
                  else begin run = {2'b00, b[7:5]} + 5'd1; value = {1'b0, b[4:0]}; end
            3'd2, 3'd3:
                  begin run = {2'b00, b[7:5]} + 5'd1; value = {1'b0, b[4:0]}; end      // 5bpp
            3'd4, 3'd6:
                  if (b[3:0] == 4'h0)
                       begin run = {1'b0, b[7:4]} + 5'd1; value = 6'd0; end            // 6bpp special
                  else begin run = {3'b000, b[7:6]} + 5'd1; value = b[5:0]; end
            default:
                  begin run = {3'b000, b[7:6]} + 5'd1; value = b[5:0]; end             // 6bpp
        endcase
        return {run, value};
    endfunction

    // =========================================================================
    // Control register and buffer selection
    // =========================================================================
    logic [7:0] control;
    wire        mogo_rise = control_wr && control_in[0] && !control[0];
    wire        erase_en  = control[1];

    // ---- per-game configuration (MAME modesc_pitfight / modesc_hydra) ----
    // Clip window x range (objects live in a 256-pixel strip of the screen),
    // the list word holding the priority order, whether command 0 means
    // CHECKSUM, and the RLE ROM size in words.
    wire signed [15:0] clip_l   = is_hydra ? 16'sd0   : 16'sd40;
    wire signed [15:0] clip_r   = is_hydra ? 16'sd255 : 16'sd295;
    wire        [2:0]  ord_word = is_hydra ? 3'd5 : 3'd6;
    wire               cmd_cs   = !is_hydra && (mo_command == 16'h0000);
    wire        [19:0] rom_end  = is_hydra ? 20'h80000 : 20'hFFFFF;     // count scan limit
    wire        [23:0] rom_len  = is_hydra ? 24'h080000 : 24'h100000;  // ROM size in words
    // As MAME: render into ~FRAME, display FRAME.
    wire        disp_buf  = control[2];
    wire        rend_buf  = ~control[2];

    always_ff @(posedge clk)
        if (reset) control <= 8'h00;
        else if (control_wr) control <= control_in;

    // =========================================================================
    // Object size table: {height[9:0], width[9:0]} per code
    // =========================================================================
    // Written by the prescan (port A), read by the renderer (port B).
    logic [19:0] info_q;
    logic [MAX_OBJ_BITS-1:0] info_raddr, info_waddr;
    logic [19:0] info_wd;
    logic        info_we;
    dpram #(.AW(MAX_OBJ_BITS), .DW(20)) u_info (
        .clk(clk),
        .addr_a(info_waddr), .d_a(info_wd), .we_a(info_we), .q_a(),
        .addr_b(info_raddr), .d_b('0), .we_b(1'b0), .q_b(info_q)
    );

    logic [MAX_OBJ_BITS:0] objectcount;
    logic                  scanned = 1'b0;  // count and prescan complete
    assign ready = scanned;

    // =========================================================================
    // Priority sort: one bucket (linked list) per priority order
    // =========================================================================
    // As MAME's sort_and_render: one pass from object 255 down to 0 prepends
    // each object to the list for its order, which leaves every list in
    // ascending object order; a second pass walks orders 1..255 and draws each
    // list. Entries are 9 bits: bit 8 = valid, bits 7:0 = object number.
    logic [8:0] head_q, next_q;
    logic [7:0] head_addr, next_addr;
    logic [8:0] head_d,   next_d;
    logic       head_we,  next_we;

    dpram #(.AW(8), .DW(9)) u_sort_head (
        .clk(clk),
        .addr_a(head_addr), .d_a(head_d), .we_a(head_we), .q_a(head_q),
        .addr_b(8'd0), .d_b(9'd0), .we_b(1'b0), .q_b()
    );
    dpram #(.AW(8), .DW(9)) u_sort_next (
        .clk(clk),
        .addr_a(next_addr), .d_a(next_d), .we_a(next_we), .q_a(next_q),
        .addr_b(8'd0), .d_b(9'd0), .we_b(1'b0), .q_b()
    );

    // =========================================================================
    // Object list snapshot
    // =========================================================================
    // MAME renders the list as it stands at one instant. Our render takes real
    // time, and the 68000 carries on editing object RAM meanwhile, so at each
    // DRAW the whole list (2048 words) is first copied here and rendered from
    // the copy.
    logic [10:0] snap_waddr, snap_raddr;
    logic [15:0] snap_d, snap_q;
    logic        snap_we;
    logic [11:0] snap_i;
    logic [10:0] sa1, sa2;      // the copy's address, delayed to meet objram_q

    dpram #(.AW(11), .DW(16)) u_objsnap (
        .clk(clk),
        .addr_a(snap_waddr), .d_a(snap_d), .we_a(snap_we), .q_a(),
        .addr_b(snap_raddr), .d_b(16'd0), .we_b(1'b0), .q_b(snap_q)
    );

    logic [7:0] sort_i;         // bucket being cleared
    logic [7:0] cur_order;      // priority order being drawn

    // =========================================================================
    // State machine
    // =========================================================================
    typedef enum logic [5:0] {
        S_IDLE,
        // ---- after the download: count the objects ----
        SB_CNT_RD0, SB_CNT_W0, SB_CNT_RD1, SB_CNT_W1, SB_CNT_NEXT,
        // ---- then measure every object ----
        SP_NEXT, SP_RD2, SP_HDR2, SP_RD3, SP_HDR3, SP_ROW, SP_RDC, SP_CNT,
        SP_ENT, SP_RDE, SP_ADD, SP_WR,
        // ---- checksum ----
        SC_RD_WAIT, SC_RD_REQ, SC_WRITE,
        // ---- draw: snapshot, bucket sort, walk the buckets ----
        SD_OBJ_W,
        SD_SNAP,
        SD_SORT_CLR, SD_SORT_RD, SD_SORT_W, SD_SORT_ORD, SD_SORT_HW, SD_SORT_LINK,
        SD_ORD_RD, SD_ORD_W, SD_ORD_CHK, SD_CHAIN_W, SD_CHAIN_S, SD_CHAIN_RD,
        // ---- draw: one object ----
        SD_OBJ_RD, SD_HDR, SD_WHW, SD_WH, SD_DIVX, SD_DIVY, SD_SETUP, SD_CLIP,
        SD_ROW_SEEK, SD_ROW_CNT, SD_ENTRY, SD_EMIT_LO, SD_EMIT_HI,
        SD_NEXT_LINE, SD_OBJ_DONE
    } state_t;
    state_t st;

    // object count scan
    logic [19:0] scan_ofs, lowest;
    logic [1:0]  rdwait;        // clocks to wait for a new ROM address to take effect
    logic [15:0] hdr_w [0:3];
    logic [1:0]  hdr_idx;
    logic [2:0]  tsel;

    // prescan
    logic [15:0] pv_q;          // the word an SP_RDx state read
    logic [MAX_OBJ_BITS:0] pv_code;
    logic [15:0] pv_w2;         // header word +2
    logic [2:0]  pv_tsel;
    logic [19:0] pv_ptr;
    logic [9:0]  pv_h;
    logic [15:0] pv_w, pv_row, pv_n;
    wire  [10:0] pv_dec_lo = rle_decode(pv_q[7:0],  pv_tsel);   // {run, value}
    wire  [10:0] pv_dec_hi = rle_decode(pv_q[15:8], pv_tsel);

    // checksum
    logic [8:0]  cs_req, cs_idx;
    logic        cs_pend;       // a checksum was requested while the engine was busy

    // draw
    logic [8:0]  objnum;
    logic [15:0] ow [0:7];      // the object's 8 list words
    logic [2:0]  ow_idx;
    logic        hflip;
    logic [3:0]  color;
    logic signed [15:0] xoffs, yoffs;
    logic [15:0] scale;
    logic [9:0]  width, height;
    logic [31:0] scalex;                    // 16.16
    logic [31:0] scaled_w, scaled_h;
    logic [31:0] dy;                        // source rows per output row, 16.16
    logic [31:0] div_n, div_d, div_q, div_r;
    logic [5:0]  div_cnt;
    logic signed [15:0] sx, sy, ex, ey, ex_raw;
    logic [31:0] sourcex, sourcey, rle_end;
    logic [9:0]  cur_row;
    logic [19:0] data_ofs, row_start, row_ptr;
    logic [15:0] entries_left;
    logic signed [15:0] dest_x, dest_y;
    logic [15:0] eword;
    logic [5:0]  run_val;

    // MAME aligns the palette base down to the object's bit depth, so the
    // colour bits can never overlap the pixel bits:
    //   palettebase = (0x200 + color) & ~((1 << bpp) - 1)
    // tsel 0 = 4bpp (align 16), 1-3 = 5bpp (align 32), 4-7 = 6bpp (align 64).
    wire [9:0] pb_unaligned = 10'h200 + {2'b00, color, 4'b0000};
    wire [9:0] palettebase  = (tsel == 3'd0)  ? (pb_unaligned & ~10'h00F)
                            : (tsel <= 3'd3)  ? (pb_unaligned & ~10'h01F)
                                              : (pb_unaligned & ~10'h03F);

    assign busy = (st != S_IDLE);

    always_ff @(posedge clk) begin
        objram_we  <= 1'b0;
        rle_rom_rd <= 1'b0;
        fb_we      <= 1'b0;
        info_we    <= 1'b0;

        // a new download invalidates the object table
        if (!rom_loaded) scanned <= 1'b0;

        if (reset) begin
            st <= S_IDLE;
            cs_pend <= 1'b0;
            if (!scanned) objectcount <= '0;
            snap_we <= 1'b0; snap_i <= 12'd0; sa1 <= 11'd0; sa2 <= 11'd0;
            head_we <= 1'b0; next_we <= 1'b0;
        end else begin
            // A CHECKSUM requested while a draw is in progress must not be lost:
            // the game waits for the result (the self-test's Motion Object Test
            // asks for one while it is drawing). Remember it and run it when the
            // engine is next idle. This sits before the case so that starting
            // the checksum on the same clock (which clears the flag) wins.
            if (mogo_rise && cmd_cs)
                cs_pend <= 1'b1;

            unique case (st)
            // =================================================================
            S_IDLE: begin
                if (rom_loaded && !scanned) begin
                    scan_ofs <= 20'd0;  lowest <= rom_end;
                    st <= SB_CNT_RD0;
                end else if (mogo_rise) begin
                    if (cmd_cs) begin
                        objram_addr <= 11'd0;  cs_pend <= 1'b0;  st <= SC_RD_WAIT;
                    end else begin
                        snap_i <= 12'd0;
                        st <= SD_SNAP;
                    end
                end else if (cs_pend) begin
                    cs_pend  <= 1'b0;
                    objram_addr <= 11'd0;
                    st <= SC_RD_WAIT;
                end
            end

            // =================================================================
            // Object count: the lowest data offset in any header, / 4
            // =================================================================
            // For each header read words +2 and +3. rle_rom_waddr is registered,
            // so each read waits (rdwait) before trusting rle_rom_ready, which
            // could otherwise still belong to the previous address.
            SB_CNT_RD0: begin
                if (scan_ofs >= lowest) begin
                    objectcount <= (MAX_OBJ_BITS+1)'(lowest >> 2);
                    scan_ofs <= 20'd0;
                    hdr_idx <= 2'd0;
                    pv_code <= '0;
                    st <= SP_NEXT;
                end else begin
                    rle_rom_waddr <= scan_ofs + 20'd2; rle_rom_rd <= 1'b1;
                    rdwait <= 2'd2;
                    st <= SB_CNT_W0;
                end
            end
            SB_CNT_W0: begin
                rle_rom_rd <= 1'b1;
                if (rdwait != 2'd0) rdwait <= rdwait - 2'd1;
                else if (rle_rom_ready) begin hdr_w[2] <= rle_rom_data; st <= SB_CNT_RD1; end
            end
            SB_CNT_RD1: begin
                rle_rom_waddr <= scan_ofs + 20'd3; rle_rom_rd <= 1'b1;
                rdwait <= 2'd2;
                st <= SB_CNT_W1;
            end
            SB_CNT_W1: begin
                rle_rom_rd <= 1'b1;
                if (rdwait != 2'd0) rdwait <= rdwait - 2'd1;
                else if (rle_rom_ready) begin
                    // offset = ((w2 & 0xff) << 16) | w3
                    if ({hdr_w[2][7:0], rle_rom_data} > {4'd0, scan_ofs} &&
                        {hdr_w[2][7:0], rle_rom_data} < {4'd0, lowest})
                        lowest <= 20'({hdr_w[2][7:0], rle_rom_data});
                    st <= SB_CNT_NEXT;
                end
            end
            SB_CNT_NEXT: begin scan_ofs <= scan_ofs + 20'd4; st <= SB_CNT_RD0; end

            // =================================================================
            // Prescan: width and height of every object (MAME prescan_rle)
            // =================================================================
            // Each read: the state before sets rle_rom_waddr and rdwait, an
            // SP_RDx state waits for the word (into pv_q), and the state after
            // uses it. (One wait state per read site, so that the state
            // register is only ever loaded with constants: a "return state"
            // register would stop Quartus treating it as a plain state machine.)
            SP_RD2: begin
                rle_rom_rd <= 1'b1;
                if (rdwait != 2'd0) rdwait <= rdwait - 2'd1;
                else if (rle_rom_ready) begin pv_q <= rle_rom_data; st <= SP_HDR2; end
            end
            SP_RD3: begin
                rle_rom_rd <= 1'b1;
                if (rdwait != 2'd0) rdwait <= rdwait - 2'd1;
                else if (rle_rom_ready) begin pv_q <= rle_rom_data; st <= SP_HDR3; end
            end
            SP_RDC: begin
                rle_rom_rd <= 1'b1;
                if (rdwait != 2'd0) rdwait <= rdwait - 2'd1;
                else if (rle_rom_ready) begin pv_q <= rle_rom_data; st <= SP_CNT; end
            end
            SP_RDE: begin
                rle_rom_rd <= 1'b1;
                if (rdwait != 2'd0) rdwait <= rdwait - 2'd1;
                else if (rle_rom_ready) begin pv_q <= rle_rom_data; st <= SP_ADD; end
            end

            SP_NEXT: begin
                if (pv_code >= objectcount || pv_code[MAX_OBJ_BITS]) begin
                    scanned <= 1'b1;                  // all measured
                    st <= S_IDLE;
                end else begin
                    rle_rom_waddr <= 20'({pv_code, 2'b10}); rle_rom_rd <= 1'b1;
                    rdwait <= 2'd2; st <= SP_RD2;
                end
            end
            SP_HDR2: begin
                pv_w2 <= pv_q;
                rle_rom_waddr <= 20'({pv_code, 2'b11}); rle_rom_rd <= 1'b1;
                rdwait <= 2'd2; st <= SP_RD3;
            end
            SP_HDR3: begin
                pv_tsel <= pv_w2[10:8];
                pv_h <= 10'd0;  pv_w <= 16'd0;
                pv_ptr <= 20'({pv_w2[7:0], pv_q});
                // data offset below its own header, or past the ROM: empty
                if ({pv_w2[7:0], pv_q} < 24'({pv_code, 2'b00}) ||
                    {pv_w2[7:0], pv_q} >= rom_len)
                    st <= SP_WR;
                else
                    st <= SP_ROW;
            end
            // each row: its entry count (bit 15 set = inverted), 0 = end
            SP_ROW: begin
                if (pv_h == 10'd1023) st <= SP_WR;
                else begin
                    rle_rom_waddr <= pv_ptr; rle_rom_rd <= 1'b1;
                    rdwait <= 2'd2; st <= SP_RDC;
                end
            end
            SP_CNT: begin
                if ((pv_q[15] ? ~pv_q : pv_q) == 16'd0) st <= SP_WR;
                else begin
                    pv_n   <= pv_q[15] ? ~pv_q : pv_q;
                    pv_row <= 16'd0;
                    pv_ptr <= pv_ptr + 20'd1;
                    st <= SP_ENT;
                end
            end
            // each entry: two chunks, add their run lengths to the row width
            SP_ENT: begin
                rle_rom_waddr <= pv_ptr; rle_rom_rd <= 1'b1;
                rdwait <= 2'd2; st <= SP_RDE;
            end
            SP_ADD: begin
                automatic logic [15:0] rw = pv_row
                    + {11'd0, pv_dec_lo[10:6]} + {11'd0, pv_dec_hi[10:6]};
                pv_ptr <= pv_ptr + 20'd1;
                pv_n   <= pv_n - 16'd1;
                if (pv_n == 16'd1) begin              // row complete
                    if (rw > pv_w) pv_w <= rw;
                    pv_h <= pv_h + 10'd1;
                    st <= SP_ROW;
                end else begin
                    pv_row <= rw;
                    st <= SP_ENT;
                end
            end
            SP_WR: begin
                info_waddr <= pv_code[MAX_OBJ_BITS-1:0];
                info_wd    <= {pv_h, pv_w[9:0]};
                info_we    <= 1'b1;
                pv_code    <= pv_code + 1'b1;
                st <= SP_NEXT;
            end

            // =================================================================
            // CHECKSUM
            // =================================================================
            // objram_addr was set to 0 on entry; wait one clock for the RAM to
            // see it, then read the requested count on the next.
            SC_RD_WAIT: st <= SC_RD_REQ;

            SC_RD_REQ: begin
                cs_req <= (objram_q[7:0] == 8'hFF) ? 9'd256
                                                   : {1'b0, objram_q[7:0]} + 9'd1;
                cs_idx <= 9'd0;
                st <= SC_WRITE;
            end
            SC_WRITE: begin
                objram_addr <= 11'(cs_idx);
                objram_d    <= (cs_idx < 9'd16) ? checksums[{cs_idx[3:0], 4'd0} +: 16] : 16'h0000;
                objram_we   <= 1'b1;
                if (cs_idx + 9'd1 >= cs_req) st <= S_IDLE;
                else cs_idx <= cs_idx + 9'd1;
            end

            // =================================================================
            // DRAW 1: copy object RAM into the snapshot, one word per clock
            // =================================================================
            // objram_q lags its address by two clocks, so sa1/sa2 carry the
            // address along to meet the data.
            SD_SNAP: begin
                objram_addr <= snap_i[10:0];
                sa1 <= snap_i[10:0];
                sa2 <= sa1;
                if (snap_i >= 12'd2) begin
                    snap_waddr <= sa2;
                    snap_d     <= objram_q;
                    snap_we    <= 1'b1;
                end
                if (snap_i == 12'd2049) begin
                    sort_i <= 8'd0;
                    st     <= SD_SORT_CLR;
                end else snap_i <= snap_i + 12'd1;
            end

            // =================================================================
            // DRAW 2: empty every bucket
            // =================================================================
            SD_SORT_CLR: begin
                snap_we <= 1'b0;      // the copy's last write lands this clock
                head_addr <= sort_i;  head_d <= 9'd0;  head_we <= 1'b1;
                if (sort_i == 8'd255) begin
                    objnum <= 9'd255;
                    st <= SD_SORT_RD;
                end else sort_i <= sort_i + 8'd1;
            end

            // =================================================================
            // DRAW 3: put each object (255 down to 0) at the head of its bucket
            // =================================================================
            SD_SORT_RD: begin
                head_we <= 1'b0;  next_we <= 1'b0;  snap_we <= 1'b0;
                snap_raddr <= {objnum[7:0], ord_word};    // priority order (word 6 / word 5)
                st <= SD_SORT_W;
            end
            SD_SORT_W:   st <= SD_SORT_ORD;           // snap_q settling
            SD_SORT_ORD: begin
                head_addr <= snap_q[7:0];
                st <= SD_SORT_HW;
            end
            SD_SORT_HW:  st <= SD_SORT_LINK;          // head_q settling
            SD_SORT_LINK: begin
                // next[objnum] = head[order];  head[order] = objnum
                next_addr <= objnum[7:0];  next_d <= head_q;      next_we <= 1'b1;
                head_d    <= {1'b1, objnum[7:0]};                 head_we <= 1'b1;
                if (objnum == 9'd0) begin
                    cur_order <= 8'd1;
                    st <= SD_ORD_RD;
                end else begin
                    objnum <= objnum - 9'd1;
                    st <= SD_SORT_RD;
                end
            end

            // =================================================================
            // DRAW 4: walk the buckets in order 1..255
            // =================================================================
            SD_ORD_RD: begin
                head_we <= 1'b0;  next_we <= 1'b0;
                head_addr <= cur_order;
                st <= SD_ORD_W;
            end
            SD_ORD_W: st <= SD_ORD_CHK;               // head_q settling
            SD_ORD_CHK: begin
                if (head_q[8]) begin
                    objnum <= {1'b0, head_q[7:0]};
                    ow_idx <= 3'd0;
                    snap_raddr <= {head_q[7:0], 3'd0};
                    st <= SD_OBJ_W;                   // draw this object
                end else if (cur_order == 8'd255) begin
                    st <= S_IDLE;                     // all done
                end else begin
                    cur_order <= cur_order + 8'd1;
                    st <= SD_ORD_RD;
                end
            end

            // after drawing an object, follow next[] along its bucket
            SD_CHAIN_W: begin
                next_addr <= objnum[7:0];
                st <= SD_CHAIN_S;
            end
            SD_CHAIN_S: st <= SD_CHAIN_RD;            // next_q settling
            SD_CHAIN_RD: begin
                if (next_q[8]) begin
                    objnum <= {1'b0, next_q[7:0]};
                    ow_idx <= 3'd0;
                    snap_raddr <= {next_q[7:0], 3'd0};
                    st <= SD_OBJ_W;
                end else if (cur_order == 8'd255) begin
                    st <= S_IDLE;
                end else begin
                    cur_order <= cur_order + 8'd1;
                    st <= SD_ORD_RD;
                end
            end

            SD_OBJ_DONE: st <= SD_CHAIN_W;

            // =================================================================
            // DRAW 5: one object
            // =================================================================
            // Read the 8 list words from the snapshot. The read address leads
            // the capture by one word to allow for the RAM's latency.
            SD_OBJ_W: begin
                snap_raddr <= {objnum[7:0], 3'd1};
                st <= SD_OBJ_RD;
            end

            SD_OBJ_RD: begin
                ow[ow_idx] <= snap_q;
                if (ow_idx == 3'd7) begin
                    rdwait <= 2'd2; st <= SD_HDR; hdr_idx <= 2'd0;
                    hflip <= ow[0][15];
                end else begin
                    ow_idx <= ow_idx + 3'd1;
                    snap_raddr <= {objnum[7:0], ow_idx + 3'd2};
                end
            end

            // Skip disabled objects (scale 0) and invalid codes; otherwise
            // fetch the object's 4-word header from ROM.
            SD_HDR: begin
                color <= ow[1][7:4];
                scale <= ow[4];
                if (ow[4] == 16'd0 || {1'b0, ow[0][14:0]} >= objectcount[MAX_OBJ_BITS:0]) begin
                    st <= SD_OBJ_DONE;
                end else begin
                    rle_rom_waddr <= {ow[0][14:0], 2'b00} + 20'(hdr_idx);
                    rle_rom_rd <= 1'b1;
                    if (rdwait != 2'd0) rdwait <= rdwait - 2'd1;
                    else if (rle_rom_ready) begin
                        hdr_w[hdr_idx] <= rle_rom_data;
                        rdwait <= 2'd2;               // next header word: wait again
                        if (hdr_idx == 2'd3) begin
                            xoffs <= $signed(hdr_w[0]);
                            yoffs <= $signed(hdr_w[1]);
                            tsel  <= hdr_w[2][10:8];
                            data_ofs <= 20'({hdr_w[2][7:0], rle_rom_data});
                            info_raddr <= MAX_OBJ_BITS'(ow[0][14:0]);
                            st <= SD_WHW;
                        end else hdr_idx <= hdr_idx + 2'd1;
                    end
                end
            end

            // width and height from the size table (two clocks of latency)
            SD_WHW: st <= SD_WH;
            SD_WH: begin
                width  <= info_q[9:0];
                height <= info_q[19:10];
                scalex <= {16'd0, scale} << 4;
                st <= SD_DIVX;
            end

            // scaled_w = (scalex * width + 0x8000) >> 16, minimum 1.
            // The 32-step divide that follows computed MAME's horizontal step
            // (width << 16) / scaled_w. The hardware-style scaler below no longer
            // needs that step, but the divide is kept so the engine's timing -
            // and so which frame each render completes in - is unchanged.
            SD_DIVX: begin
                if (div_cnt == 6'd0) begin
                    automatic logic [31:0] sw =
                        (scalex * {22'd0, width} + 32'h8000) >> 16;
                    scaled_w <= (sw == 0) ? 32'd1 : sw;
                    div_n <= {6'd0, width, 16'd0};
                    div_d <= (sw == 0) ? 32'd1 : sw;
                    div_q <= '0; div_r <= '0;
                    div_cnt <= 6'd32;
                end else begin
                    automatic logic [31:0] r = {div_r[30:0], div_n[31]};
                    div_n <= {div_n[30:0], 1'b0};
                    if (r >= div_d) begin
                        div_r <= r - div_d;
                        div_q <= {div_q[30:0], 1'b1};
                    end else begin
                        div_r <= r;
                        div_q <= {div_q[30:0], 1'b0};
                    end
                    if (div_cnt == 6'd1) begin
                        div_cnt <= 6'd0;
                        st <= SD_DIVY;
                    end else div_cnt <= div_cnt - 6'd1;
                end
            end

            // scaled_h likewise, and dy = (height << 16) / scaled_h: the source
            // row step per output row (restoring divide, 32 steps).
            SD_DIVY: begin
                if (div_cnt == 6'd0) begin
                    automatic logic [31:0] sh =
                        (scalex * {22'd0, height} + 32'h8000) >> 16;
                    scaled_h <= (sh == 0) ? 32'd1 : sh;
                    div_n <= {6'd0, height, 16'd0};
                    div_d <= (sh == 0) ? 32'd1 : sh;
                    div_q <= '0; div_r <= '0;
                    div_cnt <= 6'd32;
                end else begin
                    automatic logic [31:0] r = {div_r[30:0], div_n[31]};
                    div_n <= {div_n[30:0], 1'b0};
                    if (r >= div_d) begin
                        div_r <= r - div_d;
                        div_q <= {div_q[30:0], 1'b1};
                    end else begin
                        div_r <= r;
                        div_q <= {div_q[30:0], 1'b0};
                    end
                    if (div_cnt == 6'd1) begin
                        dy <= {div_q[30:0], (r >= div_d)};
                        div_cnt <= 6'd0;
                        st <= SD_SETUP;
                    end else div_cnt <= div_cnt - 6'd1;
                end
            end

            // Screen position: sign-extend X/Y, add the 40-pixel left clip, and
            // subtract the scaled header offsets. Signed 32-bit arithmetic as
            // MAME:
            //   scaled_xoffs = (scale * xoffs) >> 12
            //   if (hflip) scaled_xoffs = ((scale * width) >> 12) - scaled_xoffs
            //   scaled_yoffs = (scale * yoffs) >> 12
            // (scale must be treated as signed and the product kept at 32 bits,
            // or negative offsets lose their sign and large ones wrap.)
            SD_SETUP: begin
                automatic logic signed [15:0] px =
                    $signed({{6{ow[2][15]}}, ow[2][15:6]}) + clip_l;
                automatic logic signed [15:0] py =
                    $signed({{6{ow[3][15]}}, ow[3][15:6]});
                automatic logic signed [31:0] sc  = $signed({16'd0, scale});
                automatic logic signed [31:0] xo  = 32'(xoffs);
                automatic logic signed [31:0] yo  = 32'(yoffs);
                automatic logic signed [31:0] wd  = $signed({22'd0, width});
                automatic logic signed [31:0] sxo_f = (sc * xo) >>> 12;
                automatic logic signed [31:0] syo_f = (sc * yo) >>> 12;
                automatic logic signed [31:0] swd_f = (sc * wd) >>> 12;
                automatic logic signed [31:0] sxo =
                    hflip ? (swd_f - sxo_f) : sxo_f;
                sx <= px - 16'(sxo);
                sy <= py - 16'(syo_f);
                st <= SD_CLIP;
            end

            // Reject objects wholly outside the clip window. Horizontal
            // clipping of partly visible objects happens per pixel when they are
            // written; at the top edge, sourcey is advanced to the first visible
            // row.
            SD_CLIP: begin
                automatic logic signed [15:0] exv = sx + 16'(scaled_w) - 16'sd1;
                automatic logic signed [15:0] eyv = sy + 16'(scaled_h) - 16'sd1;
                automatic logic [31:0] syrc = dy >> 1;
                automatic logic signed [15:0] syv = sy;
                if (sx > clip_r || exv < clip_l ||
                    sy > 16'sd239 || eyv < 16'sd0) begin
                    st <= SD_OBJ_DONE;
                end else begin
                    ex     <= (exv > clip_r) ? clip_r : exv;
                    ex_raw <= exv;
                    if (sy < 16'sd0) begin
                        syrc = syrc + (32'(-sy)) * dy;
                        syv  = 16'sd0;
                    end
                    ey <= (eyv > 16'sd239) ? 16'sd239 : eyv;
                    sourcey   <= syrc;
                    dest_y    <= syv;
                    cur_row   <= 10'd0;
                    row_start <= data_ofs;
                    rdwait <= 2'd2; st <= SD_ROW_SEEK;
                end
            end

            // ---- each output line: step through the rows to the source row ----
            SD_ROW_SEEK: begin
                if (cur_row == 10'(sourcey >> 16)) begin
                    row_ptr <= row_start; rdwait <= 2'd2; st <= SD_ROW_CNT;
                end else begin
                    // skip a row: row_start += 1 + its count
                    rle_rom_waddr <= row_start; rle_rom_rd <= 1'b1;
                    if (rdwait != 2'd0) rdwait <= rdwait - 2'd1;
                    else if (rle_rom_ready) begin
                        automatic logic [15:0] c =
                            rle_rom_data[15] ? ~rle_rom_data : rle_rom_data;
                        row_start <= row_start + 20'd1 + 20'(c);
                        cur_row   <= cur_row + 10'd1;
                    end
                end
            end

            // ---- read the row's entry count and start the line ----
            // Horizontal scaling, done the way the board does it (not MAME's
            // way): source pixels are written to the current destination
            // address and a rate multiplier decides when that address moves on.
            // So the LAST source pixel to reach an address wins, and a
            // transparent run leaves an earlier opaque pixel standing. MAME
            // instead samples the source at fixed steps and can drop thin detail
            // the real board keeps (MAMEtesters 08884 / 6022).
            // Here rle_end accumulates the destination span of each run
            // (run x scalex) and sourcex counts output pixels.
            SD_ROW_CNT: begin
                rle_rom_waddr <= row_ptr; rle_rom_rd <= 1'b1;
                if (rdwait != 2'd0) rdwait <= rdwait - 2'd1;
                else if (rle_rom_ready) begin
                    entries_left <= rle_rom_data[15] ? ~rle_rom_data
                                                     : rle_rom_data;
                    row_ptr  <= row_ptr + 20'd1;
                    sourcex  <= 32'd0;
                    rle_end  <= 32'd0;
                    dest_x   <= hflip ? ex_raw : sx;
                    rdwait <= 2'd2; st <= SD_ENTRY;
                end
            end

            // ---- fetch an entry word (two chunks) ----
            SD_ENTRY: begin
                if (entries_left == 16'd0) st <= SD_NEXT_LINE;
                else begin
                    rle_rom_waddr <= row_ptr; rle_rom_rd <= 1'b1;
                    if (rdwait != 2'd0) rdwait <= rdwait - 2'd1;
                    else if (rle_rom_ready) begin
                        automatic logic [10:0] dlo =
                            rle_decode(rle_rom_data[7:0], tsel);
                        eword <= rle_rom_data;
                        row_ptr <= row_ptr + 20'd1;
                        entries_left <= entries_left - 16'd1;
                        run_val <= dlo[5:0];
                        rle_end <= rle_end + 32'(dlo[10:6] * scalex[19:0]);
                        st <= SD_EMIT_LO;
                    end
                end
            end

            // ---- write the low chunk's pixels, then the high chunk's ----
            SD_EMIT_LO, SD_EMIT_HI: begin
                if (sourcex < rle_end) begin
                    if (run_val != 6'd0 &&
                        dest_x >= clip_l && dest_x <= clip_r &&
                        dest_y >= 16'sd0  && dest_y <= 16'sd239) begin
                        fb_addr  <= {rend_buf, dest_y[7:0], 8'(dest_x - clip_l)};
                        fb_wdata <= {6'd0, palettebase + {4'd0, run_val}};
                        fb_we    <= 1'b1;
                    end
                    dest_x  <= hflip ? dest_x - 16'sd1 : dest_x + 16'sd1;
                    sourcex <= sourcex + 32'h0001_0000;
                end else if (st == SD_EMIT_LO) begin
                    automatic logic [10:0] dhi = rle_decode(eword[15:8], tsel);
                    run_val <= dhi[5:0];
                    rle_end <= rle_end + 32'(dhi[10:6] * scalex[19:0]);
                    st <= SD_EMIT_HI;
                end else begin rdwait <= 2'd2; st <= SD_ENTRY; end
            end

            SD_NEXT_LINE: begin
                sourcey <= sourcey + dy;
                dest_y  <= dest_y + 16'sd1;
                if (dest_y >= ey) st <= SD_OBJ_DONE;
                else begin rdwait <= 2'd2; st <= SD_ROW_SEEK; end
            end

            default: st <= S_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Scan-out through a line buffer
    // =========================================================================
    // Each line, from hcnt 304 onwards, the NEXT line of the displayed buffer
    // is copied into the line buffer (256 pixels, three clocks each, well
    // inside the window) and, with ERASE set, cleared in the framebuffer
    // behind the copy. The display reads the line buffer at
    // (hcnt - clip_l - 1): column c of the strip is shown while the counters
    // are at clip_l + c + 1, which is when the tilemaps show the same screen
    // column (checked against MAME with unscaled Hydra objects; reading at
    // hcnt - clip_l put every object one pixel to the left). By the time the
    // copy starts (hcnt 304) the beam has passed the whole strip, so it never
    // reads a pixel being overwritten. lb_armed allows exactly one copy per
    // line. Line 0 is fetched during the last line of vertical blanking
    // (vcnt 261); lines 1-239 during the line before.
    logic [7:0]  lb_fetch_x, lb_y;
    logic        lb_fetching;
    logic [1:0]  lb_ph;         // 0: set address, 1: RAM settling, 2: store
    logic        lb_armed;
    logic [7:0]  lb_wa;  logic [15:0] lb_wd;  logic lb_we;
    logic [15:0] lb_q;
    dpram #(.AW(8), .DW(16)) u_linebuf (
        .clk(clk),
        .addr_a(lb_wa), .d_a(lb_wd), .we_a(lb_we), .q_a(),
        .addr_b(8'(hcnt - 9'(clip_l) - 9'd1)), .d_b('0), .we_b(1'b0), .q_b(lb_q)
    );

    always_ff @(posedge clk) begin
        if (hcnt < 9'd304) lb_armed <= 1'b1;
        if ((hcnt >= 9'd304) && lb_armed && !lb_fetching &&
            (vcnt < 9'd239 || vcnt == 9'd261)) begin
            lb_fetching <= 1'b1; lb_fetch_x <= 8'd0; lb_ph <= 2'd0;
            lb_armed    <= 1'b0;
            lb_y        <= (vcnt == 9'd261) ? 8'd0 : vcnt[7:0] + 8'd1;
        end
        fbs_we <= 1'b0;
        lb_we  <= 1'b0;
        if (lb_fetching) begin
            case (lb_ph)
                2'd0: begin
                    fbs_addr <= {disp_buf, lb_y, lb_fetch_x};
                    lb_ph <= 2'd1;
                end
                2'd1: lb_ph <= 2'd2;
                default: begin
                    lb_wa <= lb_fetch_x;  lb_wd <= fbs_rdata;  lb_we <= 1'b1;
                    if (erase_en) begin fbs_wdata <= 16'd0; fbs_we <= 1'b1; end
                    lb_ph <= 2'd0;
                    if (lb_fetch_x == 8'd255) lb_fetching <= 1'b0;
                    else lb_fetch_x <= lb_fetch_x + 8'd1;
                end
            endcase
        end
    end

    // Only the 256-pixel strip of the clip window can hold motion objects.
    logic in_strip_q;
    always_ff @(posedge clk)
        in_strip_q <= ($signed({7'd0, hcnt}) >  clip_l) && ($signed({7'd0, hcnt}) <= clip_r + 16'sd1);
    assign scan_pix = in_strip_q ? lb_q[9:0] : 10'd0;

endmodule
