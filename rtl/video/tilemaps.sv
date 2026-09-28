// =============================================================================
// Atari G1 playfield and alphanumerics layers
// =============================================================================
//
// Behaviour follows MAME's atarig1_v.cpp and its gfx layouts (Aaron Giles,
// BSD-3-Clause).
//
// ALPHANUMERICS: 64 x 32 tiles of 8 x 8, 4bpp, packed MSB first. 4096
// characters (128KB); Pit-Fighter fits only the first 2048.
//   RAM word: [11:0] code, [15:12] colour; bit 15 also marks the tile opaque
//   (MAME's TILE_FORCE_LAYER0: pen 0 is drawn rather than transparent, so
//   opaque tiles use colours 8-15). Character ROM: 32 bytes per tile, 4 per
//   row, two pixels per byte, high nibble first. Palette = 0x100 + colour*16
//   + pen.
//
// PLAYFIELD: 64 x 64 tiles of 8 x 8, 5bpp.
//   RAM word: [11:0] code, [14:12] colour, [15] X flip.
//   Tile = (bank[1:0] << 12) | code (16384 tiles). Palette = 0x300 +
//   colour*32 + pen.
//   Scroll: each scanline has a pair of words in the ALPHANUMERICS RAM at
//   (line/8)*64 + 48 + (line&7)*2:
//     word 0, if bit 15 set: X scroll = ((word >> 6) + offset) & 0x1FF
//                            (offset 2 on Pit-Fighter, 0 on Hydra)
//     word 1, if bit 15 set: this line shows source row (word >> 6) & 0x1FF,
//                            tile bank = word & 7
//   A scroll value stays in force until another word with bit 15 set changes
//   it. Pit-Fighter sets its scroll only in scanline 0's slot.
//   Graphics for tile row y (byte addresses in the tile ROM region):
//     A  = bytes [code*16 + y*2], [+1]
//     B  = bytes [0x40000 + code*16 + y*2], [+1]
//     P4 = byte  [0x80000 + code*8 + y]
//   Pixel x (0-7), low 4 bits, high nibble first:
//     x0 B0[7:4]  x1 B0[3:0]  x2 A0[7:4]  x3 A0[3:0]
//     x4 B1[7:4]  x5 B1[3:0]  x6 A1[7:4]  x7 A1[3:0]
//   bit 4 = P4[7-x]. X flip reads pixel 7-x.
//   The wrapper stores planes A and B interleaved in SDRAM so that one burst
//   returns both (pfrom_data and pfrom_data2).
//
// HOW IT RUNS: each line is drawn into a line buffer during the line before
// it is displayed (line N+1 is prepared while line N is shown; line 0 during
// the last vblank line), then read out at the pixel clock. A whole line is
// available because each playfield tile needs two SDRAM round trips (A+B
// together, then P4), which would not fit in horizontal blanking alone.
//
// Every RAM here has a registered output, so data arrives two clocks after
// the state that sets the address; each read has explicit wait states.
// =============================================================================

module tilemaps (
    input  logic        clk,
    input  logic        reset,
    input  logic        cen_pix,
    input  logic        is_hydra,       // Hydra; otherwise Pit-Fighter

    input  logic [8:0]  hcnt,
    input  logic [8:0]  vcnt,

    // RAM port B reads (registered)
    output logic [11:0] pf_vram_addr,
    input  logic [15:0] pf_vram_q,
    output logic [10:0] al_vram_addr,
    input  logic [15:0] al_vram_q,

    // playfield graphics ROM in SDRAM (byte address in the tile region;
    // data returned as {byte[addr], byte[addr+1]})
    output logic [19:0] pfrom_addr,
    output logic        pfrom_rd,
    input  logic [15:0] pfrom_data,
    input  logic [15:0] pfrom_data2,   // plane B: the next word of the same burst
    input  logic        pfrom_ready,

    // alphanumerics character ROM (128KB block RAM, registered read)
    output logic [16:0] chrom_addr,
    input  logic [7:0]  chrom_q,

    // per-pixel outputs, registered on the pixel enable
    output logic [4:0]  pf_pen,
    output logic [2:0]  pf_color,
    output logic [3:0]  al_pen,
    output logic [3:0]  al_color,
    output logic        al_opaque
);

    // =================================================================
    // Line buffers: two lines each (written / displayed alternate by
    // line). Explicit dprams so Quartus maps them to block RAM.
    // =================================================================
    wire wr_buf = ~vcnt[0];
    wire rd_buf =  vcnt[0];

    logic [9:0] pflb_wa;  logic [7:0] pflb_wd;  logic pflb_we;
    logic [9:0] allb_wa;  logic [8:0] allb_wd;  logic allb_we;
    logic [7:0] pflb_q;   logic [8:0] allb_q;

    dpram #(.AW(10), .DW(8)) u_pflb (
        .clk(clk),
        .addr_a(pflb_wa), .d_a(pflb_wd), .we_a(pflb_we), .q_a(),
        .addr_b({rd_buf, (hcnt < 9'd336) ? hcnt : 9'd0}),
        .d_b('0), .we_b(1'b0), .q_b(pflb_q)
    );
    dpram #(.AW(10), .DW(9)) u_allb (
        .clk(clk),
        .addr_a(allb_wa), .d_a(allb_wd), .we_a(allb_we), .q_a(),
        .addr_b({rd_buf, (hcnt < 9'd336) ? hcnt : 9'd0}),
        .d_b('0), .we_b(1'b0), .q_b(allb_q)
    );

    // =================================================================
    // Scroll state
    // =================================================================
    logic [8:0] hscroll, src_row;
    // As MAME, the vertical scroll is kept as an offset (scrolly) and the
    // source row is (line + scrolly), so with no scroll writes a still image
    // stays still.
    logic [8:0] scrolly;
    logic [2:0] tile_bank;
    logic [8:0] next_line;      // the line being prepared
    // pfrom_addr is registered, so pfrom_ready is ignored for two clocks after
    // a new address (it could still belong to the previous one).
    logic [1:0] pf_wait;

    // =================================================================
    // Render FSM
    // =================================================================
    typedef enum logic [4:0] {
        R_IDLE, R_SCR0A, R_SCR0W, R_SCR0B, R_SCR1W, R_SCR1B,
        R_PF_MAP, R_PF_MAPW, R_PF_MAPQ, R_PF_A, R_PF_BW, R_PF_BQ, R_PF_P4, R_PF_EMIT,
        R_AL_MAP, R_AL_MAPW, R_AL_MAPQ, R_AL_ROWW, R_AL_ROW, R_AL_EMIT, R_DONE
    } rstate_t;
    rstate_t rs;

    logic [5:0]  tcol;
    logic [8:0]  out_x;
    logic [2:0]  spx;            // source pixel within current tile row
    logic [13:0] code;
    logic        pf_flip;
    logic [2:0]  pf_col_c;
    logic [15:0] gA, gB;
    logic [7:0]  g4;
    logic [3:0]  al_col_c;
    logic        al_opq_c;
    logic        al_blank;       // Pit-Fighter: character code beyond its ROM
    logic [2:0]  al_byte;
    logic [31:0] al_row;

    // 5bpp pixel from fetched row data, source pixel x (pre-flip)
    function automatic logic [4:0] pf_pixel(input logic [2:0] x);
        logic [3:0] nib;
        unique case (x)
            3'd0: nib = gB[15:12];
            3'd1: nib = gB[11:8];
            3'd2: nib = gA[15:12];
            3'd3: nib = gA[11:8];
            3'd4: nib = gB[7:4];
            3'd5: nib = gB[3:0];
            3'd6: nib = gA[7:4];
            3'd7: nib = gA[3:0];
        endcase
        return {g4[3'd7 - x], nib};
    endfunction

    always_ff @(posedge clk) begin
        pfrom_rd <= 1'b0;
        pflb_we  <= 1'b0;
        allb_we  <= 1'b0;

        if (reset) begin
            rs <= R_IDLE;
            hscroll <= '0; src_row <= '0; tile_bank <= '0; scrolly <= '0;
        end else begin
            unique case (rs)
            // Prepare line N+1 from the start of line N. Line 0 is prepared
            // during the last vblank line (261): its scroll slot is the one
            // Pit-Fighter writes, and must be read every frame.
            R_IDLE: if (hcnt == 9'd0 && (vcnt < 9'd240 || vcnt == 9'd261)) begin
                next_line <= (vcnt == 9'd261) ? 9'd0 : vcnt + 9'd1;
                rs <= R_SCR0A;
            end
            R_SCR0A: begin
                // (line/8)*64 + 48 + (line&7)*2
                al_vram_addr <= {next_line[7:3], 6'd48}
                              + {7'd0, next_line[2:0], 1'b0};
                rs <= R_SCR0W;
            end
            R_SCR0W: rs <= R_SCR0B;   // VRAM q settles
            R_SCR0B: begin
                // al_vram_q = scroll word 0 (X scroll). The offset added is
                // 2 on Pit-Fighter and 0 on Hydra, as MAME.
                if (al_vram_q[15])
                    hscroll <= 9'((al_vram_q >> 6) + (is_hydra ? 16'd0 : 16'd2));
                al_vram_addr <= al_vram_addr + 11'd1;
                rs <= R_SCR1W;
            end
            R_SCR1W: rs <= R_SCR1B;
            R_SCR1B: begin
                // al_vram_q = scroll word 1 (Y scroll and bank). As MAME:
                // scrolly = ((word >> 6) - line) & 0x1ff, row = line + scrolly.
                if (al_vram_q[15]) begin
                    scrolly   <= 9'(9'(al_vram_q >> 6) - next_line[8:0]);
                    tile_bank <= al_vram_q[2:0];
                    src_row   <= 9'(al_vram_q >> 6);
                end else
                    src_row <= 9'(next_line[8:0] + scrolly);
                tcol  <= 6'd0;
                out_x <= 9'd0;
                spx   <= hscroll[2:0];    // first tile starts mid-row
                rs    <= R_PF_MAP;
            end

            // ---------------- playfield ----------------
            R_PF_MAP: begin
                pf_vram_addr <= {src_row[8:3], 6'(hscroll[8:3] + tcol)};
                rs <= R_PF_MAPW;
            end
            R_PF_MAPW: rs <= R_PF_MAPQ;
            R_PF_MAPQ: begin
                code     <= {tile_bank[1:0], pf_vram_q[11:0]};
                pf_flip  <= pf_vram_q[15];
                pf_col_c <= pf_vram_q[14:12];
                pf_wait <= 2'd2;
                rs <= R_PF_A;
            end
            // One burst returns plane A (pfrom_data) and plane B (pfrom_data2).
            R_PF_A: begin
                pfrom_addr <= {1'b0, code, src_row[2:0], 2'b00};
                pfrom_rd   <= 1'b1;
                if (pf_wait != 2'd0) pf_wait <= pf_wait - 2'd1;
                else if (pfrom_ready) begin gA <= pfrom_data; pf_wait <= 2'd2; rs <= R_PF_BW; end
            end
            // The hub latches pfrom_data2 a clock after pfrom_ready, so it is
            // valid two clocks after ready: R_PF_BW is that wait.
            R_PF_BW: rs <= R_PF_BQ;
            R_PF_BQ: begin
                gB <= pfrom_data2;
                pf_wait <= 2'd2;
                rs <= R_PF_P4;
            end
            R_PF_P4: begin
                pfrom_addr <= 20'h80000 | {3'b000, code, src_row[2:0]};
                pfrom_rd   <= 1'b1;
                if (pf_wait != 2'd0) pf_wait <= pf_wait - 2'd1;
                else if (pfrom_ready) begin
                    // the word holds two rows' P4 bytes: pick by row bit 0
                    g4 <= src_row[0] ? pfrom_data[7:0] : pfrom_data[15:8];
                    rs <= R_PF_EMIT;
                end
            end
            R_PF_EMIT: begin
                if (out_x < 9'd336) begin
                    pflb_wa <= {wr_buf, out_x[8:0]};
                    pflb_wd <= {pf_col_c, pf_pixel(pf_flip ? 3'd7 - spx : spx)};
                    pflb_we <= 1'b1;
                end
                out_x <= out_x + 9'd1;
                if (spx == 3'd7) begin
                    spx  <= 3'd0;
                    tcol <= tcol + 6'd1;
                    if (out_x >= 9'd335) begin
                        tcol <= 6'd0;
                        rs   <= R_AL_MAP;
                    end else
                        rs <= R_PF_MAP;
                end else
                    spx <= spx + 3'd1;
            end

            // ---------------- alpha ----------------
            R_AL_MAP: begin
                al_vram_addr <= {next_line[7:3], tcol};
                rs <= R_AL_MAPW;
            end
            R_AL_MAPW: rs <= R_AL_MAPQ;
            R_AL_MAPQ: begin
                al_col_c   <= al_vram_q[15:12];
                al_opq_c   <= al_vram_q[15];
                chrom_addr <= {al_vram_q[11:0], next_line[2:0], 2'd0};
                // Pit-Fighter's character ROM region is 128KB with only the
                // first 64KB fitted, so codes 2048-4095 read as blank.
                al_blank   <= !is_hydra && al_vram_q[11];
                al_byte    <= 3'd0;
                rs <= R_AL_ROWW;
            end
            // Read the tile row's 4 bytes. The character ROM is registered, so
            // the address runs one byte ahead of the capture:
            //   ROWW : address A presented
            //   ROW  : capture data[A], data[A+1], data[A+2], data[A+3]
            R_AL_ROWW: begin
                chrom_addr <= chrom_addr + 17'd1;
                rs <= R_AL_ROW;
            end
            R_AL_ROW: begin
                chrom_addr <= chrom_addr + 17'd1;
                al_row <= {al_row[23:0], al_blank ? 8'h00 : chrom_q};
                if (al_byte == 3'd3) begin spx <= 3'd0; rs <= R_AL_EMIT; end
                else al_byte <= al_byte + 3'd1;
            end
            R_AL_EMIT: begin
                allb_wa <= {wr_buf, tcol[5:0], spx};
                allb_wd <= {al_opq_c, al_col_c, al_row[31 - {2'd0,spx,2'b00} -: 4]};
                allb_we <= 1'b1;
                if (spx == 3'd7) begin
                    if (tcol == 6'd41) rs <= R_DONE;
                    else begin tcol <= tcol + 6'd1; rs <= R_AL_MAP; end
                end else spx <= spx + 3'd1;
            end

            R_DONE: if (hcnt != 9'd0) rs <= R_IDLE;
            default: rs <= R_IDLE;
            endcase
        end
    end

    // =================================================================
    // Display: the line buffers' read address follows hcnt (stable
    // between pixel enables); the output is registered on the enable.
    // =================================================================
    always_ff @(posedge clk) if (cen_pix) begin
        {pf_color, pf_pen}            <= pflb_q;
        {al_opaque, al_color, al_pen} <= allb_q;
    end

endmodule
