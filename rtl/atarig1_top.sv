// =============================================================================
// Atari G1 system board (Pit-Fighter, Hydra) - top level
// =============================================================================
//
// Everything on the main board lives here: the 68000, its address decoder and
// RAMs, the slapstic, the playfield/alphanumerics tilemaps, the RLE motion-
// object engine with its framebuffer, the palette and layer mixer, the EEPROM,
// and the JSA II sound board.
//
// The large ROMs (68000 program, playfield tiles, RLE object data, OKI samples)
// sit in SDRAM and are fetched through the memory hub in the wrapper; each has
// its own request/ready port below. The small ROMs (alphanumerics and the
// 6502 program) are copied into block RAM during the ROM download.
//
// Clocking: one system clock (57.272724 MHz) with clock enables:
//   cen_7m    7.159 MHz  pixel clock (MAME: 14.318181 MHz / 2)
//   cen_3m58  3.579 MHz  YM2151
//   cen_1m79  1.790 MHz  6502
//   cen_1m19  1.193 MHz  OKI6295 (3.579545 MHz / 3)
// and the 68000 runs at clk_sys / 4 = 14.318 MHz (see "68000" below).
//
// 68000 memory map (MAME atarig1.cpp):
//   000000-07FFFF  program ROM (SDRAM). The slapstic window is 038000-03FFFF
//                  on Pit-Fighter and 078000-07FFFF on Hydra.
//   F80000         watchdog (not implemented; writes are ignored)
//   F88000         EEPROM unlock
//   F90000         sound command (to the JSA II)
//   F98000         sound CPU reset
//   FA0000         RLE control register (byte at FA0001)
//   FB0000         video interrupt acknowledge
//   FC0000         IN0: vblank, service, sound-command-pending, player 1
//   FC8000         Pit-Fighter: IN1, players 2 and 3
//                  Hydra: ADC0809 - read the result (upper byte); a write to
//                  FC8000+2n starts a conversion of channel n
//   FD0000         sound response (upper byte lane)
//   FD8000-FDFFFF  EEPROM (low byte lane)
//   FE8000-FE89FF  palette RAM
//   FF0000-FF0FFF  RLE object list RAM
//   FF2000         motion-object command register
//   FF4000-FF5FFF  playfield RAM
//   FF6000-FF6FFF  alphanumerics RAM
//   FF0000-FFFFFF  work RAM (wherever the video RAMs above do not answer)
//
// Game differences (is_hydra): the slapstic window, FC8000, IN0 bit 13 (ADC
// end of conversion on Hydra), motion-object command 0 (checksum on
// Pit-Fighter, draw on Hydra), the motion-object clip window and order word,
// the playfield scroll offset, and the alphanumerics ROM (128KB on Hydra;
// Pit-Fighter's has only its first 64KB fitted).
//
// A note on timing that applies throughout: every block RAM here has a
// registered output, so read data belongs to the address presented on the
// PREVIOUS clock. Most of the bugs found while bringing this core up were a
// value sampled one clock too early; the comments point out where it matters.
// =============================================================================

`include "rom_map.svh"

module atarig1_top (
    input  logic        clk_sys,        // 57.272724 MHz
    input  logic        reset,

    input  logic        cen_7m,         // pixel clock enable
    input  logic        cen_3m58,       // YM2151
    input  logic        cen_1m79,       // 6502
    input  logic        cen_1m19,       // OKI6295

    // ---- ROM download (from the HPS) ----
    input  logic        dl_wr,
    input  logic [24:0] dl_addr,        // flat byte address, see rom_map.svh
    input  logic [7:0]  dl_data,
    input  logic        rom_loaded,     // high once the download has finished
    input  logic [255:0] checksums,     // per-ROM checksums from the hub (for MO command 0)

    // ---- which game (from the MRA) ----
    input  logic        is_hydra,       // Hydra; otherwise Pit-Fighter
    input  logic [3:0]  slap_chip,      // slapstic: 0 = none, 1..8 = 137412-111..-118
    input  logic        slap_bootleg,   // the Pit-Fighter bootleg's replacement slapstic

    // ---- SDRAM clients (arbitrated by mem_hub in the wrapper) ----
    // 68000 program ROM, word reads
    output logic [24:0] prog_addr,
    output logic        prog_rd,
    input  logic [15:0] prog_data,
    input  logic        prog_ready,
    // playfield tile graphics
    output logic [19:0] pfrom_addr,
    output logic        pfrom_rd,
    input  logic [15:0] pfrom_data,
    input  logic [15:0] pfrom_data2,    // the next word of the same burst (plane B)
    input  logic        pfrom_ready,
    // RLE motion-object data
    output logic [19:0] rle_rom_waddr,
    output logic        rle_rom_rd,
    input  logic [15:0] rle_rom_data,
    input  logic        rle_rom_ready,
    // OKI ADPCM samples
    output logic [17:0] oki_rom_addr,
    output logic        oki_rom_rd,
    input  logic [7:0]  oki_rom_data,
    input  logic        oki_rom_ready,

    // ---- controls ----
    input  logic [11:0] in0_buttons,    // player 1, active low (bits 6:0 used)
    input  logic [15:0] in1_buttons,    // players 2 and 3, active low (Pit-Fighter)
    input  logic [7:0]  adc_ch0,        // Hydra: yoke X   (ADC0809 channel 0)
    input  logic [7:0]  adc_ch1,        // Hydra: yoke Y   (channel 1)
    input  logic [7:0]  adc_ch2,        // Hydra: accelerator pedal (channel 2)
    input  logic        service_n,      // test switch, active low
    input  logic        pause,          // freeze both CPUs; video keeps running
    input  logic        coin1, coin2,   // active high, read by the sound CPU

    // ---- video ----
    output logic [5:0]  red, green, blue,
    output logic        hblank, vblank, hsync, vsync,
    output logic        ce_pix,

    // ---- audio (the JSA II is mono) ----
    output logic signed [15:0] audio,

    // ---- NVRAM: second port of the EEPROM, for load/save through the HPS ----
    input  logic [10:0] nvram_addr,
    input  logic [7:0]  nvram_din,
    output logic [7:0]  nvram_dout,
    input  logic        nvram_wr,
    output logic        nvram_wpulse,   // one pulse per byte the game writes (drives autosave)

    // ---- status ----
    output logic        prescan_busy    // the CPUs are held while the RLE engine measures objects
);

    // =========================================================================
    // Video timing: 456 x 262 total, 336 x 240 visible, 59.92 Hz
    // =========================================================================
    logic [8:0] hcnt, vcnt;
    logic       vblank_rise;
    logic       t_hblank, t_vblank, t_hsync, t_vsync;

    video_timing u_timing (
        .clk(clk_sys), .cen_pix(cen_7m), .reset(reset),
        .hcnt(hcnt), .vcnt(vcnt),
        .hblank(t_hblank), .vblank(t_vblank), .hsync(t_hsync), .vsync(t_vsync),
        .vblank_rise(vblank_rise)
    );
    assign ce_pix = cen_7m;

    // The colour of pixel h leaves the palette RAM one pixel clock after the
    // counters reach h, so the blanking and syncs sent to the display are
    // delayed by one pixel to match. (Checked against MAME: without this the
    // whole picture sat one pixel to the right.) The game itself reads the
    // undelayed vblank.
    always_ff @(posedge clk_sys)
        if (cen_7m) begin
            hblank <= t_hblank;  vblank <= t_vblank;
            hsync  <= t_hsync;   vsync  <= t_vsync;
        end

    // =========================================================================
    // 68000 (fx68k)
    // =========================================================================
    // fx68k wants enPhi1 and enPhi2 as single-clock pulses. One pair every four
    // system clocks gives 14.318 MHz. PAUSE withholds the enables; the video
    // timing keeps its own enable so the picture stays up while paused.
    logic [1:0] phi_cnt;
    always_ff @(posedge clk_sys) phi_cnt <= phi_cnt + 2'd1;
    wire en_phi1 = (phi_cnt == 2'd0) && !pause;
    wire en_phi2 = (phi_cnt == 2'd2) && !pause;

    logic [23:1] cpu_addr;
    logic [15:0] cpu_dout, cpu_din;
    logic        cpu_rw;
    logic        cpu_as_n, cpu_uds_n, cpu_lds_n;
    logic        fc0, fc1, fc2;
    logic        irq_video, irq_sound;

    wire iack = fc0 & fc1 & fc2;        // function code 7 = interrupt acknowledge

    // Interrupt levels as MAME: sound = 2, video = 1. All are autovectored.
    wire [2:0] ipl   = irq_sound ? 3'd2 : (irq_video ? 3'd1 : 3'd0);
    wire [2:0] ipl_n = ~ipl;

    // Both CPUs are held in reset until the RLE engine has measured every
    // motion object after the ROM download (see rle_objects.sv); that takes
    // about half a second. As a failsafe they are let go after 2^28 clocks
    // (4.7 s) whatever the prescan is doing, so a stuck prescan shows up as a
    // game without motion objects (and the wrapper's LED staying lit) rather
    // than a black screen.
    logic        rle_ready;
    logic [27:0] hold_ctr;
    logic        hold_timeout;
    always_ff @(posedge clk_sys) begin
        if (reset) begin
            hold_ctr <= 28'd0;  hold_timeout <= 1'b0;
        end else if (!rle_ready && !hold_timeout) begin
            hold_ctr <= hold_ctr + 28'd1;
            if (&hold_ctr) hold_timeout <= 1'b1;
        end
    end
    wire  cpu_reset = reset | (~rle_ready & ~hold_timeout);
    assign prescan_busy = rom_loaded & ~rle_ready;

    fx68k u_cpu (
        .clk     (clk_sys),
        .HALTn   (1'b1),
        .extReset(cpu_reset),
        .pwrUp   (cpu_reset),
        .enPhi1  (en_phi1),
        .enPhi2  (en_phi2),

        .eRWn    (cpu_rw),
        .ASn     (cpu_as_n),
        .LDSn    (cpu_lds_n),
        .UDSn    (cpu_uds_n),
        .E       (),
        .VMAn    (),
        .FC0     (fc0), .FC1(fc1), .FC2(fc2),
        .BGn     (),
        .oRESETn (), .oHALTEDn(),

        .DTACKn  (dtack_n),
        .VPAn    (~iack),               // autovector every interrupt
        .BERRn   (1'b1),
        .BRn     (1'b1), .BGACKn(1'b1),
        .IPL0n   (ipl_n[0]), .IPL1n(ipl_n[1]), .IPL2n(ipl_n[2]),
        .iEdb    (cpu_din),
        .oEdb    (cpu_dout),
        .eab     (cpu_addr)
    );

    // ---- DTACK ----
    // Everything on the board answers at once except program ROM, which waits
    // for the SDRAM hub. The fetched word is latched and held until AS rises so
    // it stays stable for the rest of the bus cycle.
    logic        prog_valid;
    logic [15:0] prog_lat;
    always_ff @(posedge clk_sys) begin
        if (cpu_as_n) prog_valid <= 1'b0;
        else if (prog_ready) begin
            prog_lat   <= prog_data;
            prog_valid <= 1'b1;
        end
    end
    // DTACK must stay deasserted during interrupt acknowledge, otherwise the
    // 68000 reads the bus (0xFFFF) as a vector number instead of using VPA.
    wire dtack_n = iack ? 1'b1 : (sel_rom & cpu_rw & ~prog_valid);

    // ---- bus strobes ----
    wire  ds_n = cpu_uds_n & cpu_lds_n;
    logic ds_n_q, as_n_q;
    always_ff @(posedge clk_sys) begin
        ds_n_q <= ds_n;
        as_n_q <= cpu_as_n;
    end
    wire cyc     = ~cpu_as_n & ~iack;                   // a real (non-IACK) bus cycle
    wire wr_stb  = cyc & ~cpu_rw & ds_n_q & ~ds_n;      // one pulse per write cycle
    wire bus_stb = as_n_q & ~cpu_as_n;                  // one pulse per bus cycle

    // =========================================================================
    // Address decode
    // =========================================================================
    wire [23:0] a = {cpu_addr, 1'b0};

    wire sel_rom      = cyc && (a[23:19] == 5'b00000);            // 000000-07FFFF
    wire sel_slapstic = cyc && (slap_chip != 4'd0)
                            && (a[23:15] == (is_hydra ? 9'b0_0000_1111      // 078000-07FFFF
                                                      : 9'b0_0000_0111));   // 038000-03FFFF
    wire sel_eep_unlk = cyc && (a[23:15] == 9'h1F1);              // F88000
    wire sel_snd_cmd  = cyc && (a[23:15] == 9'h1F2);              // F90000
    wire sel_snd_rst  = cyc && (a[23:15] == 9'h1F3);              // F98000
    wire sel_rle_ctl  = cyc && (a[23:15] == 9'h1F4);              // FA0000
    wire sel_vint_ack = cyc && (a[23:15] == 9'h1F6);              // FB0000
    wire sel_in0      = cyc && (a[23:15] == 9'h1F8);              // FC0000
    wire sel_in1      = cyc && (a[23:15] == 9'h1F9);              // FC8000 (IN1 / ADC)
    wire sel_snd_rsp  = cyc && (a[23:15] == 9'h1FA);              // FD0000
    wire sel_eeprom   = cyc && (a[23:15] == 9'h1FB);              // FD8000
    wire sel_palette  = cyc && (a[23:12] == 12'hFE8) && (a[11:0] < 12'hA00);
    wire sel_rle_ram  = cyc && (a[23:12] == 12'hFF0);             // FF0000-FF0FFF
    wire sel_mo_cmd   = cyc && (a[23:1]  == 23'h7F9000);          // FF2000
    wire sel_pf_ram   = cyc && (a[23:13] == 11'h7FA);             // FF4000-FF5FFF
    wire sel_al_ram   = cyc && (a[23:12] == 12'hFF6);             // FF6000-FF6FFF
    wire sel_wram     = cyc && (a[23:16] == 8'hFF)
                            && !sel_rle_ram && !sel_pf_ram && !sel_al_ram;

    // =========================================================================
    // Video interrupt: raised at the start of vblank, cleared by a write to
    // FB0000. (The sound interrupt comes straight from the JSA II below.)
    // =========================================================================
    always_ff @(posedge clk_sys) begin
        if (cpu_reset)        irq_video <= 1'b0;
        else if (vblank_rise) irq_video <= 1'b1;
        else if (sel_vint_ack && wr_stb) irq_video <= 1'b0;
    end

    // =========================================================================
    // Slapstic and the program ROM address
    // =========================================================================
    // The slapstic watches every bus cycle and chooses which 8KB bank of its
    // window the 68000 sees (the window's 32KB holds the four banks).
    logic [1:0] slap_bank;
    slapstic u_slapstic (
        .clk(clk_sys), .reset(reset),
        .chip(slap_chip), .bootleg(slap_bootleg), .win_hi(is_hydra),
        .cs(bus_stb),
        .addr(a),
        .bank(slap_bank)
    );

    wire [24:0] slap_base = is_hydra ? 25'h78000 : 25'h38000;
    assign prog_addr = sel_slapstic
        ? (ROM_MAINCPU_BASE | slap_base | {20'd0, slap_bank, 3'd0} << 10
           | {12'd0, a[12:0]})
        : (ROM_MAINCPU_BASE | {6'd0, a[18:0]});
    assign prog_rd = sel_rom & cpu_rw & ~prog_valid;

    // =========================================================================
    // On-board RAMs
    // =========================================================================
    // Each 16-bit RAM is built from two 8-bit halves, written separately by UDS
    // and LDS. A 68000 byte write puts the byte on both halves of the data bus,
    // so a single word-wide RAM with one write enable would overwrite the
    // neighbouring byte on every byte write.
    wire ram_wr = cyc & ~cpu_rw & ~ds_n;    // level; repeating a RAM write is harmless

    logic [15:0] wram_q, pf_q, al_q, rle_q, pal_q;      // 68000 side
    logic [15:0] pf_qb, al_qb, rle_qb, pal_qb;          // video side

    // ---- work RAM (64KB window; the video RAMs above take priority) ----
    // The video RAMs hide 16KB of the window (FF0000-FF0FFF and FF4000-
    // FF6FFF), so only 48KB is ever reached. To save 16 block RAMs the
    // reachable parts are packed into 24K words:
    //   FF8000-FFFFFF -> words 0x0000-0x3FFF
    //   FF1000-FF3FFF -> words 0x4000-0x57FF
    //   FF7000-FF7FFF -> words 0x5800-0x5FFF
    wire [1:0]  wram_blk  = (cpu_addr[14:12] == 3'd7) ? 2'd3 : (cpu_addr[13:12] - 2'd1);
    wire [14:0] wram_addr = cpu_addr[15] ? {1'b0, cpu_addr[14:1]}
                                         : {2'b10, wram_blk, cpu_addr[11:1]};
    dpram #(.AW(15), .DW(8), .DEPTH(24576)) u_wram_hi (
        .clk(clk_sys),
        .addr_a(wram_addr), .d_a(cpu_dout[15:8]),
        .we_a(ram_wr & sel_wram & ~cpu_uds_n), .q_a(wram_q[15:8]),
        .addr_b('0), .d_b('0), .we_b(1'b0), .q_b()
    );
    dpram #(.AW(15), .DW(8), .DEPTH(24576)) u_wram_lo (
        .clk(clk_sys),
        .addr_a(wram_addr), .d_a(cpu_dout[7:0]),
        .we_a(ram_wr & sel_wram & ~cpu_lds_n), .q_a(wram_q[7:0]),
        .addr_b('0), .d_b('0), .we_b(1'b0), .q_b()
    );

    // ---- playfield RAM: port B is read by the tilemap engine ----
    logic [11:0] tm_pf_addr;
    logic [10:0] tm_al_addr;

    dpram #(.AW(12), .DW(8)) u_pfram_hi (
        .clk(clk_sys),
        .addr_a(cpu_addr[12:1]), .d_a(cpu_dout[15:8]),
        .we_a(ram_wr & sel_pf_ram & ~cpu_uds_n), .q_a(pf_q[15:8]),
        .addr_b(tm_pf_addr), .d_b('0), .we_b(1'b0), .q_b(pf_qb[15:8])
    );
    dpram #(.AW(12), .DW(8)) u_pfram_lo (
        .clk(clk_sys),
        .addr_a(cpu_addr[12:1]), .d_a(cpu_dout[7:0]),
        .we_a(ram_wr & sel_pf_ram & ~cpu_lds_n), .q_a(pf_q[7:0]),
        .addr_b(tm_pf_addr), .d_b('0), .we_b(1'b0), .q_b(pf_qb[7:0])
    );

    // ---- alphanumerics RAM: port B is read by the tilemap engine ----
    dpram #(.AW(11), .DW(8)) u_alram_hi (
        .clk(clk_sys),
        .addr_a(cpu_addr[11:1]), .d_a(cpu_dout[15:8]),
        .we_a(ram_wr & sel_al_ram & ~cpu_uds_n), .q_a(al_q[15:8]),
        .addr_b(tm_al_addr), .d_b('0), .we_b(1'b0), .q_b(al_qb[15:8])
    );
    dpram #(.AW(11), .DW(8)) u_alram_lo (
        .clk(clk_sys),
        .addr_a(cpu_addr[11:1]), .d_a(cpu_dout[7:0]),
        .we_a(ram_wr & sel_al_ram & ~cpu_lds_n), .q_a(al_q[7:0]),
        .addr_b(tm_al_addr), .d_b('0), .we_b(1'b0), .q_b(al_qb[7:0])
    );

    // ---- RLE object list RAM ----
    // Port B belongs to the RLE engine, which reads the object list and, for
    // the checksum command, writes results back (always whole words).
    logic [10:0] rle_ob_addr;
    logic [15:0] rle_ob_d;
    logic        rle_ob_we;

    dpram #(.AW(11), .DW(8)) u_rleram_hi (
        .clk(clk_sys),
        .addr_a(cpu_addr[11:1]), .d_a(cpu_dout[15:8]),
        .we_a(ram_wr & sel_rle_ram & ~cpu_uds_n), .q_a(rle_q[15:8]),
        .addr_b(rle_ob_addr), .d_b(rle_ob_d[15:8]), .we_b(rle_ob_we), .q_b(rle_qb[15:8])
    );
    dpram #(.AW(11), .DW(8)) u_rleram_lo (
        .clk(clk_sys),
        .addr_a(cpu_addr[11:1]), .d_a(cpu_dout[7:0]),
        .we_a(ram_wr & sel_rle_ram & ~cpu_lds_n), .q_a(rle_q[7:0]),
        .addr_b(rle_ob_addr), .d_b(rle_ob_d[7:0]), .we_b(rle_ob_we), .q_b(rle_qb[7:0])
    );

    // ---- palette RAM: 1280 IRGB-1555 entries, port B is the colour lookup ----
    logic [10:0] pal_index;

    dpram #(.AW(11), .DW(8)) u_palram_hi (
        .clk(clk_sys),
        .addr_a(cpu_addr[11:1]), .d_a(cpu_dout[15:8]),
        .we_a(ram_wr & sel_palette & ~cpu_uds_n), .q_a(pal_q[15:8]),
        .addr_b(pal_index), .d_b('0), .we_b(1'b0), .q_b(pal_qb[15:8])
    );
    dpram #(.AW(11), .DW(8)) u_palram_lo (
        .clk(clk_sys),
        .addr_a(cpu_addr[11:1]), .d_a(cpu_dout[7:0]),
        .we_a(ram_wr & sel_palette & ~cpu_lds_n), .q_a(pal_q[7:0]),
        .addr_b(pal_index), .d_b('0), .we_b(1'b0), .q_b(pal_qb[7:0])
    );

    // =========================================================================
    // Block-RAM ROMs, filled during the download
    // =========================================================================
    // Alphanumerics character ROM, 128KB (read by the tilemap engine). The
    // download delivers it in two 64KB pieces; Pit-Fighter supplies only the
    // first, and the tilemap engine blanks its upper character codes.
    logic [16:0] tm_chrom_addr;
    logic [7:0]  chrom_q;
    wire  chr_lo = dl_addr >= ROM_CHARS_BASE    && dl_addr < ROM_CHARS_BASE    + 25'h10000;
    wire  chr_hi = dl_addr >= ROM_CHARS_HI_BASE && dl_addr < ROM_CHARS_HI_BASE + 25'h10000;
    dpram #(.AW(17), .DW(8)) u_chram (
        .clk(clk_sys),
        .addr_a({chr_hi, dl_addr[15:0]}), .d_a(dl_data),
        .we_a(dl_wr && (chr_lo || chr_hi)), .q_a(),
        .addr_b(tm_chrom_addr), .d_b('0), .we_b(1'b0), .q_b(chrom_q)
    );

    // JSA II 6502 program ROM, 64KB.
    logic [15:0] jsa_rom_addr;
    logic [7:0]  jsa_rom_q;
    dpram #(.AW(16), .DW(8)) u_jsarom (
        .clk(clk_sys),
        .addr_a(dl_addr[15:0]), .d_a(dl_data),
        .we_a(dl_wr && dl_addr >= ROM_JSACPU_BASE
                    && dl_addr < ROM_JSACPU_BASE + 25'h10000), .q_a(),
        .addr_b(jsa_rom_addr), .d_b('0), .we_b(1'b0), .q_b(jsa_rom_q)
    );

    // =========================================================================
    // Motion-object command and RLE control registers
    // =========================================================================
    // FF2000: the command the RLE engine runs at the next MOGO (on Pit-Fighter
    // 0 = checksum, anything else = draw; on Hydra always draw). The register
    // comes out of reset as $FFFF ("no command"): running a checksum before the
    // game has written a command would overwrite the object list. MAME likewise
    // does nothing until the game writes one.
    logic [15:0] mo_command;
    always_ff @(posedge clk_sys) begin
        if (reset)
            mo_command <= 16'hFFFF;
        else if (sel_mo_cmd && wr_stb)
            mo_command <= cpu_dout;
    end

    // FA0001: RLE control byte (MOGO, erase, frame flip - see rle_objects.sv).
    logic       rle_ctl_wr;
    logic [7:0] rle_ctl;
    always_ff @(posedge clk_sys) begin
        rle_ctl_wr <= 1'b0;
        if (sel_rle_ctl && wr_stb && !cpu_lds_n) begin
            rle_ctl    <= cpu_dout[7:0];
            rle_ctl_wr <= 1'b1;
        end
    end

    // =========================================================================
    // Motion-object framebuffer (block RAM)
    // =========================================================================
    // As on the real board, the RLE engine has its own video RAM: two buffers
    // of 256 x 240 10-bit palette indices. The engine addresses it as
    // {buffer, y, x}; lines 240-255 never exist, so buffer 1 is placed
    // straight after buffer 0's 240 lines (at 0xF000) and the RAM is 240 x 2
    // lines deep, which saves 8 block RAMs. Port A takes the renderer's
    // writes; port B serves scan-out and its erase-behind-the-beam writes.
    // The renderer draws into one buffer while scan-out shows the other.
    logic [16:0] fb_addr, fbs_addr;
    logic [15:0] fb_wdata, fbs_wdata;
    logic        fb_we, fbs_we;
    logic [9:0]  fbram_qb;

    // {buf, y, x} -> buf * 0xF000 + {y, x}
    wire [16:0] fb_lin  = {1'b0, fb_addr[15:0]}  + (fb_addr[16]  ? 17'h0F000 : 17'h00000);
    wire [16:0] fbs_lin = {1'b0, fbs_addr[15:0]} + (fbs_addr[16] ? 17'h0F000 : 17'h00000);

    dpram #(.AW(17), .DW(10), .DEPTH(2 * 240 * 256)) u_fbram (
        .clk(clk_sys),
        .addr_a(fb_lin),  .d_a(fb_wdata[9:0]),  .we_a(fb_we),  .q_a(),
        .addr_b(fbs_lin), .d_b(fbs_wdata[9:0]), .we_b(fbs_we), .q_b(fbram_qb)
    );

    // =========================================================================
    // EEPROM (2816, 2KB x 8)
    // =========================================================================
    // A write to F88000 unlocks it for exactly one byte write. Port B is the
    // NVRAM path to the SD card. It starts erased (all $FF, from
    // eeprom_init.mem): a blank RAM would read $00, which the game could take
    // for valid settings instead of writing its defaults.
    logic       eep_unlocked;
    logic [7:0] eep_q;
    always_ff @(posedge clk_sys) begin
        if (reset) eep_unlocked <= 1'b0;
        else begin
            if (sel_eep_unlk && wr_stb) eep_unlocked <= 1'b1;
            else if (sel_eeprom && wr_stb && eep_unlocked)
                eep_unlocked <= 1'b0;
        end
    end

    dpram #(.AW(11), .DW(8), .INIT_FILE("eeprom_init.mem")) u_eeprom (
        .clk(clk_sys),
        .addr_a(cpu_addr[11:1]), .d_a(cpu_dout[7:0]),
        .we_a(sel_eeprom && wr_stb && eep_unlocked), .q_a(eep_q),
        .addr_b(nvram_addr), .d_b(nvram_din),
        .we_b(nvram_wr), .q_b(nvram_dout)
    );
    assign nvram_wpulse = sel_eeprom && wr_stb && eep_unlocked;

    // =========================================================================
    // JSA II sound board
    // =========================================================================
    // The 68000 writes a command byte to F90000 (the 6502 gets an NMI), reads
    // the 6502's reply from FD0000 (the 68000 gets IRQ 2 while one is waiting),
    // and can reset the sound CPU by writing F98000.
    logic [7:0] jsa_response;
    logic       jsa_irq, jsa_m2s;

    jsa2 u_jsa (
        .clk(clk_sys), .reset(cpu_reset),
        .cen_1m79(cen_1m79 && !pause), .cen_3m58(cen_3m58), .cen_1m19(cen_1m19),
        .main_cmd(cpu_dout[7:0]),
        .main_cmd_wr(sel_snd_cmd && wr_stb),
        .main_response(jsa_response),
        .main_resp_rd(sel_snd_rsp && bus_stb && cpu_rw),
        .sound_reset(sel_snd_rst && wr_stb),
        .irq_to_main(jsa_irq),
        .m2s_ready(jsa_m2s),
        .s2m_ready(),
        .test_in(~service_n),           // the sound CPU's self-test input is active high
        .coin1(coin1), .coin2(coin2),
        .cpu_rom_addr(jsa_rom_addr),
        .cpu_rom_data(jsa_rom_q),
        .oki_rom_addr(oki_rom_addr),
        .oki_rom_rd(oki_rom_rd),
        .oki_rom_data(oki_rom_data),
        .oki_rom_ready(oki_rom_ready),
        .audio_out(audio)
    );
    assign irq_sound = jsa_irq;

    // =========================================================================
    // Hydra: ADC0809 for the yoke and the accelerator pedal
    // =========================================================================
    // As MAME's adc0808 device, clocked at 14.318181 MHz / 16 = 64 system
    // clocks per ADC clock. A write to FC8000 + 2n starts a conversion of
    // channel n: two ADC clocks later EOC drops and the input is sampled, and
    // 64 ADC clocks after that EOC rises again with the result. The game polls
    // EOC on IN0 bit 13 and reads the result from FC8000.
    logic [5:0]  adc_div;           // system clocks within an ADC clock
    logic [6:0]  adc_step;          // ADC clocks since the start (0 = idle)
    logic [2:0]  adc_chan;
    logic [7:0]  adc_sar;
    logic        adc_eoc;
    wire  [7:0]  adc_in = (adc_chan == 3'd0) ? adc_ch0 :
                          (adc_chan == 3'd1) ? adc_ch1 :
                          (adc_chan == 3'd2) ? adc_ch2 : 8'h00;
    always_ff @(posedge clk_sys) begin
        if (reset) begin
            adc_div <= 6'd0; adc_step <= 7'd0; adc_chan <= 3'd0;
            adc_sar <= 8'hFF; adc_eoc <= 1'b1;
        end else if (is_hydra && sel_in1 && wr_stb) begin
            adc_chan <= {1'b0, a[2:1]};             // start: channel from the address
            adc_div  <= 6'd0;
            adc_step <= 7'd1;
        end else if (adc_step != 7'd0) begin
            adc_div <= adc_div + 6'd1;
            if (adc_div == 6'd63) begin
                adc_step <= (adc_step == 7'd66) ? 7'd0 : adc_step + 7'd1;
                if (adc_step == 7'd2) begin          // conversion begins
                    adc_sar <= adc_in;
                    adc_eoc <= 1'b0;
                end
                if (adc_step == 7'd66) begin         // conversion ends
                    adc_sar <= adc_in;
                    adc_eoc <= 1'b1;
                end
            end
        end
    end

    // =========================================================================
    // 68000 read data
    // =========================================================================
    wire [15:0] in0 = { t_vblank,           // 15    vblank, active high
                        service_n,          // 14    test switch, active low
                        is_hydra ? adc_eoc  // 13    Hydra: ADC end of conversion
                                 : 1'b1,    //       Pit-Fighter: unused
                        ~jsa_m2s,           // 12    0 = a sound command is still unread
                        5'b11111,           // 11:7  unused
                        in0_buttons[6:0] }; // 6:0   Pit-Fighter: jump/start, kick, punch, L, R, U, D
                                            //       Hydra: boost, right thumb, left thumb,
                                            //       right trigger, left trigger (bits 4:0)

    always_comb begin
        cpu_din = 16'hFFFF;
        if      (sel_rom)     cpu_din = prog_lat;
        else if (sel_in0)     cpu_din = in0;
        else if (sel_in1)     cpu_din = is_hydra ? {adc_sar, 8'h00} : in1_buttons;
        // The sound response is one byte at an even address, so it is on the
        // UPPER byte lane (the game reads it with move.b $fd0000). The EEPROM,
        // by contrast, is on the lower lane.
        else if (sel_snd_rsp) cpu_din = {jsa_response, 8'hFF};
        else if (sel_eeprom)  cpu_din = {8'hFF, eep_q};
        else if (sel_palette) cpu_din = pal_q;
        else if (sel_pf_ram)  cpu_din = pf_q;
        else if (sel_al_ram)  cpu_din = al_q;
        else if (sel_rle_ram) cpu_din = rle_q;
        else if (sel_wram)    cpu_din = wram_q;
    end

    // =========================================================================
    // Playfield and alphanumerics
    // =========================================================================
    logic [4:0] pf_pen;  logic [2:0] pf_color;
    logic [3:0] al_pen,  al_color;
    logic       al_opaque;

    tilemaps u_tilemaps (
        .clk(clk_sys), .reset(reset), .cen_pix(cen_7m), .is_hydra(is_hydra),
        .hcnt(hcnt), .vcnt(vcnt),
        .pf_vram_addr(tm_pf_addr), .pf_vram_q(pf_qb),
        .al_vram_addr(tm_al_addr), .al_vram_q(al_qb),
        .pfrom_addr(pfrom_addr), .pfrom_rd(pfrom_rd),
        .pfrom_data(pfrom_data), .pfrom_data2(pfrom_data2), .pfrom_ready(pfrom_ready),
        .chrom_addr(tm_chrom_addr), .chrom_q(chrom_q),
        .pf_pen(pf_pen), .pf_color(pf_color),
        .al_pen(al_pen), .al_color(al_color), .al_opaque(al_opaque)
    );

    // =========================================================================
    // RLE motion-object engine
    // =========================================================================
    logic [9:0] rle_pix;

    rle_objects u_rle (
        .clk(clk_sys), .reset(reset), .is_hydra(is_hydra),
        .mo_command(mo_command),
        .control_wr(rle_ctl_wr), .control_in(rle_ctl),
        .rom_loaded(rom_loaded), .checksums(checksums),
        .busy(), .ready(rle_ready),
        .objram_addr(rle_ob_addr), .objram_q(rle_qb),
        .objram_d(rle_ob_d), .objram_we(rle_ob_we),
        .rle_rom_waddr(rle_rom_waddr), .rle_rom_rd(rle_rom_rd),
        .rle_rom_data(rle_rom_data), .rle_rom_ready(rle_rom_ready),
        .fb_addr(fb_addr), .fb_wdata(fb_wdata), .fb_we(fb_we),
        .fbs_addr(fbs_addr), .fbs_rdata({6'd0, fbram_qb}),
        .fbs_wdata(fbs_wdata), .fbs_we(fbs_we),
        .hcnt(hcnt), .vcnt(vcnt),
        .scan_pix(rle_pix)
    );

    // =========================================================================
    // Layer priority and palette
    // =========================================================================
    // Alphanumerics (palette $100-$1FF) sit on top, then motion objects
    // ($000-$3FF, index 0 transparent), then the playfield ($300-$3FF).
    always_comb begin
        if (al_pen != 4'd0 || al_opaque)
            pal_index = 11'h100 + {3'd0, al_color, al_pen};
        else if (rle_pix != 10'd0)
            pal_index = {1'b0, rle_pix};
        else
            pal_index = 11'h300 + {3'd0, pf_color, pf_pen};
    end

    // IRGB-1555: bit 15 is an intensity bit shared as the LSB of all three
    // 6-bit channels.
    assign red   = {pal_qb[14:10], pal_qb[15]};
    assign green = {pal_qb[9:5],   pal_qb[15]};
    assign blue  = {pal_qb[4:0],   pal_qb[15]};

endmodule
