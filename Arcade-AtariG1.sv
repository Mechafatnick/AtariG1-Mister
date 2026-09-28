//============================================================================
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

// MiSTer wrapper for the Atari G1 core (Pit-Fighter, Hydra).
//
// This file connects the framework (hps_io, video_mixer, SDRAM pins) to the
// core:
//   pll         57.272724 MHz system clock
//   sdram       SDRAM controller (Sorgelig's, channel 3 used)
//   mem_hub     ROM download into SDRAM and arbitration for the core's
//               ROM clients (rtl/mem_hub.sv)
//   atarig1_top the game board: 68000, video, JSA II sound (rtl/atarig1_top.sv)
//   comp_blend  the optional Composite Blend filter (at the end of this file)
//
// ROM layout in the download (and in SDRAM) is given in rtl/rom_map.svh and
// must match the MRA. The MRA also sends two configuration bytes (rom index
// 1) that choose the game and the slapstic; see "game configuration" below.

module emu
(
	`include "sys/emu_ports.vh"
);

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = '0;  

assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_S = 1;   // signed samples from the JSA II
assign AUDIO_MIX = 0;

assign LED_DISK = 0;
assign LED_POWER = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////

wire [1:0] ar = status[122:121];

assign VIDEO_ARX = (!ar) ? 12'd4 : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? 12'd3 : 12'd0;

`include "build_id.v" 
localparam CONF_STR = {
	"AtariG1;;",
	"-;",
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O[3:2],Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%;",
	"O[14],Composite Blend,Off,On;",
	"H0O[15],Yoke Up/Down,Normal,Inverted;",
	"-;",
	"O[6],Service,Off,On;",
	"O[12],Pause,Off,On;",
	"-;",
	"T[0],Reset;",
	"R[0],Reset and close OSD;",
	"J1,Punch,Kick,Jump,Start,Coin,Pause;",
	"v,0;",
	"V,v",`BUILD_DATE
};

wire forced_scandoubler;
wire   [1:0] buttons;
wire [127:0] status;
wire  [10:0] ps2_key;
wire [31:0] joystick_0, joystick_1;
wire [15:0] joystick_l_analog_0, joystick_r_analog_0;   // Y [15:8], X [7:0], signed
wire        ioctl_download;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire [15:0] ioctl_index;
wire        ioctl_wait;
// NVRAM save path (hps_io upload)
wire        ioctl_upload;
wire        ioctl_rd;
wire  [7:0] ioctl_din;
wire  [7:0] nvram_dout;
wire        nvram_wpulse;
wire        nv_save_req;

wire [21:0] gamma_bus;

// game configuration from the MRA, see below
reg       cfg_hydra = 1'b0;
reg [3:0] cfg_slap  = 4'd4;
reg       cfg_boot  = 1'b0;

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_menumask({~cfg_hydra}),     // bit 0 hides the Hydra-only options (H0)

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.joystick_l_analog_0(joystick_l_analog_0),
	.joystick_r_analog_0(joystick_r_analog_0),

	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_index(ioctl_index),
	.ioctl_wait(ioctl_wait),
	.ioctl_upload(ioctl_upload),
	.ioctl_upload_req(nv_save_req),
	.ioctl_upload_index(8'd2),      // matches <nvram index="2"> in the MRA
	.ioctl_din(ioctl_din),
	.ioctl_rd(ioctl_rd),

	.ps2_key(ps2_key)
);

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;           // 57.272724 MHz = 4 x the board's 14.318181 MHz
wire pll_locked;
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.locked(pll_locked)
);

// ---- game configuration (MRA rom index 1) ----
//   byte 0  game: 0 = Pit-Fighter, 1 = Hydra
//   byte 1  slapstic: [3:0] 0 = none, 1..8 = 137412-111..-118,
//                     [4]   the Pit-Fighter bootleg's replacement chip
// An MRA without these bytes gets Pit-Fighter with slapstic 114 (rev 9); one
// with only byte 0 gets that game's production slapstic (114, or 116 for Hydra).
// Loading an MRA always reloads the core, so these start from the defaults.
// (The registers are declared above hps_io, which uses cfg_hydra.)
always @(posedge clk_sys) begin
	if (ioctl_download && ioctl_wr && ioctl_index == 16'd1) begin
		if (ioctl_addr == 27'd0) begin
			cfg_hydra <= ioctl_dout[0];
			cfg_slap  <= ioctl_dout[0] ? 4'd6 : 4'd4;
		end
		if (ioctl_addr == 27'd1) begin
			cfg_slap <= ioctl_dout[3:0];
			cfg_boot <= ioctl_dout[4];
		end
	end
end

// ---- button layout (joystick bits; the MRA names them in this order) ----
//   bit  Pit-Fighter  Hydra
//    4   Punch        Pedal (fully pressed; the game starts on the pedal)
//    5   Kick         Right Trigger
//    6   Jump         Right Thumb
//    7   Start        Boost
//    8   Coin         Left Trigger
//    9   Pause        Left Thumb
//   10                Coin
//   11                Pause
// The board's test (service) switch is the OSD's "Service" option.
wire j0_coin    = cfg_hydra ? joystick_0[10] : joystick_0[8];
wire j1_coin    = cfg_hydra ? joystick_1[10] : joystick_1[8];
wire j_pause    = cfg_hydra ? (joystick_0[11] | joystick_1[11])
                            : (joystick_0[9]  | joystick_1[9]);

wire reset = RESET | status[0] | ioctl_download;

// Pause: the OSD option, or the Pause button, which toggles (press to pause,
// press again to resume; a reset or a new game resumes). The CPUs stop; video
// keeps running.
reg pause_btn = 1'b0;
reg j_pause_d = 1'b0;
always @(posedge clk_sys) begin
	j_pause_d <= j_pause;
	if (reset)                      pause_btn <= 1'b0;
	else if (j_pause && !j_pause_d) pause_btn <= ~pause_btn;
end
wire pause = status[12] | pause_btn;

// ---- NVRAM (EEPROM) saving: on OSD open and after the game's writes ----
// (rtl/nv_save.sv; never while the game is part-way through writing a record)
assign ioctl_din = nvram_dout;
nv_save u_nv_save (
    .clk(clk_sys), .ee_write(nvram_wpulse), .osd(OSD_STATUS),
    .download(ioctl_download), .upload(ioctl_upload), .save_req(nv_save_req)
);

// ---- clock enables ----
// 57.272724 / 8  = 7.159 MHz    pixel clock
// 57.272724 / 16 = 3.579545 MHz YM2151
// 57.272724 / 32 = 1.789772 MHz 6502 (exactly half the YM2151 enable)
// 57.272724 / 48 = 1.193182 MHz OKI6295 (MAME: JSA master clock / 3)
wire cen_7m, cen_3m58, cen_1m79, cen_1m19;
reg [5:0] cen_ctr;   // 0..47, for the OKI
always @(posedge clk_sys) cen_ctr <= (cen_ctr == 6'd47) ? 6'd0 : cen_ctr + 6'd1;
reg [4:0] div32;
always @(posedge clk_sys) div32 <= div32 + 5'd1;
assign cen_7m   = (div32[2:0] == 3'd0);
assign cen_3m58 = (div32[3:0] == 4'd0);
assign cen_1m79 = (div32       == 5'd0);
assign cen_1m19 = (cen_ctr == 6'd0);

// ---- SDRAM controller (only channel 3 is used, by the memory hub) ----
wire [26:1] ch3_addr;  wire [15:0] ch3_din, ch3_dout;
wire [15:0] ch3_dout2;   // second word of the burst
wire  [1:0] ch3_be;    wire ch3_req, ch3_rnw, ch3_ready;
wire  [2:0] sd_rdsel;          // read sample point, chosen by mem_hub

sdram u_sdram (
	.init(~pll_locked), .clk(clk_sys), .doRefresh(1'b0),
	.SDRAM_A(SDRAM_A), .SDRAM_DQ(SDRAM_DQ),
	.SDRAM_DQML(SDRAM_DQML), .SDRAM_DQMH(SDRAM_DQMH),
	.SDRAM_BA(SDRAM_BA), .SDRAM_nCS(SDRAM_nCS), .SDRAM_nWE(SDRAM_nWE),
	.SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nCAS(SDRAM_nCAS),
	.SDRAM_CKE(SDRAM_CKE), .SDRAM_CLK(SDRAM_CLK),
	.ch1_addr(26'd0), .ch1_req(1'b0), .ch1_dout(), .ch1_ready(),
	.ch2_addr(26'd0), .ch2_req(1'b0), .ch2_dout(), .ch2_ready(),
	.ch3_addr(ch3_addr), .ch3_din(ch3_din), .ch3_be(ch3_be),
	.ch3_req(ch3_req), .ch3_rnw(ch3_rnw),
	.ch3_dout(ch3_dout), .ch3_dout2(ch3_dout2), .ch3_ready(ch3_ready),
	.ch3_rdsel(sd_rdsel)
);

// ---- memory hub ----
wire        rom_loaded;
wire [255:0] g1_checksums;
wire        dl_wr;   wire [24:0] dl_addr;  wire [7:0] dl_data;
wire [24:0] prog_addr;  wire prog_rd;  wire [15:0] prog_data;  wire prog_ready;
wire [19:0] pfrom_addr; wire pfrom_rd; wire [15:0] pfrom_data; wire pfrom_ready;
wire [15:0] pfrom_data2;   // plane B word, from the same burst
wire [19:0] rle_waddr;  wire rle_rd;   wire [15:0] rle_data;   wire rle_ready;
wire [17:0] oki_addr;   wire oki_rd;   wire [7:0] oki_data;    wire oki_ready;

// Interleave the two playfield bitplanes as the ROM streams in, so a tile
// row's plane A and plane B words sit next to each other in SDRAM and one
// 4-word burst returns both. (The real board reads its bitplanes from separate
// chips in parallel.) Offsets within the 640KB tile region:
//   plane A  (o <  0x40000):  new = (o>>1)*4 + (o&1)
//   plane B  (o <  0x80000):  new = ((o-0x40000)>>1)*4 + 2 + (o&1)
//   plane 4  (o >= 0x80000):  unchanged
localparam [26:0] TILES_LO = 27'h0E0000;
localparam [26:0] TILES_HI = 27'h180000;
wire [26:0] tofs = ioctl_addr - TILES_LO;
wire in_tiles    = (ioctl_addr >= TILES_LO) && (ioctl_addr < TILES_HI);
wire [26:0] tofsb = tofs - 27'h40000;
wire [26:0] tnew  = (tofs < 27'h40000) ? (((tofs  >> 1) << 2) | (tofs  & 27'd1))
                  : (tofs < 27'h80000) ? (((tofsb >> 1) << 2) | 27'd2 | (tofsb & 27'd1))
                  :                      tofs;
wire [26:0] ioctl_addr_rp = in_tiles ? (TILES_LO + tnew) : ioctl_addr;

// The hub is reset only by the framework's reset: it must keep running during
// the download (it writes the ROM into SDRAM) and through an OSD reset.
mem_hub u_hub (
	.clk(clk_sys), .reset(RESET),
	.ioctl_download(ioctl_download), .ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr_rp), .ioctl_dout(ioctl_dout),
	.ioctl_index(ioctl_index), .ioctl_wait(ioctl_wait),
	.rom_loaded(rom_loaded), .checksums(g1_checksums),
	// SDRAM read sample point, calibrated after each download (see
	// mem_hub.sv / sdram.sv). rdsel_force n forces point n-1, for testing.
	.rdsel_force(4'd0), .rdsel(sd_rdsel), .cal_mask(),
	.dl_wr(dl_wr), .dl_addr(dl_addr), .dl_data(dl_data),
	.prog_addr(prog_addr), .prog_rd(prog_rd),
	.prog_data(prog_data), .prog_ready(prog_ready),
	.pfrom_addr(pfrom_addr), .pfrom_rd(pfrom_rd),
	.pfrom_data(pfrom_data), .pfrom_data2(pfrom_data2), .pfrom_ready(pfrom_ready),
	.rle_waddr(rle_waddr), .rle_rd(rle_rd),
	.rle_data(rle_data), .rle_ready(rle_ready),
	.oki_addr(oki_addr), .oki_rd(oki_rd),
	.oki_data(oki_data), .oki_ready(oki_ready),
	.ch3_addr(ch3_addr), .ch3_din(ch3_din), .ch3_be(ch3_be),
	.ch3_req(ch3_req), .ch3_rnw(ch3_rnw),
	.ch3_dout(ch3_dout), .ch3_dout2(ch3_dout2), .ch3_ready(ch3_ready)
);

// ---- controls ----
// MiSTer joystick bits [3:0] = up, down, left, right as bits 3, 2, 1, 0;
// the buttons are in the table above.
//
// Pit-Fighter: IN0 (FC0000) bits 6:0, as MAME: Jump/Start, Kick, Punch, L, R,
// U, D. On this cabinet Jump is also the start button, so Start is ORed into it.
wire [6:0] pf_in0 = ~{joystick_0[6] | joystick_0[7], joystick_0[5], joystick_0[4],
                      joystick_0[1], joystick_0[0],
                      joystick_0[3], joystick_0[2]};
// Hydra: IN0 bits 4:0, as MAME: Boost, Right Thumb, Left Thumb, Right
// Trigger, Left Trigger.
wire [6:0] hy_in0 = ~{2'b00, joystick_0[7], joystick_0[6], joystick_0[9],
                      joystick_0[5], joystick_0[8]};
wire [11:0] g1_in0 = {5'b11111, cfg_hydra ? hy_in0 : pf_in0};
// IN1 (FC8000, Pit-Fighter): player 3 in bits 6:0, player 2 in 14:8 (active
// low). Only player 2 is mapped (from the second joystick).
wire [15:0] g1_in1 = ~{1'b0,
                       joystick_1[6] | joystick_1[7], joystick_1[5], joystick_1[4],
                       joystick_1[1], joystick_1[0], joystick_1[3], joystick_1[2],
                       1'b0,
                       7'b0};

// Hydra's analogue controls, read through the ADC0809 (0x80 = centre):
//   channel 0  yoke left/right: left stick X, or the d-pad at full deflection
//   channel 1  yoke up/down:    left stick Y, or the d-pad (OSD can invert it)
//   channel 2  accelerator pedal: right stick pushed up (centre = released),
//                               or the Pedal button (fully pressed)
// Directions follow MAME: left and up give low values.
wire [7:0] lx = joystick_l_analog_0[7:0];
wire [7:0] ly = joystick_l_analog_0[15:8];
wire [7:0] ry = joystick_r_analog_0[15:8];
wire [7:0] yoke_x  = joystick_0[1] ? 8'h00 : joystick_0[0] ? 8'hFF : {~lx[7], lx[6:0]};
wire [7:0] yoke_y0 = joystick_0[3] ? 8'h00 : joystick_0[2] ? 8'hFF : {~ly[7], ly[6:0]};
wire [7:0] yoke_y  = status[15] ? ~yoke_y0 : yoke_y0;
wire [7:0] ry_up   = 8'd0 - ry;                       // how far up, when ry < 0
wire [7:0] thr_an  = !ry[7]   ? 8'h00 :
                     ry_up[7] ? 8'hFF : {ry_up[6:0], 1'b0};
wire [7:0] pedal   = joystick_0[4] ? 8'hFF : thr_an;


wire signed [15:0] g1_audio;
wire g1_prescan_busy;
wire [5:0] g1_r, g1_g, g1_b;
wire g1_hb, g1_vb, g1_hs, g1_vs, g1_cepix;

atarig1_top u_atarig1 (
    .clk_sys(clk_sys), .reset(reset | ~rom_loaded),
    .cen_7m(cen_7m), .cen_3m58(cen_3m58),
    .cen_1m79(cen_1m79), .cen_1m19(cen_1m19),
    .dl_wr(dl_wr), .dl_addr(dl_addr), .dl_data(dl_data),
    .rom_loaded(rom_loaded), .checksums(g1_checksums),
    .is_hydra(cfg_hydra), .slap_chip(cfg_slap), .slap_bootleg(cfg_boot),
    .prog_addr(prog_addr), .prog_rd(prog_rd),
    .prog_data(prog_data), .prog_ready(prog_ready),
    .pfrom_addr(pfrom_addr), .pfrom_rd(pfrom_rd),
    .pfrom_data(pfrom_data), .pfrom_data2(pfrom_data2), .pfrom_ready(pfrom_ready),
    .rle_rom_waddr(rle_waddr), .rle_rom_rd(rle_rd),
    .rle_rom_data(rle_data), .rle_rom_ready(rle_ready),
    .oki_rom_addr(oki_addr), .oki_rom_rd(oki_rd),
    .oki_rom_data(oki_data), .oki_rom_ready(oki_ready),
    .in0_buttons(g1_in0), .in1_buttons(g1_in1),
    .adc_ch0(yoke_x), .adc_ch1(yoke_y), .adc_ch2(pedal),
    .service_n(~status[6]),
    .pause(pause),
    .coin1(j0_coin), .coin2(j1_coin),
    .red(g1_r), .green(g1_g), .blue(g1_b),
    .hblank(g1_hb), .vblank(g1_vb), .hsync(g1_hs), .vsync(g1_vs),
    .ce_pix(g1_cepix),
    .audio(g1_audio),
    // NVRAM: the EEPROM's second port.
    //   load: the HPS sends the .nvm as index 2 (the hub only takes index 0,
    //         and the CPUs are held in reset for the whole download)
    //   save: the HPS reads ioctl_din, one byte per ioctl_rd, at ioctl_addr
    .nvram_addr(ioctl_addr[10:0]), .nvram_din(ioctl_dout), .nvram_dout(nvram_dout),
    .nvram_wr(ioctl_download && ioctl_wr && (ioctl_index == 16'd2) && (ioctl_addr < 27'd2048)),
    .nvram_wpulse(nvram_wpulse),
    .prescan_busy(g1_prescan_busy)
);

assign AUDIO_L = g1_audio;         // the JSA II is mono
assign AUDIO_R = g1_audio;

// ---- video ----
assign CLK_VIDEO = clk_sys;

// Composite Blend (OSD): each pixel averaged with its left-hand neighbour.
wire [5:0] vid_r, vid_g, vid_b;
wire       vid_hs, vid_vs, vid_hb, vid_vb;

comp_blend #(.W(6)) u_blend (
	.clk(clk_sys), .ce_pix(g1_cepix), .enable(status[14]),
	.r_in(g1_r), .g_in(g1_g), .b_in(g1_b),
	.hs_in(g1_hs), .vs_in(g1_vs), .hb_in(g1_hb), .vb_in(g1_vb),
	.r_out(vid_r), .g_out(vid_g), .b_out(vid_b),
	.hs_out(vid_hs), .vs_out(vid_vs), .hb_out(vid_hb), .vb_out(vid_vb)
);

wire [2:0] fx = status[3:2] == 2'd0 ? 3'd0 :          // None
                status[3:2] == 2'd1 ? 3'd1 :          // HQ2x
                status[3:2] == 2'd2 ? 3'd3 : 3'd4;    // CRT 25% / 50%
wire [2:0] sl = fx ? fx - 1'd1 : 3'd0;
wire       scandoubler = (fx != 0) || forced_scandoubler;

assign VGA_SL = sl[1:0];

video_mixer #(.LINE_LENGTH(480), .HALF_DEPTH(0), .GAMMA(1)) video_mixer
(
	.CLK_VIDEO(CLK_VIDEO),
	.CE_PIXEL(CE_PIXEL),
	.ce_pix(g1_cepix),

	.scandoubler(scandoubler),
	.hq2x(fx == 3'd1),
	.gamma_bus(gamma_bus),

	.R({vid_r, vid_r[5:4]}),
	.G({vid_g, vid_g[5:4]}),
	.B({vid_b, vid_b[5:4]}),

	.HSync(vid_hs),
	.VSync(vid_vs),
	.HBlank(vid_hb),
	.VBlank(vid_vb),

	.HDMI_FREEZE(1'b0),
	.freeze_sync(),

	.VGA_R(VGA_R),
	.VGA_G(VGA_G),
	.VGA_B(VGA_B),
	.VGA_VS(VGA_VS),
	.VGA_HS(VGA_HS),
	.VGA_DE(VGA_DE)
);

// lit while the ROM is loading, and while the core then measures the motion
// objects (about half a second; if it stays lit, that prescan is stuck)
assign LED_USER = ioctl_download | g1_prescan_busy;

endmodule


// ============================================================================
// comp_blend - "Composite Blend" video option
// (kept in this file so that it needs no entry in the project's file lists)
//
// Like the Composite Blend option in the MiSTer Genesis core: a composite
// video path cannot resolve single-pixel detail, so each pixel runs into its
// left-hand neighbour. Here every output pixel is
// the average of the current and previous pixel on the line, which turns
// dithered patterns into their blended colour and softens hard vertical edges.
//
// All video signals pass through one pixel register, so syncs and blanks stay
// aligned with the colour whether the blend is on or off. The first pixel
// after horizontal blanking is not mixed with the blanking colour, so the left
// edge of the picture is not darkened.
// ============================================================================
module comp_blend #(
    parameter W = 6                      // bits per colour channel
) (
    input  logic         clk,
    input  logic         ce_pix,
    input  logic         enable,

    input  logic [W-1:0] r_in, g_in, b_in,
    input  logic         hs_in, vs_in, hb_in, vb_in,

    output logic [W-1:0] r_out, g_out, b_out,
    output logic         hs_out, vs_out, hb_out, vb_out
);

    // rounded average of two channel values (plain Verilog-2001 form, which
    // every Quartus version accepts)
    function [W-1:0] avg;
        input [W-1:0] a;
        input [W-1:0] b;
        reg   [W:0]   s;
        begin
            s   = {1'b0, a} + {1'b0, b} + 1'b1;
            avg = s[W:1];
        end
    endfunction

    logic [W-1:0] r_prev, g_prev, b_prev;
    logic         blank_prev;

    wire blank_now = hb_in | vb_in;
    wire mix       = enable & ~blank_now & ~blank_prev;

    always_ff @(posedge clk) begin
        if (ce_pix) begin
            r_prev     <= r_in;
            g_prev     <= g_in;
            b_prev     <= b_in;
            blank_prev <= blank_now;

            r_out  <= mix ? avg(r_prev, r_in) : r_in;
            g_out  <= mix ? avg(g_prev, g_in) : g_in;
            b_out  <= mix ? avg(b_prev, b_in) : b_in;

            hs_out <= hs_in;
            vs_out <= vs_in;
            hb_out <= hb_in;
            vb_out <= vb_in;
        end
    end

endmodule
