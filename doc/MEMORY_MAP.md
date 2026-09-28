# Atari G1 (Pit-Fighter) hardware reference

Everything here is transcribed from `atarig1.cpp` / `atarig1.h` / `atarijsa.h`
(MAME, BSD-3-Clause, Aaron Giles) and verified against the `pitfight` ROM set.

## Clocks

| Clock | Frequency | Derivation |
|---|---|---|
| Master XTAL | 14.318181 MHz | crystal |
| 68000 | 14.318181 MHz | XTAL/1 |
| Pixel clock | 7.1590905 MHz | XTAL/2 |
| JSA II 6502 | 1.7897725 MHz | 3.579545/2 |
| YM2151 | 3.579545 MHz | XTAL/4 |
| OKI6295 | 1.193182 MHz | 3.579545/3, pin-7 high |

## Video timing (from `set_raw`)

- Total: 456 x 262 dots @ 7.1590905 MHz → 59.92 Hz, 15.699 kHz hsync
- Visible: 336 x 240 (hbstart=336, vbstart=240)
- VBLANK asserts 68K **IRQ1** (level, ack at 0xFB0000)
- A scanline timer fires every 8 lines starting at line 0 (per-scanline
  playfield scroll updates, read from alpha RAM — port from `video/atarig1.cpp`)

## 68000 memory map (`main_map` + `pitfight_map`)

| Range | R/W | Function |
|---|---|---|
| 000000-07FFFF | R | Program ROM (512KB) |
| 038000-039FFF (mirror 0x6000) | R | **Slapstic bank window** (4 x 8KB banks from ROM 0x38000) |
| F80000-F80001 | W | Watchdog reset |
| F88000-F8FFFF | W | EEPROM unlock |
| F90000 (byte) | W | JSA sound command |
| F98000-F98001 | W | JSA sound reset |
| FA0001 (byte) | W | RLE object control |
| FB0000-FB0001 | W | Video IRQ (IRQ1) ack |
| FC0000-FC0001 | R | IN0 (see bits below) |
| FC8000-FC8007 | R/W | ADC0809 (Hydra only — absent on Pit-Fighter) |
| FD0000 (byte) | R | JSA sound response |
| FD8000-FDFFFF | R/W | 2816 EEPROM, low byte lanes (umask 0x00FF) |
| FE8000-FE89FF | R/W | Palette RAM, IRGB-1555, 1280 entries |
| FF0000-FF0FFF | R/W | RLE object list RAM (shared with RLE engine) |
| FF1000-FF1FFF | R/W | RAM |
| FF2000-FF2001 | R/W | MO command word (writes also latch to RLE engine) |
| FF2002-FF3FFF | R/W | RAM |
| FF4000-FF5FFF | R/W | Playfield VRAM (64x64 tilemap, 8x8 tiles) |
| FF6000-FF6FFF | R/W | Alpha VRAM (64x32 tilemap, 8x8 tiles) |
| FF7000-FFFFFF | R/W | Work RAM |

### IN0 @ FC0000 (Pit-Fighter)

- bit 12: JSA main-to-sound ready (active low custom)
- bit 13: ADC EOC (Hydra) — unused on Pit-Fighter
- bit 14: SERVICE (active low); also fed back to JSA as test_read
- bit 15: VBLANK (active high)
- low bits: player controls (punch/kick/jump per player — see driver input ports)

### Interrupts

- IRQ1: VBLANK (assert on vblank rising, ack @ FB0000)
- IRQ2: JSA II sound-to-main interrupt

## Slapstic

`pitfight` (rev 9) uses **slapstic 137412-114** mapped at 0x38000-0x39FFF
(mirror 0x6000), banking 4 x 8KB from ROM offset 0x38000. Revisions differ
only in slapstic number (111/112/113/114). The genuine state machine
constants must be ported from MAME `slapstic.cpp` (not uploaded yet).

The **bootleg** replacement logic (fully in the driver, implemented in
`rtl/slapstic.sv`): reads within the window, `offset = addr & 0x3fff` (word
offset); offset 0 primes; when primed, offsets 0x42/0x52/0x62/0x72 select
banks 0-3 and unprime; any other offset while primed re-primes.

## Video layers (verified against atarig1_v.cpp + gfx_atarig1)

- **Playfield**: 64x64, 8x8, 5bpp. VRAM word: [11:0] code, [14:12] color,
  [15] X-FLIP (not priority!). Effective code = (bank[1:0]<<12)|code.
  Gfx bytes per tile row y: half A at code*16+y*2, half B at
  0x40000+code*16+y*2, plane-4 byte at 0x80000+code*8+y. Source pixels
  MSB-nibble-first: x0:B0hi x1:B0lo x2:A0hi x3:A0lo x4:B1hi x5:B1lo
  x6:A1hi x7:A1lo; bit4 = P4[7-x]. Palette 0x300 + color*32 + pen.
  Decode validated against the real ROMs (sim renders coherent tiles).
- **Per-scanline scroll**: pairs in ALPHA VRAM at (line/8)*64+48+(line&7)*2.
  w0.15 → hscroll = ((w0>>6)+2)&0x1FF (Pit-Fighter xoffset 2);
  w1.15 → playfield source row = (w1>>6)&0x1FF for that line, bank = w1&7.
- **Alpha**: 64x32, 8x8, **4bpp** packed MSB-first (32 bytes/tile,
  nibble per pixel). VRAM word: [11:0] code, [15:12] color, bit15 also =
  opaque flag (pen 0 forced opaque; opaque tiles use colors 8-15).
  Palette 0x100 + color*16 + pen. Decode validated (font is readable).
- **Layering** (screen_update): playfield (opaque bottom) → RLE objects
  (pen 0 transparent) → alpha (pen 0 transparent unless opaque flag).
- **RLE objects**: object list at FF0000, MO command kicks. Pit-Fighter:
  clip x 40-295, palette base 0x200; word0 [15]=hflip [14:0]=code;
  word1 [7:4]=color; word2 [15:6]=X; word3 [15:6]=Y; word4 = scale
  (4.12); word6 [7:0]=order (0 = hidden; draw 1→255, descending index
  within an order).
- **Palette**: IRGB-1555, 1280 entries (alpha 0x100, RLE 0x200,
  playfield 0x300)

## JSA II audio board (atarijsa.h; full map in atarijsa.cpp — port needed)

6502 + YM2151 + OKI6295, banked 6502 ROM, mix register controlling relative
YM/OKI volumes, communication latches to/from 68000 with ready flags.
