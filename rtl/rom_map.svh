// Atari G1 - layout of the ROM download (and of SDRAM). Must match the MRAs.
`ifndef ROM_MAP_SVH
`define ROM_MAP_SVH

localparam ROM_MAINCPU_BASE = 25'h000_0000; // 512KB 68000 program (LE words)
localparam ROM_JSACPU_BASE  = 25'h008_0000; // 64KB 6502 program
localparam ROM_OKI_BASE     = 25'h009_0000; // 256KB ADPCM
localparam ROM_CHARS_BASE   = 25'h00D_0000; // alphanumerics, first 64KB
localparam ROM_CHARS_HI_BASE= 25'h039_0000; // alphanumerics, second 64KB (Hydra only)
localparam ROM_TILES_BASE   = 25'h00E_0000; // 640KB playfield (MAME hole layout)
localparam ROM_RLE_BASE     = 25'h018_0000; // RLE object data (2MB Pit-Fighter, 1MB Hydra)
localparam ROM_PROM_BASE    = 25'h038_0000; // 1.5KB growth-renderer microcode

`endif
