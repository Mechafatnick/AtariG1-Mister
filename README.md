# Atari G1 (Pit-Fighter, Hydra) for MiSTer

An FPGA recreation of Atari Games' G1 arcade board, for the MiSTer DE10-Nano. The board ran
two games, which were both supported: **Pit-Fighter** (1990) and **Hydra** (1990). 
This is fundamentally a port of the MAME interpretation, with improvements from the original schematics used to bring it as close
to 1:1 accuracy as is possible from someone who only partially knows what they are doing.

# The Inevitable AI decleration

Yes, chat, This core is indeed an AI-assisted slop core. I'm sorry! Though I do have some programming Skillz I don't have the bandwidth
to learn enough about the verilog side of things so I've relied on Claude for the implementation. I had access to a Claude account (long story) so I thought I'd make use of it. Partly because i wanted my Pit-Fighter-themed Mister arcade machine to actually be able to play Pit-Fighter, and partly because I was curious about how well AI could realistically 'vibe code' a core.

# How I sleep at night

It's pretty obvious that the powers that be aren't interested in little people - they want big business buy in military buy in, so everyday people bycotting is unlikely to be an effective strategy (that's not to say lose hope and accept it - lobbying your local politician against the building of the data centers is likely to be a more effective result.)

The way I see it, its useful to have people who can argue against things based on experience rather than the theoretical. "I needed to tell it three times before it would acknowledge the schematic it needed to use to solve the problem" is a stronger argument than "it just can't do, X, OK!?". If i'm going to talk about the limits of AI i'd much rather do so from the perspective of having used it in good faith, with this core being evidence I've done just that.

On top of that, would AI execs actually want us to use their products? I was loath to start a new session, so many of my prompts used  5 hours worth of usage in a single go. I'd hate to think how much Pit-Fighter and Hydra must have cost Anthropic, and how commercially useful is it to have extra examples of code targeted at the 68k?

If you aren't swayed by these arguments that's fine. These were dumb games in 1990 and they are dumb games now. I'm sure someone will get round to creating a more authentic core based on hands-on experience with the original board eventually :)

# Was the Ai actually any good?

Very mixed to be honest with you. At the beginning i had to bully it into taking on the project in the first place ("A project of this nature would take an experienced engineer many months of work" "well, you'd better crack on then!") but of course when i forced it the AI was useful in the way it could rapidly edit code and test it in a simulator. 

Perhaps I was doing something, but I do have trouble how anyone gets anything effective out of vibe coding. Though the AI could progress under its own steam to a point, it frequently gave away its inability to reason. Without me setting it on a better path it would simply have sat their reving its engine in neutral, often forgetting basic elements of the design (like SLAPSTIC being specifically designed to stop the kind of reverse engineering we were engaged with). I was reluctant to start a new session because I was worried it wouldn't recover the "knowledge" it had from the previous session, and this fear was justified as it started creating debug overlays in a completely different manner (despite me having had it create detailed "handover" notes for itself). I'm really not sure how far anyone would have got without some form of tehcnical knowledge of how the board hangs together and how it achieved its scaling.


# Core Deets

## OSD
| Option | |
|---|---|
| Scandoubler Fx | None, HQ2x, CRT 25%, CRT 50% |
| Composite Blend | Off / On: each pixel is averaged with its left neighbour, as a composite video path would, so dithering blends |
| Yoke Up/Down | Hydra only: Normal / Inverted |
| Service | The board's test switch (turn it on and reset for the self-test menus) |
| Pause | Stops both CPUs; also on a mappable button |

### Pit-Fighter controls
Punch, Kick, Jump, Start (Start and Jump are the same button on this cabinet) Pause. 
Player 2 uses the second joystick.

### Hydra controls
The cabinet has Atari's flight yoke (steering, and altitude in flight) with two triggers (main weapons. Dakka dakka.)
and two thumb buttons (left selects a special weapon, right fires it), a Boost
button and an accelerator pedal. The game starts on the pedal ("Press PEDAL to start").

| Control | Pad |
|---|---|
| Yoke | left analogue stick, or the d-pad (full deflection) |
| Pedal | right analogue stick pushed up (centred = released), or the Pedal button (fully pressed) |
| Right Trigger, Right Thumb, Boost, Left Trigger, Left Thumb | buttons |
| Coin, Service, Pause | buttons |

## ROMs
One MRA per game, in `mra/`, all made by `tools/gen_mra.py` from MAME's driver:

| MRA | MAME set | Tested |
|---|---|---|
| Pit-Fighter (rev 9) | `pitfight` | hardware |
| Hydra | `hydra` | simulation |
| Pit-Fighter (rev 7, 6, 5, 4, 3, 2), (rev 1, 2 players), (Japan rev 3, 2 players), (bootleg) | `pitfight7` ... `pitfightb` | no |
| Hydra (prototype 5-14-90), (prototype 5-25-90) | `hydrap`, `hydrap2` | no |

Your ideal rom set is the merged set for both titles as this will load all of the variants.

## Source layout
| File | Contents |
|---|---|
| `Arcade-AtariG1.sv` | MiSTer wrapper: OSD, clocks, SDRAM, memory hub, video output, and the Composite Blend filter |
| `rtl/atarig1_top.sv` | the board: 68000, address decoder, RAMs, interrupts, EEPROM, layer mixer |
| `rtl/mem_hub.sv` | ROM download into SDRAM and arbitration for the core's ROM reads |
| `rtl/sdram.sv` | SDRAM controller (Sorgelig's, with a second burst word on channel 3) |
| `rtl/slapstic.sv` | Atari slapstics 137412-111 to -118 and the Pit-Fighter bootleg's replacement (ROM banking / protection) |
| `rtl/video/video_timing.sv` | 456 x 262 raster timing |
| `rtl/video/tilemaps.sv` | playfield and alphanumerics layers |
| `rtl/video/rle_objects.sv` | the RLE motion-object engine ("growth renderer"), its start-up object prescan and its framebuffer scan-out |
| `rtl/jsa2/jsa2.sv` | JSA II sound board: 6502, YM2151, OKI6295, mailbox with the 68000 |
| `rtl/dpram.sv` | dual-port block RAM used throughout |
| `rtl/eeprom_init.mem` | an erased EEPROM (all $FF) |
| `rtl/ext/` | vendored cores: fx68k (68000), Arlet Ottens' 6502, jt51 (YM2151), jt6295 (OKI6295) |
| `sim/` | Verilator testbenches for the whole board and for the sound board (see `sim/README.md`) |
| `doc/MEMORY_MAP.md` | hardware reference transcribed from MAME |
| `tools/gen_mra.py` | writes the MRAs from MAME's `atarig1.cpp` |
| `tools/gen_rle_info.py` | the motion-object size table computed offline, to check the core's prescan |

The vendored cores are unmodified except for one fix in the 6502 (`rtl/ext/a6502/cpu.v`, marked
"LOCAL CHANGE"): an NMI arriving during the entry to an IRQ was lost, which could deadlock the
sound board's mailbox. jt6295 is upstream as of commit 7d76b0b (ADPCM overflow fix).

## Where the core deliberately differs from MAME
- **Motion-object horizontal scaling** follows the board's rate-multiplier method as laid out in the schematic rather than mame's sampling, so thin detail that MAME drops is kept (MAMEtesters 08884 / 6022).
- **Playfield scroll on line 0**: the game's per-line scroll table has an entry for line 0,
  and the core applies it to line 0. MAME draws line 0 before applying it (its partial update
  for the line before 0 is clamped to line 0 itself), so in MAME line 0 shows a stray strip of
  playfield from the previous frame's scroll - visible in Hydra as a line of canyon rock at the
  top of the sky.

## Building
Open `Arcade-AtariG1.qpf` in Quartus Prime 17.0 (the MiSTer standard). `lint_wrapper.sh` runs a
quick Verilator syntax check of the wrapper.

## Credits
Hardware behaviour from MAME (Aaron Giles and contributors, BSD-3-Clause). fx68k by Jorge
Cwik, verilog-6502 by Arlet Ottens, jt51 and jt6295 by Jose Tejada (jotego), SDRAM controller
by Sorgelig. MiSTer framework by Sorgelig and contributors. General santiy checking, accuracy-insistance, schematic-finding and Ai-bullying Nick Greenfield (FatNick) Ai copiloting and questionable use of other's hard work by Claude.
