// =====================================================================
// Atari JSA II audio board — ported from MAME atarijsa.cpp (BSD-3-Clause,
// Aaron Giles). 6502 @ 1.789772 MHz + YM2151 @ 3.579545 MHz +
// OKI6295 @ 1.193182 MHz (pin 7 high at reset; runtime bit WRIO.3).
//
// 6502 memory map (atarijsa2_map — exact):
//   0000-1FFF  RAM (8KB)
//   2000-2001  YM2151 (r/w)
//   2800 (mirror 01F9)  R: OKI6295 read              /RDV
//   2802 (mirror 01F9)  R: sound command from 68000  /RDP  (clears NMI)
//   2804 (mirror 01F9)  R: I/O port                  /RDIO
//   2806 (mirror 01F9)  R/W: timed-IRQ ack           /IRQACK
//   2A00 (mirror 01F9)  W: OKI6295 write             /WRV
//   2A02 (mirror 01F9)  W: response to 68000         /WRP
//   2A04 (mirror 01F9)  W: WRIO                      /WRIO
//   2A06 (mirror 01F9)  W: MIX                       /MIX
//   3000-3FFF  banked ROM (bank = WRIO[7:6], from ROM offset bank*0x1000)
//   4000-FFFF  ROM (fixed, ROM offset 0x4000-0xFFFF)
//
// RDIO bits: 7=self-test (XOR'd with test_in cb, active low semantics),
//   6=main_to_sound ready (ACTIVE LOW), 5=sound_to_main ready,
//   4:3=+5V (read as 1), 2=coin3, 1=coin2, 0=coin1
//
// WRIO bits: 7:6=6502 ROM bank, 5=coin counter 2, 4=coin counter 1,
//   3=OKI pin7 (frequency), 2=OKI reset (active low), 0=YM2151 reset
//   (active low)
//
// MIX bits: 3:1=YM2151 volume 0-7 (linear /7), 0=OKI volume (1 ? 1.0 : 0.5)
//
// IRQs to 6502:
//   IRQ  = timed_int | ym2151_int.  timed_int set at 3.579545MHz/14336
//          (= /4/16/16/14 ≈ 249.7 Hz), cleared by any /IRQACK access.
//   NMI  = main_to_sound latch full (set on 68K command write, cleared
//          when 6502 reads 2802).
// Comm latches (atariscom behavior — verify against atariscom.cpp):
//   68K write F90000 → command latch, NMI, m2s_ready=1
//   6502 read 2802   → clears NMI + m2s_ready, returns latch
//   6502 write 2A02  → response latch, s2m_ready=1, 68K IRQ2 asserted
//   68K read FD0000  → clears s2m_ready/IRQ2, returns latch
//   68K write F98000 → sound board reset (also clears latches)
// =====================================================================

module jsa2 (
    input  logic        clk,
    input  logic        reset,          // system or sound_reset
    input  logic        cen_1m79,
    input  logic        cen_3m58,
    input  logic        cen_1m19,

    // ---- main CPU side ----
    input  logic [7:0]  main_cmd,
    input  logic        main_cmd_wr,
    output logic [7:0]  main_response,
    input  logic        main_resp_rd,
    input  logic        sound_reset,
    input  logic        fix_leak,       // v107: patch the driver's node leak (OSD)
    output logic        irq_to_main,
    output logic        m2s_ready,
    output logic        s2m_ready,
    input  logic        test_in,        // service switch (active low)
    input  logic        coin1, coin2,   // active high
    output logic        dbg_coin_seen,  // latched: 6502 read RDIO with a coin bit set
    output logic        dbg_cpu_alive,  // latched: the 6502 has fetched anything at all
    output logic        dbg_rdio_read,  // latched: the 6502 has read RDIO at all
    output logic        dbg_in_reset,   // live: JSA held in reset
    output logic        dbg_resp_sent,  // latched: 6502 wrote a response (2A02)
    output logic        dbg_resp_after_coin, // latched: response sent AFTER a coin was seen
    output logic        dbg_main_read,  // latched: 68000 read the response (FD0000)
    output logic        dbg_any_write,  // latched: 6502 performed ANY io write at all
    output logic        dbg_cmd_sent,   // latched: 68000 wrote a command to the sound board
    output logic        dbg_nmi_raised, // latched: the 6502 was NMI'd for that command
    output logic        dbg_cmd_read,   // latched: the 6502 actually read the command
    output logic        dbg_w_oki,      // latched: wrote 2A00 (OKI/voice)
    output logic        dbg_w_wrio,     // latched: wrote 2A04 (WRIO - banks/coin counters)
    output logic        dbg_w_mix,      // latched: wrote 2A06 (MIX)
    // YM2151 voice tracking. Register 0x08 is KEY ON/OFF: bits 6:3 enable the
    // four operators, bits 2:0 select the channel. A non-zero operator field is
    // a key-ON, all-zero is a key-OFF. If key-offs go missing, voices pile up
    // every time a tune loops - it gets louder and louder until all 8 channels
    // are stuck on and the driver has nowhere left to play.
    output logic [3:0]  dbg_voices,     // how many of the 8 channels are keyed on
    output logic        dbg_voice_max,  // latched: all 8 were on at once
    // Does the driver ARM a YM timer, and does jt51 ever RAISE the interrupt?
    // The sound driver releases notes on a timer interrupt; if that never
    // fires, note-offs never happen and the voices saturate.
    output logic        dbg_ym_t_armed, // latched: wrote reg 0x14 (timer ctl)
    output logic        dbg_ym_irq_ever,// latched: jt51 asserted irq_n
    output logic        dbg_ym_irq_now,
    // The music is paced by the JSA TIMED INTERRUPT (the driver never arms a
    // YM timer). Is it being delivered and serviced? If timed_int stays
    // asserted the 6502 is stuck in its handler; if it is raised far more
    // often than it is acknowledged, ticks are being lost.
    output logic        dbg_ti_now,     // timed_int asserted right now
    output logic        dbg_ti_stuck,   // latched: asserted for an abnormally long time
    output logic        dbg_ack_now,    // an IRQ acknowledge just happened
    output logic        dbg_ack_ever,
    // HEARTBEATS. A single-cycle pulse at 249.7Hz is invisible on screen, so
    // divide the interrupt and acknowledge streams down to something the eye
    // can see. If the stream stops, the heartbeat FREEZES - which is the whole
    // question: does the music die because its clock stops?
    output logic        dbg_ti_beat,    // toggles every 64 timed interrupts
    output logic        dbg_ack_beat,   // toggles every 64 acknowledges
    output logic        dbg_ymwr_beat,  // toggles every 64 YM writes
    output logic        dbg_ym_write,   // latched: wrote the YM2151
    output logic        dbg_ym_read,    // latched: READ the YM2151 (status polling)

    // ---- v73: READ THE DRIVER'S OWN VERDICT ----------------------------
    // The disassembly of 136081-1060.1b shows the sound driver diagnoses
    // itself into zero page, and gates the music on two single bytes.
    //
    //   $01  sequencer kill switch. The timed-IRQ handler opens with
    //          $41C5 lda $280E   ; ack
    //          $41C8 lda $01
    //          $41CA beq $41D7   ; zero -> run the sequencer
    //          $41CC inc $00     ; NON-ZERO -> tick and leave
    //        so a non-zero $01 stops every YM write while the interrupt
    //        still fires and is still acknowledged - exactly the v72
    //        heartbeat picture. Set to $FF at $40F0 during self-test,
    //        cleared at $4116 on entry to the main loop, never touched
    //        again. Non-zero after that means zero page was corrupted.
    //
    //   $02  the self-test status byte:
    //          b1 = YM2151 BUSY poll timed out ("MUSIC CHIP TIME OUT")
    //          b2 = timed interrupt never fired
    //          b3 = RAM fault, page >= 8
    //          b7 = ROM checksum mismatch
    //
    //   $0E  sticky "YM timed out" flag. Once bit 7 is set, the busy-wait
    //        at $4FC6 returns immediately for ever after.
    output logic [7:0]  dbg_zp01,        // live value of zero page $01
    output logic [7:0]  dbg_zp02_ever,   // sticky OR of every value written to $02
    output logic        dbg_zp01_bad,    // latched: $01 went non-zero AFTER init cleared it
    output logic        dbg_zp0e_to,     // latched: $0E written with bit7 set
    // PC snoops. The address bus is the 6502's program counter on an opcode
    // fetch, so we can watch for execution reaching a specific instruction.
    // Both addresses below are chosen to be immune to the 6502's branch-taken
    // over-fetch (which reads PC+2 and discards it), so they can only be seen
    // if that path genuinely executed.
    output logic        dbg_seq_skipped, // latched: reached $41CE (guard fired)
    output logic        dbg_ym_timeout,  // latched: reached $4FDB (busy poll gave up)
    output logic [7:0]  dbg_mix,         // v106: live MIX register (YM vol b3:1, OKI b0)
    output logic        dbg_cmd_race,    // latched: 68K command write collided with 6502 read
    output logic        dbg_resp_race,   // latched: 6502 response write collided with 68K read

    // ---- v75: measure the YM2151 BUSY handshake directly ----------------
    output logic        dbg_busy_stuck,  // latched: busy held >2x its design duration
    output logic [3:0]  dbg_busy_log,    // log2 of the longest busy burst seen
    output logic        dbg_ymrst_drop,  // latched: WRIO bit0 (YM reset) dropped after init
    output logic [3:0]  dbg_to_count,    // how many times the busy poll gave up

    // ---- v76: trace the music stream ------------------------------------
    // The sequencer walks its track through the zero-page pointer $06/$07
    // (lda ($06),y at $5017) and retires a channel at $5001, where a stream
    // byte of $BB or above writes $FF to $0248,x and returns with carry clear
    // - "this channel is finished". We freeze the pointer at that instant, so
    // the overlay shows WHERE in the stream the track decided it was over.
    output logic [15:0] dbg_seq_ptr,     // $06/$07 at the last channel retire
    output logic [3:0]  dbg_retire_cnt,  // how many channels have retired
    output logic        dbg_seq_beat,    // heartbeat: sequencer entered ($4FE3)

    // ---- v77: the sequencer's node pool ---------------------------------
    // Every $8D/$8E CALL allocates a 4-byte node from a free list whose head
    // index lives in zero page $1C, with the slots at $09AE. $8F returns the
    // node. If the pool ever runs dry, $42D8 returns 0 and $51DF SILENTLY
    // DROPS THE CALL - the stream runs on until it reads a byte >= $BB and
    // the channel retires. That is the music stopping.
    output logic        dbg_alloc_fail,  // latched: allocation failed ($42E6)
    output logic [7:0]  dbg_fl_head,     // live free-list head ($1C)
    output logic [7:0]  dbg_pool_bal,    // allocations minus frees (leak detector)
    output logic        dbg_pool_init,   // latched: pool initialiser ran ($42A7)
    output logic [7:0]  dbg_slot1,       // last value written to slot[1].next ($09AE)
    output logic        dbg_slot1_seen,  // latched: $09AE was written at all
    output logic [7:0]  dbg_snap_own,    // v107: YM voices that had an owner at the last silence
    output logic [7:0]  dbg_snap_act,    // v107: ...whose owner channel was active
    output logic [7:0]  dbg_snap_cov,    // v107: ...whose owner was covering another channel
    output logic [63:0] dbg_cmd_hist,    // v108: last 8 music/control commands before the last silence
    output logic [7:0]  dbg_snap_nch,    // v108: active channels at the last silence
    output logic [7:0]  dbg_snap_free,   // v108: free pool nodes at the last silence
    output logic [23:0] dbg_snd_rst,     // v109: {68000 sound resets, $02 before the last, 6502 addr hi then}

    // ---- v79: who put the free list into the failing state? -------------
    // Only six instructions write the list head $1C. Record which one ran
    // most recently, and freeze it - together with the head itself - at the
    // instant allocation fails, so the failure names its own cause.
    //   1 = $42A9 initialiser      4 = $4338 free loop (second chain)
    //   2 = $42E2 allocate         5 = $48EF (channel teardown)
    //   3 = $4320 free loop        6 = $5254 inlined free in the $8F handler
    output logic [3:0]  dbg_fail_who,    // last $1C writer before the failure
    output logic [7:0]  dbg_fail_head,   // head at the moment of failure
    output logic [3:0]  dbg_fail_cnt,    // how many allocation failures

    // ---- v80: does each OKI voice ever report itself idle? ---------------
    // The driver's voice manager at $5807 retires a voice only when
    //   $5812 lda $58A8,x   ; mask 01/02/04/08, one bit per voice
    //   $5815 and $2808     ; OKI status
    //   $5818 beq $582C     ; NOT busy -> ... jsr $58C8 (teardown)
    // takes that branch. A voice whose busy bit never clears is never torn
    // down, so its channel is never freed and its pool node leaks - which is
    // exactly the failure we have measured. These watch the status the CPU
    // actually sees, and whether the teardown ever runs.
    output logic [7:0]  dbg_oki_stat,    // live OKI status as the 6502 reads it
    output logic [3:0]  dbg_voice_idle,  // latched per voice: has EVER read idle
    output logic [3:0]  dbg_teardown,    // count of $58C8 teardown calls

    // ---- 6502 ROM (64KB BRAM, preloaded at ioctl time) ----
    output logic [15:0] cpu_rom_addr,
    input  logic [7:0]  cpu_rom_data,

    // ---- OKI ADPCM ROM (256KB, SDRAM or BRAM) ----
    output logic [17:0] oki_rom_addr,
    output logic        oki_rom_rd,
    input  logic [7:0]  oki_rom_data,
    input  logic        oki_rom_ready,

    output logic signed [15:0] audio_out
);

    wire rst = reset | sound_reset;

    // =================================================================
    // Registers
    // =================================================================
    logic [7:0] wrio, mix;
    logic [7:0] ym_regsel;
    logic [7:0] ym_voice_on;
    logic       voice_max_r;
    always_ff @(posedge clk) begin
        if (rst) begin
            ym_regsel <= 8'h00; ym_voice_on <= 8'h00; voice_max_r <= 1'b0;
        end else if (cpu_wr && sel_ym) begin
            if (!a[0]) ym_regsel <= do_d;                    // register select
            else if (ym_regsel == 8'h08) begin               // KEY ON/OFF
                ym_voice_on[do_d[2:0]] <= |do_d[6:3];
            end
        end
        if (&ym_voice_on) voice_max_r <= 1'b1;
    end
    assign dbg_voices    = 4'(ym_voice_on[0]) + 4'(ym_voice_on[1]) + 4'(ym_voice_on[2])
                         + 4'(ym_voice_on[3]) + 4'(ym_voice_on[4]) + 4'(ym_voice_on[5])
                         + 4'(ym_voice_on[6]) + 4'(ym_voice_on[7]);
    assign dbg_voice_max = voice_max_r;

    logic t_armed_r, ym_irq_ever_r;
    always_ff @(posedge clk) begin
        if (rst) begin t_armed_r <= 1'b0; ym_irq_ever_r <= 1'b0; end
        else begin
            if (cpu_wr && sel_ym && a[0] && (ym_regsel == 8'h14)) t_armed_r <= 1'b1;
            if (!ym_irq_n) ym_irq_ever_r <= 1'b1;
        end
    end
    assign dbg_ym_t_armed  = t_armed_r;
    assign dbg_ym_irq_ever = ym_irq_ever_r;
    assign dbg_ym_irq_now  = ~ym_irq_n;

    // timed_int should be raised at 249.7Hz and cleared promptly by the
    // handler. Holding it for more than ~2 interrupt periods means the 6502
    // is not servicing it.
    logic [17:0] ti_hold;
    logic        ti_stuck_r, ack_ever_r;
    always_ff @(posedge clk) begin
        if (rst) begin ti_hold <= '0; ti_stuck_r <= 1'b0; ack_ever_r <= 1'b0; end
        else begin
            if (!timed_int)            ti_hold <= '0;
            else if (!(&ti_hold))      ti_hold <= ti_hold + 1'd1;
            if (&ti_hold)              ti_stuck_r <= 1'b1;
            if (irq_ack)               ack_ever_r <= 1'b1;
        end
    end
    assign dbg_ti_now    = timed_int;
    assign dbg_ti_stuck  = ti_stuck_r;
    assign dbg_ack_now   = irq_ack;
    assign dbg_ack_ever  = ack_ever_r;

    logic [5:0] ti_div, ack_div, ymwr_div;
    logic       ti_beat_r, ack_beat_r, ymwr_beat_r;
    logic       ti_q;
    always_ff @(posedge clk) begin
        if (rst) begin
            ti_div <= '0; ack_div <= '0; ymwr_div <= '0;
            ti_beat_r <= 1'b0; ack_beat_r <= 1'b0; ymwr_beat_r <= 1'b0; ti_q <= 1'b0;
        end else begin
            ti_q <= timed_int;
            if (timed_int && !ti_q) begin                 // rising edge
                ti_div <= ti_div + 1'd1;
                if (&ti_div) ti_beat_r <= ~ti_beat_r;
            end
            if (irq_ack) begin
                ack_div <= ack_div + 1'd1;
                if (&ack_div) ack_beat_r <= ~ack_beat_r;
            end
            if (cpu_wr && sel_ym && a[0]) begin           // YM data writes
                ymwr_div <= ymwr_div + 1'd1;
                if (&ymwr_div) ymwr_beat_r <= ~ymwr_beat_r;
            end
        end
    end
    // =================================================================
    // v107 SILENCE SNAPSHOT
    // On hardware the music dies while the sequencer apparently keeps going;
    // the sound-board harness cannot reproduce it. So capture the driver's
    // voice bookkeeping at the moment the YM goes quiet.
    //   $0860[$1E+v] = owner channel of YM voice v (1-based, 0 = unowned)
    //   $03EC[ch]    = channel active (0 = free)
    //   $0860[ch]    = non-zero when the channel covers another; the voice
    //                  update at $4E51 then skips its register rewrite
    // The scanner reads these through RAM port B continuously, one voice at
    // a time, with the usual set-address / wait / read-data rhythm of a
    // registered BRAM. "Silent" = no YM data write for 2^24 clocks (293 ms).
    // At each onset of silence the last complete scan is latched.
    // v108: the scanner now makes three passes over sound RAM (through port
    // B, read-only): the 8 YM voice owners as before, a count of active
    // channels ($03EC..$0409 non-zero), and a walk of the node free list from
    // its head at $1C. It also keeps the last 8 music/control commands the
    // 6502 received (pings and OKI sample commands are skipped, see KEEP).
    // All of it is latched at each onset of silence.
    localparam logic [255:0] CMD_KEEP = 256'h000000000000000000001e0000000000240000000000001ffffffffffffffff7;
    logic [4:0]  sc_st;
    logic [2:0]  sc_v;
    logic [4:0]  sc_ch;
    logic [7:0]  sc_own, sc_idx, sc_cnt;
    logic [7:0]  own_live, act_live, cov_live;
    logic [7:0]  own_done, act_done, cov_done;
    logic [7:0]  own_snap, act_snap, cov_snap;
    logic [7:0]  nch_done, free_done, nch_snap, free_snap;
    logic [63:0] hist, hist_snap;
    logic [7:0]  zp02_live;
    logic [24:0] ym_idle;
    logic        silent_q;
    wire         silent = ym_idle[24];
    always_ff @(posedge clk) begin
        if (rst) begin
            sc_st <= 5'd0; sc_v <= 3'd0; sc_ch <= 5'd0; sc_addr <= 13'd0;
            sc_own <= 8'd0; sc_idx <= 8'd0; sc_cnt <= 8'd0;
            own_live <= 8'd0; act_live <= 8'd0; cov_live <= 8'd0;
            own_done <= 8'd0; act_done <= 8'd0; cov_done <= 8'd0;
            own_snap <= 8'd0; act_snap <= 8'd0; cov_snap <= 8'd0;
            nch_done <= 8'd0; free_done <= 8'd0; nch_snap <= 8'd0; free_snap <= 8'd0;
            hist <= 64'd0; hist_snap <= 64'd0;
            ym_idle <= 25'd0; silent_q <= 1'b0;
        end else begin
            case (sc_st)
                // ---- pass 1: YM voice owners ----
                5'd0: begin sc_addr <= 13'h087E + {10'd0, sc_v}; sc_st <= 5'd1; end
                5'd1: sc_st <= 5'd2;                       // RAM samples the address
                5'd2: begin                                // owner byte valid
                    sc_own <= sc_q;
                    own_live[sc_v] <= (sc_q != 8'd0);
                    if (sc_q == 8'd0) begin
                        act_live[sc_v] <= 1'b0; cov_live[sc_v] <= 1'b0; sc_st <= 5'd7;
                    end else begin
                        sc_addr <= 13'h03EC + {5'd0, sc_q} - 13'd1; sc_st <= 5'd3;
                    end
                end
                5'd3: sc_st <= 5'd4;
                5'd4: begin
                    act_live[sc_v] <= (sc_q != 8'd0);
                    sc_addr <= 13'h0860 + {5'd0, sc_own} - 13'd1; sc_st <= 5'd5;
                end
                5'd5: sc_st <= 5'd6;
                5'd6: begin cov_live[sc_v] <= (sc_q != 8'd0); sc_st <= 5'd7; end
                5'd7: begin
                    sc_v <= sc_v + 3'd1;
                    if (sc_v == 3'd7) begin
                        own_done <= own_live; act_done <= act_live; cov_done <= cov_live;
                        sc_ch <= 5'd0; sc_cnt <= 8'd0; sc_st <= 5'd8;
                    end else sc_st <= 5'd0;
                end
                // ---- pass 2: count active channels ----
                5'd8:  begin sc_addr <= 13'h03EC + {8'd0, sc_ch}; sc_st <= 5'd9; end
                5'd9:  sc_st <= 5'd10;
                5'd10: begin
                    if (sc_q != 8'd0) sc_cnt <= sc_cnt + 8'd1;
                    if (sc_ch == 5'd29) sc_st <= 5'd11;
                    else begin sc_ch <= sc_ch + 5'd1; sc_st <= 5'd8; end
                end
                // ---- pass 3: walk the free list ----
                5'd11: begin nch_done <= sc_cnt; sc_addr <= 13'h001C; sc_st <= 5'd12; end
                5'd12: sc_st <= 5'd13;
                5'd13: begin sc_idx <= sc_q; sc_cnt <= 8'd0; sc_st <= 5'd14; end
                5'd14: begin
                    if (sc_idx == 8'd0 || sc_cnt == 8'd255) begin
                        free_done <= sc_cnt; sc_st <= 5'd17;
                    end else begin
                        sc_addr <= 13'h09AE + {3'd0, sc_idx - 8'd1, 2'b00}; sc_st <= 5'd15;
                    end
                end
                5'd15: sc_st <= 5'd16;
                5'd16: begin sc_idx <= sc_q; sc_cnt <= sc_cnt + 8'd1; sc_st <= 5'd14; end
                // ---- pass 4 (v109): live copy of the driver status byte $02 ----
                // (deliberately NOT cleared by rst, so the value from just
                // before a sound-CPU reset survives into the capture below)
                5'd17: begin sc_addr <= 13'h0002; sc_st <= 5'd18; end
                5'd18: sc_st <= 5'd19;
                5'd19: begin zp02_live <= sc_q; sc_st <= 5'd0; end
                default: sc_st <= 5'd0;
            endcase

            // command history: the same read that clears nmi_pending
            if (cen_1m79 && sel_cmd_r && cpu_rd && CMD_KEEP[cmd_latch])
                hist <= {hist[55:0], cmd_latch};

            if (cpu_wr && sel_ym && a[0]) ym_idle <= 25'd0;
            else if (!silent)             ym_idle <= ym_idle + 25'd1;
            silent_q <= silent;
            if (silent && !silent_q) begin
                own_snap  <= own_done; act_snap <= act_done; cov_snap <= cov_done;
                hist_snap <= hist; nch_snap <= nch_done; free_snap <= free_done;
            end
        end
    end
    assign dbg_snap_own  = own_snap;
    assign dbg_snap_act  = act_snap;
    assign dbg_snap_cov  = cov_snap;
    assign dbg_cmd_hist  = hist_snap;
    assign dbg_snap_nch  = nch_snap;
    assign dbg_snap_free = free_snap;

    // v109 SOUND-CPU RESET RECORDER. The game runs its own sound watchdog
    // (68000 $14640): every 32 frames it sends command $07, which replies
    // with the driver status byte $02 and then sets bits 0 and 2 of it; the
    // main loop clears bit 0, every timer IRQ clears bit 2, a YM busy-poll
    // timeout sets bit 1. One frame later the game checks the reply, and if
    // it has not arrived or (reply & 7) != 0 it RESETS THE SOUND CPU - which
    // reinitialises every channel, so the music dies while effects carry on.
    // Record each such reset, with the status byte and the 6502's address
    // just before it. Cleared only by a SYSTEM reset. The boot sequence
    // resets the sound CPU once, so a count of 1 is normal.
    logic [7:0] rst_cnt, rst_zp02, rst_pchi;
    always_ff @(posedge clk) begin
        if (reset) begin
            rst_cnt <= 8'd0; rst_zp02 <= 8'd0; rst_pchi <= 8'd0;
        end else if (sound_reset) begin
            if (rst_cnt != 8'hFF) rst_cnt <= rst_cnt + 8'd1;
            rst_zp02 <= zp02_live;
            rst_pchi <= a_held[15:8];
        end
    end
    assign dbg_snd_rst = {rst_cnt, rst_zp02, rst_pchi};

    // v107: CHANNEL STEP heartbeat. $4727 is the first instruction of the
    // per-channel step inside $471C and follows a two-byte instruction, so
    // unlike the old $4FE3 probe (which sat after an RTS and fired on the
    // 6502 core's prefetch of the following byte) it only counts real steps.
    logic [5:0] step_div;
    logic       step_beat_r;
    always_ff @(posedge clk) begin
        if (rst) begin step_div <= 6'd0; step_beat_r <= 1'b0; end
        else if (cen_1m79 && a_held == 16'h4727) begin
            step_div <= step_div + 6'd1;
            if (&step_div) step_beat_r <= ~step_beat_r;
        end
    end

    assign dbg_ti_beat   = ti_beat_r;
    assign dbg_ack_beat  = ack_beat_r;
    assign dbg_ymwr_beat = ymwr_beat_r;
    logic [7:0] cmd_latch, resp_latch;
    logic       nmi_pending;            // == m2s full
    logic       timed_int, ym_irq_n;

    assign m2s_ready     = nmi_pending;
    assign s2m_ready     = irq_to_main;
    assign main_response = resp_latch;

    // =================================================================
    // Timed interrupt: cen_3m58 / 14336 ≈ 249.7 Hz
    // =================================================================
    logic [13:0] tictr;
    logic        irq_ack;

    logic tick;
    always_ff @(posedge clk) begin
        if (rst) begin
            tictr <= '0; timed_int <= 1'b0;
        end else begin
            // A timer expiry and an IRQ acknowledge can land on the SAME clock.
            // Previously the ack came last and silently destroyed the interrupt
            // that had just been raised, so the 6502 quietly lost ticks - and
            // the music sequencer runs off this interrupt. Real hardware latches
            // the new interrupt, so a coincident expiry must WIN over the ack.
            tick = 1'b0;
            if (cen_3m58) begin
                if (tictr == 14'd14335) begin
                    tictr <= '0;
                    tick  = 1'b1;
                end else tictr <= tictr + 14'd1;
            end
            if (tick)         timed_int <= 1'b1;
            else if (irq_ack) timed_int <= 1'b0;
        end
    end

    // =================================================================
    // 6502 — T65 core
    // =================================================================
    logic [7:0]  cpu_di, cpu_do;
    logic        cpu_rw_n;   // 1 = read

    // Arlet Ottens' Verilog 6502. Chosen over the VHDL T65 so that the
    // identical core is used in synthesis AND in Verilator simulation —
    // the sound board was previously the one block that could never be
    // simulated, which is exactly where the game turned out to hang.
    // RDY gates the core to the 1.789 MHz enable; DI must be valid on the
    // clock the CPU consumes it, which holds because the address is
    // stable for the whole 32-cycle enable period.
    wire [15:0] cpu_ab;
    wire        cpu_we;

    // DI is registered before entering the core: it breaks the
    // combinational path Verilator cannot settle (UNOPTFLAT on DIMUX),
    // and is harmless because AB is stable for the whole 32-cycle enable
    // -----------------------------------------------------------------
    // 6502 bus interface (Arlet Ottens core) on block RAM.
    //
    // The core drives AB combinationally (AB = {DIMUX, ADD}) and, at each
    // RDY edge, commits the address for the cycle it is starting. We run
    // one 6502 cycle per cen_1m79 (32 fast clocks apart). Registering AB
    // AT the RDY edge captures the committed address cleanly (avoiding the
    // mid-cycle {DIMUX,ADD} transients that corrupt vector fetches); the
    // registered BRAM answers one clock later, and DI is combinational
    // from that result, stable for the rest of the cycle and valid at the
    // next RDY edge where the core samples it. Using the same held address
    // for BRAM and decode keeps data and read-mux select in agreement and
    // breaks the DI -> DIMUX -> AB -> decode -> DI combinational loop.
    // -----------------------------------------------------------------
    cpu u_cpu (
        .clk   (clk),
        .reset (rst),
        .AB    (cpu_ab),
        .DI    (cpu_di),
        .DO    (cpu_do),
        .WE    (cpu_we),
        .IRQ   (timed_int | ~ym_irq_n),
        .NMI   (nmi_pending),
        .RDY   (cen_1m79)
    );

    assign cpu_rw_n = ~cpu_we;

    // Free-running position counter within the 6502 cycle. cen_1m79 marks
    // the RDY edge (position 0). AB is combinational and fully settled by
    // late in the cycle, so latch it near the end; the registered BRAM
    // then answers with margin before the next RDY edge, where the core
    // samples DI. Latching this late (not early) avoids the transient
    // {DIMUX, ADD} states AB passes through mid-cycle on vector/indirect
    // fetches — which is what made the reset vector read back byte-swapped.
    // Position within the now-regular /32 6502 cycle (cen_1m79 at pos 0).
    // Latch AB near the end of the cycle, when it is fully settled, so the
    // BRAM answers before the core samples DI at the next edge.
    // AB is combinational and settles at the start of each 6502 cycle,
    // staying stable until the next RDY edge (verified in sim). The core
    // samples DI at that same RDY edge and expects the byte for the
    // address currently on AB — i.e. asynchronous-memory behaviour.
    //
    // So the memories are addressed from cpu_ab DIRECTLY: with 32 fast
    // clocks per 6502 cycle, the registered BRAM output settles hundreds
    // of nanoseconds before the RDY edge, giving async-equivalent timing
    // without a combinational memory. The only thing that must not depend
    // combinationally on cpu_ab is the read MUX select feeding DI back
    // toward AB, so a one-clock registered address copy (a) is used for
    // the decode terms, breaking DI -> DIMUX -> AB -> decode -> DI while
    // leaving the data itself current.
    // Single registered address used for BOTH memory addressing and decode
    // so the BRAM data and the read-mux select always correspond to the
    // same address. `a` updates one clock after the RDY edge (when AB has
    // settled); the BRAM answers one clock after that; DI is then stable
    // for the ~29 remaining clocks until the next RDY edge, where the core
    // samples it. This also breaks the DI->DIMUX->AB->decode->DI loop.
    // Register AB at the RDY edge and hold it for the whole 6502 cycle.
    // The core presents the next address combinationally right at the RDY
    // edge, so capturing on that edge grabs the committed address (not the
    // mid-cycle {DIMUX,ADD} transients). BRAM + decode both use this held
    // copy; DI is combinational from the result and is valid long before
    // the following RDY edge where the core samples it.
    logic [15:0] a_held;
    always_ff @(posedge clk) if (cen_1m79) a_held <= cpu_ab;
    wire [15:0] a     = a_held;
    wire [15:0] mem_a = a_held;

    // Arlet's core presents AB combinationally from DIMUX, and on the
    // clocks between RDY pulses DIMUX == DIHOLD (the last byte latched at
    // the previous RDY). So AB is only guaranteed correct AT the RDY edge.
    // Registering DI so the core sees, at each RDY edge, the byte fetched
    // for the address it presented at the PREVIOUS RDY edge matches the
    // synchronous-memory model the core's testbench uses.
    wire cpu_rd =  cpu_rw_n;

    // WRITE TIMING FIX
    // a_held captures cpu_ab ON the cen_1m79 edge, so during that edge it
    // still holds the PREVIOUS cycle's address. cpu_wr/cpu_do, however, are
    // the CURRENT cycle's. Firing the write on that same edge therefore
    // stored the new data at the OLD address - every JSR pushed its return
    // address one slot adrift, so every RTS came back to garbage. Straight
    // line code survived; anything using the stack did not.
    // Delay the write by one 6502 cycle so enable+data line up with a_held.
    logic       we_d;
    logic [7:0] do_d;
    always_ff @(posedge clk) if (cen_1m79) begin
        we_d <= cpu_we;
        do_d <= cpu_do;
    end
    wire cpu_wr = we_d & cen_1m79;

    // decode (mirrors folded: 2800-29FF reads / 2A00-2BFF writes decode
    // on a[2:1]; mirror mask 0x01F9 means a[8:3] and a[0] are don't-care)
    // JSA II 6502 map (MAME atarijsa.cpp). The I/O registers at 0x2800/2/
    // 4/6 (reads) and 0x2A00/2/4/6 (writes) each have .mirror(0x01F9),
    // meaning address bits 0, 3, 4, 5, 6, 7 and 8 are DON'T-CARE and the
    // register is chosen by bits [2:1] alone. The game relies on this: it
    // reads the command via 0x280A and the IRQ-ack via 0x280E, which are
    // mirror aliases of 0x2802 and 0x2806. The previous decode forced
    // bits [8:3]==0 and so never matched those aliases — the sound CPU
    // could never read its command, which is why the handshake hung.
    wire sel_ram   = (a[15:13] == 3'b000);                 // 0000-1FFF
    wire sel_ym    = (a[15:11] == 5'b00100) && !a[10];     // 2000-27FF→2000-2001
    wire sel_io_r  = (a[15:13] == 3'b001) && (a[12:9] == 4'b0100)
                   && !a[10];                              // 2800-29FF (+mirrors)
    wire sel_io_w  = (a[15:13] == 3'b001) && (a[12:9] == 4'b0101)
                   && !a[10];                              // 2A00-2BFF (+mirrors)
    wire sel_rd    = sel_io_r;
    wire sel_wr    = sel_io_w;
    wire sel_bank  = (a[15:12] == 4'h3);                   // 3000-3FFF
    wire sel_rom   = (a[15:14] != 2'b00) && !sel_bank
                   && (a >= 16'h4000);                     // 4000-FFFF

    // register select = bits [2:1] only (mirror mask 0x01F9)
    wire sel_oki_r  = sel_rd && (a[2:1] == 2'b00);   // 2800 /RDV
    wire sel_cmd_r  = sel_rd && (a[2:1] == 2'b01);   // 2802 /RDP
    wire sel_rdio   = sel_rd && (a[2:1] == 2'b10);   // 2804 /RDIO
    wire sel_irqack = sel_rd && (a[2:1] == 2'b11);   // 2806 /IRQACK
    wire sel_oki_w  = sel_wr && (a[2:1] == 2'b00);   // 2A00 /WRV
    wire sel_resp_w = sel_wr && (a[2:1] == 2'b01);   // 2A02 /WRP
    wire sel_wrio   = sel_wr && (a[2:1] == 2'b10);   // 2A04 /WRIO
    wire sel_mix    = sel_wr && (a[2:1] == 2'b11);   // 2A06 /MIX

    assign irq_ack = cen_1m79 && ((sel_irqack && cpu_rd) || (sel_irqack && !cpu_rw_n));

    // v107: port B of the sound RAM feeds the silence-snapshot scanner
    logic [12:0] sc_addr;
    logic [7:0]  sc_q;
    // v107: driver-fix patch signals (declared ahead of the read mux)
    logic        fix_r;
    logic        patch_hit;
    logic [7:0]  patch_byte;

    // sound RAM (explicit dpram for guaranteed M10K inference)
    logic [7:0] ram_q;
    dpram #(.AW(13), .DW(8)) u_ram (
        .clk(clk),
        .addr_a(mem_a[12:0]), .d_a(do_d),
        .we_a(cpu_wr && sel_ram), .q_a(ram_q),
        .addr_b(sc_addr), .d_b(8'd0), .we_b(1'b0), .q_b(sc_q)
    );

    // ROM addressing: banked 3000-3FFF, fixed 4000-FFFF
    assign cpu_rom_addr = sel_bank ? {2'b00, wrio[7:6], mem_a[11:0]}
                                   : mem_a;

    // =================================================================
    // v73 PROBES - the driver's own self-diagnosis
    // =================================================================
    // Shadow the three zero-page bytes the music depends on by snooping the
    // sound RAM write port. cpu_wr/do_d/mem_a are already mutually aligned
    // (see the v23 write-timing fix), so this sees exactly what the RAM sees.
    logic [7:0] zp01_r, zp02_ever_r;
    logic       zp01_armed_r, zp01_bad_r, zp0e_to_r;
    // NOTE: cleared on the GLOBAL reset only, not on `rst` (= reset |
    // sound_reset). The 68000 can reset the sound board mid-game, and if these
    // latches followed that they would erase the evidence we are trying to
    // photograph.
    //
    // v74 PROBE FIX. In v73 this read RED throughout, which was my bug, not the
    // core's. The boot self-test legitimately writes $01 = $FF at $40F0, and
    // zp01_armed_r once armed never disarmed - so the SECOND init (any sound
    // reset from the 68000) saw that legitimate $FF and tripped the corruption
    // latch. Disarming whenever the sound board is reset makes the latch mean
    // what it claims: $01 went non-zero after the main loop cleared it.
    always_ff @(posedge clk) begin
        if (reset) begin
            zp01_r       <= 8'h00;
            zp02_ever_r  <= 8'h00;
            zp01_armed_r <= 1'b0;
            zp01_bad_r   <= 1'b0;
            zp0e_to_r    <= 1'b0;
        end else if (rst) begin
            // sound board re-entering its self-test: stand the latch down, but
            // KEEP zp01_bad_r / zp02_ever_r / zp0e_to_r so evidence survives
            zp01_armed_r <= 1'b0;
        end else if (cpu_wr && sel_ram) begin
            case (mem_a[12:0])
                13'h0001: begin
                    zp01_r <= do_d;
                    // $01 is legitimately $FF through the boot self-test, so
                    // only arm the corruption latch once the main loop has
                    // cleared it at $4116. After that, ANY non-zero write is
                    // the music being switched off.
                    if (do_d == 8'h00)          zp01_armed_r <= 1'b1;
                    else if (zp01_armed_r)      zp01_bad_r   <= 1'b1;
                end
                // both gated past the self-test, same reason as the PC snoops
                13'h0002: if (zp01_armed_r) zp02_ever_r <= zp02_ever_r | do_d;
                13'h000E: if (do_d[7] && zp01_armed_r) zp0e_to_r <= 1'b1;
                default: ;
            endcase
        end
    end
    assign dbg_zp01      = zp01_r;
    assign dbg_zp02_ever = zp02_ever_r;
    assign dbg_zp01_bad  = zp01_bad_r;
    assign dbg_zp0e_to   = zp0e_to_r;

    // PC snoops. a_held is the address the 6502 committed for this bus cycle,
    // so sampling it at cen_1m79 catches the fetch. $41CE is the second
    // instruction of the "sequencer skipped" path and $4FDB the second of the
    // "busy poll gave up" path - one past the branch target in each case, so
    // the branch-taken over-fetch of PC+2 cannot trigger them spuriously.
    //
    // v74: both are now gated on zp01_armed_r, which is set the moment the
    // main loop writes $01 = 0 at $4116. Without that gate they latched during
    // the BOOT SELF-TEST, where both paths run legitimately:
    //   - $41CE executes every boot, because the self-test deliberately sets
    //     $01 = $FF at $40F0 and then enables interrupts at $40FD to check the
    //     handler is alive; the guard path IS the expected path there.
    //   - $4FDB executes because the driver holds the YM2151 in reset via
    //     WRIO bit 0 at $4007/$400C/$4011, so a busy poll in that window is
    //     supposed to time out.
    // Latching those told us nothing except that the core boots correctly.
    logic seq_skipped_r, ym_timeout_r;
    always_ff @(posedge clk) begin
        if (reset) begin            // global reset only - see note above
            seq_skipped_r <= 1'b0; ym_timeout_r <= 1'b0;
        end else if (cen_1m79 && zp01_armed_r) begin
            // No read qualifier: both addresses are in the fixed ROM region,
            // so they can never appear as a write target. Adding cpu_rd would
            // mix the CURRENT cycle's R/W with a_held's PREVIOUS-cycle address
            // - the same misalignment the v23 write fix exists to avoid.
            if (a_held == 16'h41CE) seq_skipped_r <= 1'b1;
            if (a_held == 16'h4FDB) ym_timeout_r  <= 1'b1;
        end
    end
    assign dbg_seq_skipped = seq_skipped_r;
    assign dbg_ym_timeout  = ym_timeout_r;
    assign dbg_mix         = mix;

    // v74: direct evidence that the race this build fixes is real. These latch
    // when a 68000 access lands on the exact clk as the matching 6502 access -
    // the collision that used to drop the event. With the reordering above
    // nothing is lost when they fire, so RED here is now "the race happened and
    // was handled", not "something broke". If cmd_race lights on a continue and
    // the music no longer stops, that is the fix confirmed end to end.
    logic cmd_race_r, resp_race_r;
    always_ff @(posedge clk) begin
        if (reset) begin
            cmd_race_r <= 1'b0; resp_race_r <= 1'b0;
        end else begin
            if (main_cmd_wr && cen_1m79 && sel_cmd_r && cpu_rd) cmd_race_r  <= 1'b1;
            if (main_resp_rd && cpu_wr && sel_resp_w)           resp_race_r <= 1'b1;
        end
    end
    assign dbg_cmd_race  = cmd_race_r;
    assign dbg_resp_race = resp_race_r;

    // latch proof that the SOUND CPU actually sampled a coin
    // Diagnostic ladder, so we can tell WHICH stage fails:
    //   alive -> is the 6502 running at all?
    //   rdio  -> does it ever poll the coin port?
    //   coin  -> does a coin ever coincide with that poll?
    // These latch and never clear (except on reset) so a photo can't miss them.
    logic coin_seen_r = 1'b0, cpu_alive_r = 1'b0, rdio_read_r = 1'b0;
    assign dbg_coin_seen = coin_seen_r;
    assign dbg_cpu_alive = cpu_alive_r;
    assign dbg_rdio_read = rdio_read_r;
    assign dbg_in_reset  = reset;
    always_ff @(posedge clk) begin
        if (reset) begin
            coin_seen_r <= 1'b0; cpu_alive_r <= 1'b0; rdio_read_r <= 1'b0;
        end else begin
            if (cen_1m79)                 cpu_alive_r <= 1'b1;
            if (cen_1m79 && sel_rdio && cpu_rd) rdio_read_r <= 1'b1;
            if (cen_1m79 && sel_rdio && cpu_rd && (coin1 | coin2)) coin_seen_r <= 1'b1;
        end
    end

    // ---- v15: the comm path from "6502 saw a coin" to "68000 got told" ----
    logic resp_sent_r = 1'b0, resp_after_coin_r = 1'b0, main_read_r = 1'b0, any_write_r = 1'b0;
    logic cmd_sent_r = 1'b0, nmi_raised_r = 1'b0, cmd_read_r = 1'b0;
    logic w_oki_r=1'b0, w_wrio_r=1'b0, w_mix_r=1'b0, ym_wr_r=1'b0, ym_rd_r=1'b0;
    assign dbg_resp_sent       = resp_sent_r;
    assign dbg_resp_after_coin = resp_after_coin_r;
    assign dbg_main_read       = main_read_r;
    assign dbg_any_write       = any_write_r;
    assign dbg_cmd_sent        = cmd_sent_r;
    assign dbg_nmi_raised      = nmi_raised_r;
    assign dbg_cmd_read        = cmd_read_r;
    assign dbg_w_oki           = w_oki_r;
    assign dbg_w_wrio          = w_wrio_r;
    assign dbg_w_mix           = w_mix_r;
    assign dbg_ym_write        = ym_wr_r;
    assign dbg_ym_read         = ym_rd_r;
    always_ff @(posedge clk) begin
        if (reset) begin
            resp_sent_r <= 1'b0; resp_after_coin_r <= 1'b0; main_read_r <= 1'b0; any_write_r <= 1'b0;
            cmd_sent_r <= 1'b0; nmi_raised_r <= 1'b0; cmd_read_r <= 1'b0;
            w_oki_r<=1'b0; w_wrio_r<=1'b0; w_mix_r<=1'b0; ym_wr_r<=1'b0; ym_rd_r<=1'b0;
        end else begin
            // NOTE: must use cpu_wr, exactly as the real response write at the
            // "if (cpu_wr && sel_resp_w)" block below does. A previous version
            // used (cen_1m79 && !cpu_rw_n), which never coincided, so this
            // latch stayed dark and wrongly implied the 6502 was silent.
            if (cpu_wr && sel_resp_w) begin
                resp_sent_r <= 1'b1;
                if (coin_seen_r) resp_after_coin_r <= 1'b1;
            end
            if (main_resp_rd) main_read_r <= 1'b1;
            if (cpu_wr && sel_wr) any_write_r <= 1'b1;   // any 2A0x write
            // --- 68000 -> 6502 direction, each mirroring the real code exactly ---
            if (main_cmd_wr)                          cmd_sent_r   <= 1'b1;  // as line "if (main_cmd_wr)"
            if (main_cmd_wr)                          nmi_raised_r <= 1'b1;  // that same line sets nmi_pending
            if (cen_1m79 && sel_cmd_r && cpu_rd)      cmd_read_r   <= 1'b1;  // as the NMI-clear line
            // which of the four 2A0x registers does it actually write?
            if (cpu_wr && sel_oki_w)  w_oki_r  <= 1'b1;
            if (cpu_wr && sel_wrio)   w_wrio_r <= 1'b1;
            if (cpu_wr && sel_mix)    w_mix_r  <= 1'b1;
            // and does it talk to / poll the YM2151?
            if (cpu_wr && sel_ym)                 ym_wr_r <= 1'b1;
            if (cen_1m79 && sel_ym && cpu_rd)     ym_rd_r <= 1'b1;
        end
    end

    // RDIO  (MAME jsa_ii_ioports, exact):
    //   b0 COIN1 active high      b1 COIN2 active high    b2 COIN3 active high
    //   b3 UNUSED  -> reads 0     b4 UNUSED  -> reads 0
    //   b5 sound_to_main_ready active high
    //   b6 main_to_sound_ready active LOW
    //   b7 self test active high
    // b3/b4 were previously hardwired to 1 ("+5V"). MAME declares them
    // IP_ACTIVE_HIGH/IPT_UNUSED, and since IP_ACTIVE_HIGH is 0 the default
    // value is 0 - same wrong assumption that had COIN3 stuck high.
    wire [7:0] rdio = { test_in,                 // b7 self-test (active high)
                        ~nmi_pending,            // b6 in-full   (active low)
                        irq_to_main,             // b5 out-full  (active high)
                        2'b00,                   // b4,b3 unused - MUST read 0
                        1'b0,                    // b2 coin3 - not wired
                        coin2, coin1 };          // b1,b0 coins (active high)

    // read mux
    logic [7:0] ym_dout, oki_dout;
    always_comb begin
        cpu_di = 8'hFF;
        if      (sel_ram)    cpu_di = ram_q;
        else if (sel_ym)     cpu_di = ym_dout;
        else if (sel_oki_r)  cpu_di = oki_dout;
        else if (sel_cmd_r)  cpu_di = cmd_latch;
        else if (sel_rdio)   cpu_di = rdio;
        else if (sel_irqack) cpu_di = 8'h00;
        else if (patch_hit)  cpu_di = patch_byte;          // v107 driver fix
        else if (sel_bank || sel_rom) cpu_di = cpu_rom_data;
    end

    // =================================================================
    // v107 SOUND DRIVER LEAK FIX (optional, OSD "Sound driver fix")
    // Atari's own bug. When a new FM sound takes a voice by priority, the
    // steal at $45A7-$45B2 wipes the previous owner with JSR $4668, which
    // zeroes that channel's call/loop stack heads ($0734/$0752,x) WITHOUT
    // returning their nodes to the pool. The driver's other steal path
    // ($461D) calls the proper free, JSR $430B, first. Jingles loop
    // internally, so every jingle interrupted mid-loop loses up to 8 of the
    // 134 nodes, until the music's own loops can no longer be pushed and the
    // track plays out and stops.
    // Fix: redirect that JSR to a 9-byte trampoline in unused ROM ($FC5C;
    // $FC5B-$FFF9 is all zero and unreferenced) that frees first:
    //   $45B0  JSR $FC5C            (was JSR $4668)
    //   $FC5C  PHP / SEI / JSR $430B / PLP / JMP $4668
    // $430B takes the channel in X (which the steal has already set), keeps
    // Y, and only clobbers A, which $4668 reloads. Interrupts are held off so
    // the IRQ-side sequencer cannot touch the free list mid-way. The switch
    // is sampled at reset so it can never change under a running trampoline.
    // =================================================================
    // v108: DISABLED. The trampoline also ran on the NORMAL allocation path
    // ($4567 BEQ $45AE lands on the same JSR), and after a command $00 reset
    // ($420D rebuilds the pool but leaves stale stack heads) it freed nodes
    // that were already back on the free list - a double free that silenced
    // the music outright in the harness. The game's own $00 before each track
    // keeps the pool healthy anyway, so the original driver is kept.
    assign fix_r = 1'b0;
    wire   unused_fix_leak = fix_leak;
    always_comb begin
        patch_hit = 1'b0; patch_byte = 8'h00;
        if (fix_r && sel_rom) begin
            patch_hit = 1'b1;
            case (a)
                16'h45B1: patch_byte = 8'h5C;   // JSR $FC5C (low)
                16'h45B2: patch_byte = 8'hFC;   //           (high)
                16'hFC5C: patch_byte = 8'h08;   // PHP
                16'hFC5D: patch_byte = 8'h78;   // SEI
                16'hFC5E: patch_byte = 8'h20;   // JSR $430B
                16'hFC5F: patch_byte = 8'h0B;
                16'hFC60: patch_byte = 8'h43;
                16'hFC61: patch_byte = 8'h28;   // PLP
                16'hFC62: patch_byte = 8'h4C;   // JMP $4668
                16'hFC63: patch_byte = 8'h68;
                16'hFC64: patch_byte = 8'h46;
                default:  patch_hit  = 1'b0;
            endcase
        end
    end

    // =================================================================
    // Communication latches
    // =================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            nmi_pending <= 1'b0; irq_to_main <= 1'b0;
            cmd_latch <= 8'h00; resp_latch <= 8'h00;
        end else begin
            // v74 RACE FIX - the same defect as the v63 timed-interrupt race.
            //
            // These were written set-first, clear-second. A non-blocking
            // assignment takes the LAST value assigned, so when the 68000's
            // command write landed on the same clk as the 6502's read of
            // $2802, the read wiped the NEW command's NMI: cmd_latch held the
            // new byte but no NMI ever fired, so the 6502 never came to
            // collect it and the command was lost for ever.
            //
            // Ordering clear-first / set-second makes the ARRIVING event win,
            // which is also what the real hardware does: the write sets the
            // flip-flop, the read clears it, and a simultaneous set+clear
            // leaves it set. The 6502 then reads the old byte on this cycle,
            // sees NMI still asserted, and comes straight back for the new
            // one. Nothing is dropped.
            //
            // This matters most on CONTINUE, where the 68000 sends a burst of
            // commands (stop track, start new track) while the 6502 is still
            // servicing the previous NMI - the one moment a write is likely to
            // collide with a read. Losing the "start" after the "stop" leaves
            // the sequencer believing the track ended: YM writes cease while
            // the interrupt, acknowledge and CPU heartbeats all keep running.
            if (cen_1m79 && sel_cmd_r && cpu_rd) nmi_pending <= 1'b0;
            if (main_cmd_wr) begin
                cmd_latch   <= main_cmd;
                nmi_pending <= 1'b1;
            end

            // Identical race in the other direction: a 6502 response write
            // coinciding with the 68000's read of $FD0000 would drop the
            // response IRQ. Same fix, same reason.
            if (main_resp_rd) irq_to_main <= 1'b0;
            if (cpu_wr && sel_resp_w) begin
                resp_latch  <= do_d;
                irq_to_main <= 1'b1;
            end
        end
    end

    // =================================================================
    // WRIO / MIX
    // =================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            wrio <= 8'h00; mix <= 8'h00;
        end else begin
            if (cpu_wr && sel_wrio) wrio <= do_d;
            if (cpu_wr && sel_mix)  mix  <= do_d;
        end
    end

    // =================================================================
    // YM2151 (jt51) — reset also via WRIO bit0 (active low)
    // =================================================================
    logic signed [15:0] ym_l, ym_r;
    // =================================================================
    // v75 YM HANDSHAKE MEASUREMENT
    // =================================================================
    // The driver polls the YM2151 BUSY bit before every register write and
    // gives up after 255 polls (~1.85ms). jt51 should release BUSY after 32
    // cen_3m58 cycles (~8.9us), so a timeout should be impossible - yet the
    // timeout path demonstrably executes on hardware. These measure the chip
    // rather than reason about it.
    //
    // busy_run counts cen_3m58 cycles while BUSY is asserted. Design value is
    // 32, so anything past 64 is wrong. busy_log keeps the log2 of the longest
    // burst so a single number says how far out we are: 5 = 32 (correct),
    // 6 = 64, ... 15 = 32768 cycles (~9ms, i.e. effectively stuck).
    // BUSY is bit 7 of what the chip returns on a read - the same bit the
    // driver tests with "bit $2001" - so we measure exactly what it sees.
    wire ym_busy = ym_dout[7];

    // Priority encoder: highest set bit of the current run length.
    logic [15:0] busy_run;
    logic [3:0]  busy_top;
    always_comb begin
        busy_top = 4'd0;
        for (int b = 0; b < 16; b++) if (busy_run[b]) busy_top = 4'(b);
    end

    logic [3:0] busy_log_r;
    logic       busy_stuck_r;
    always_ff @(posedge clk) begin
        if (reset) begin
            busy_run <= 16'd0; busy_log_r <= 4'd0; busy_stuck_r <= 1'b0;
        end else if (cen_3m58) begin
            if (ym_busy) begin
                if (busy_run != 16'hFFFF) busy_run <= busy_run + 16'd1;
                if (busy_run > 16'd64)    busy_stuck_r <= 1'b1;
                if (busy_top > busy_log_r) busy_log_r <= busy_top;
            end else begin
                busy_run <= 16'd0;
            end
        end
    end
    assign dbg_busy_stuck = busy_stuck_r;
    assign dbg_busy_log   = busy_log_r;

    // Did WRIO bit 0 - the YM2151 RESET line - ever drop after the main loop
    // started? The driver sets WRIO = $05 once at $411E and every later write
    // preserves bit 0, so a drop means OUR write path glitched it. That would
    // wipe every instrument register mid-track, which is what "the instruments
    // hadn't reset" sounds like.
    logic ymrst_drop_r;
    always_ff @(posedge clk) begin
        if (reset) ymrst_drop_r <= 1'b0;
        else if (zp01_armed_r && !wrio[0]) ymrst_drop_r <= 1'b1;
    end
    assign dbg_ymrst_drop = ymrst_drop_r;

    // How many times did the busy poll give up? Once is a startup artefact;
    // many means it is happening throughout play. Saturates at 15.
    logic [3:0] to_count_r;
    always_ff @(posedge clk) begin
        if (reset) to_count_r <= 4'd0;
        else if (cen_1m79 && zp01_armed_r && a_held == 16'h4FDB
                 && to_count_r != 4'd15) to_count_r <= to_count_r + 4'd1;
    end
    assign dbg_to_count = to_count_r;

    // =================================================================
    // v76 MUSIC STREAM TRACE
    // =================================================================
    logic [7:0] zp06_r, zp07_r;
    always_ff @(posedge clk) begin
        if (reset) begin zp06_r <= 8'h00; zp07_r <= 8'h00; end
        else if (cpu_wr && sel_ram) begin
            if (mem_a[12:0] == 13'h0006) zp06_r <= do_d;
            if (mem_a[12:0] == 13'h0007) zp07_r <= do_d;
        end
    end

    logic [15:0] seq_ptr_r;
    logic [3:0]  retire_cnt_r;
    logic [5:0]  seq_div;
    logic        seq_beat_r;
    always_ff @(posedge clk) begin
        if (reset) begin
            seq_ptr_r <= 16'h0000; retire_cnt_r <= 4'd0;
            seq_div <= 6'd0; seq_beat_r <= 1'b0;
        end else if (cen_1m79 && zp01_armed_r) begin
            // channel retired - freeze the stream pointer as it stands now.
            // Overwrites each time, so this is always the MOST RECENT retire,
            // which is the one that silenced the music.
            if (a_held == 16'h5001) begin
                seq_ptr_r <= {zp07_r, zp06_r};
                if (retire_cnt_r != 4'd15) retire_cnt_r <= retire_cnt_r + 4'd1;
            end
            // sequencer entry heartbeat: divide by 64 so it flickers visibly
            // and FREEZES if the sequencer stops being called at all.
            if (a_held == 16'h4FE3) begin
                seq_div <= seq_div + 6'd1;
                if (seq_div == 6'd63) seq_beat_r <= ~seq_beat_r;
            end
        end
    end
    assign dbg_seq_ptr    = seq_ptr_r;
    assign dbg_retire_cnt = retire_cnt_r;
    assign dbg_seq_beat   = step_beat_r;   // v107: was $4FE3 (prefetch artefact)

    // =================================================================
    // v77 NODE POOL MEASUREMENT
    // =================================================================
    logic [7:0] fl_head_r;
    always_ff @(posedge clk) begin
        if (reset) fl_head_r <= 8'h00;
        else if (cpu_wr && sel_ram && mem_a[12:0] == 13'h001C) fl_head_r <= do_d;
    end
    assign dbg_fl_head = fl_head_r;

    // Balance: +1 on a successful allocation ($42E4), -1 on each free - the
    // inlined one in the $8F handler ($5249) and the shared routine ($430B).
    // A healthy track oscillates. A leak climbs and never comes back down.
    // v78: the free at $430B walks a CHAIN (jmp $430d / jmp $4325), so the
    // v77 version counting one decrement per CALL was wrong and its reading
    // was meaningless. Count the per-node relink sites instead - $4320 and
    // $4338 inside the loop, and $5254 for the inlined free in the $8F
    // handler - so every node returned is counted exactly once.
    logic [7:0] pool_bal_r;
    logic       alloc_fail_r, pool_init_r;
    always_ff @(posedge clk) begin
        if (reset) begin
            pool_bal_r <= 8'h00; alloc_fail_r <= 1'b0; pool_init_r <= 1'b0;
        end else if (cen_1m79 && zp01_armed_r) begin
            if (a_held == 16'h42E6) alloc_fail_r <= 1'b1;
            if (a_held == 16'h42A7) pool_init_r  <= 1'b1;
            if (a_held == 16'h42E4 && pool_bal_r != 8'hFF)
                pool_bal_r <= pool_bal_r + 8'd1;
            else if ((a_held == 16'h4320 || a_held == 16'h4338
                      || a_held == 16'h5254) && pool_bal_r != 8'h00)
                pool_bal_r <= pool_bal_r - 8'd1;
        end
    end
    assign dbg_pool_init = pool_init_r;

    // Watch slot[1].next at $09AE directly. The initialiser must write $02
    // here. If nothing ever writes it, the pool was never built and our
    // all-zero BRAM is the pool - which is not what real hardware gives.
    logic [7:0] slot1_r; logic slot1_seen_r;
    always_ff @(posedge clk) begin
        if (reset) begin slot1_r <= 8'h00; slot1_seen_r <= 1'b0; end
        else if (cpu_wr && sel_ram && mem_a[12:0] == 13'h09AE) begin
            slot1_r <= do_d; slot1_seen_r <= 1'b1;
        end
    end
    // Track the most recent writer of $1C, then freeze it when $42D8 fails.
    logic [3:0] lastw_r, fail_who_r, fail_cnt_r;
    logic [7:0] fail_head_r;
    always_ff @(posedge clk) begin
        if (reset) begin
            lastw_r <= 4'd0; fail_who_r <= 4'd0;
            fail_head_r <= 8'h00; fail_cnt_r <= 4'd0;
        end else if (cen_1m79 && zp01_armed_r) begin
            case (a_held)
                16'h42A9: lastw_r <= 4'd1;   // initialiser
                16'h42E2: lastw_r <= 4'd2;   // allocate
                16'h4320: lastw_r <= 4'd3;   // free loop, chain 1
                16'h4338: lastw_r <= 4'd4;   // free loop, chain 2
                16'h48EF: lastw_r <= 4'd5;   // channel teardown
                16'h5254: lastw_r <= 4'd6;   // inlined free in $8F
                default: ;
            endcase
            if (a_held == 16'h42E6) begin   // allocation failed
                fail_who_r  <= lastw_r;
                fail_head_r <= fl_head_r;
                if (fail_cnt_r != 4'd15) fail_cnt_r <= fail_cnt_r + 4'd1;
            end
        end
    end
    // v80 OKI voice lifecycle
    logic [3:0] voice_idle_r, teardown_r;
    always_ff @(posedge clk) begin
        if (reset) begin voice_idle_r <= 4'd0; teardown_r <= 4'd0; end
        else begin
            // oki_dout[3:0] is the per-voice busy nibble. Record, per voice,
            // that it has been seen LOW at least once since reset.
            voice_idle_r <= voice_idle_r | ~oki_dout[3:0];
            if (cen_1m79 && zp01_armed_r && a_held == 16'h58C8
                && teardown_r != 4'd15) teardown_r <= teardown_r + 4'd1;
        end
    end
    assign dbg_oki_stat   = oki_dout;
    assign dbg_voice_idle = voice_idle_r;
    assign dbg_teardown   = teardown_r;

    assign dbg_fail_who  = fail_who_r;
    assign dbg_fail_head = fail_head_r;
    assign dbg_fail_cnt  = fail_cnt_r;

    assign dbg_slot1      = slot1_r;
    assign dbg_slot1_seen = slot1_seen_r;
    assign dbg_alloc_fail = silent;        // v107: was $42E6 (prefetch artefact)
    assign dbg_pool_bal   = pool_bal_r;

    // v110 YM2151 RESET STRETCH.
    // jt51 keeps every per-operator register (DT1/MUL, TL, KS/AR, AMS/D1R,
    // DT2/D2R, D1L/RR) and the envelope/key-on state in 32-stage shift
    // registers clocked by cen_p1 (= cen_1m79 here), and a slot is only
    // cleared as it passes the reset tap - so reset must be held for 32
    // cen_p1 cycles to clear the whole chip (see jt51_sh.v). The 6502 pulses
    // WRIO bit 0 low for just ~6 cycles at boot, and after a 68000 sound reset
    // the chip is held for ~20 more: measured in the harness, 6 of the 32
    // operator slots kept the previous tune's settings. MAME's wrio_w calls
    // ym2151->reset(), which clears everything at once, so stretch the reset
    // to 64 cen_p1 cycles after the last request.
    wire       ym_rst_req = rst | ~wrio[0];
    logic [6:0] ym_rst_ctr = 7'd0;
    always_ff @(posedge clk) begin
        if (ym_rst_req)                         ym_rst_ctr <= 7'd0;
        else if (cen_1m79 && !ym_rst_ctr[6])    ym_rst_ctr <= ym_rst_ctr + 7'd1;
    end
    wire ym_rst = ym_rst_req | ~ym_rst_ctr[6];

    jt51 u_ym (
        .rst    (ym_rst),
        .clk    (clk),
        .cen    (cen_3m58),
        .cen_p1 (cen_1m79),      // cen/2 — reuse the 6502 enable (same rate)
        .cs_n   (~(sel_ym && cen_1m79)),
        // A0 selects the YM2151's two ports: 2000 = register select,
        // 2001 = data. The bit that distinguishes them is the 6502's a[0].
        // This was wired to a[1], so BOTH addresses presented a0=0 and every
        // write went to the register-select port - the data port was never
        // written at all, so no voice parameters ever reached the chip.
        // That is why the sound test produced only pops and clicks.
        .wr_n   (~we_d),         // delayed, to match a_held (see the write fix above)
        .a0     (a[0]),
        .din    (do_d),          // delayed, to match a_held
        .dout   (ym_dout),
        .irq_n  (ym_irq_n),
        .xleft  (ym_l),
        .xright (ym_r),
        .sample (), .left(), .right(), .ct1(), .ct2()
    );

    // =================================================================
    // OKI6295 (jt6295) — reset via WRIO bit2 (active low);
    // pin7 runtime-switchable via WRIO bit3 (Pit-Fighter uses high)
    // =================================================================
    // The 6502 write strobe is ONE clk_sys cycle wide (cpu_wr = we_d & cen_1m79,
    // i.e. div32==0), but jt6295 only samples wrn when cen_1m19 fires - and
    // cen_1m19 comes from a SEPARATE counter (cen_ctr 0..47). div32==0 and
    // cen_ctr==0 coincide only every 96 clocks, so most OKI writes were simply
    // never seen by the chip. Hold the strobe and the data until the chip has
    // had a chance to sample them.
    logic       oki_wr_pend;
    logic [7:0] oki_wdata;
    always_ff @(posedge clk) begin
        if (rst) begin
            oki_wr_pend <= 1'b0; oki_wdata <= 8'h00;
        end else begin
            if (sel_oki_w && cpu_wr) begin
                oki_wr_pend <= 1'b1;
                oki_wdata   <= cpu_do;
            end else if (cen_1m19) begin
                oki_wr_pend <= 1'b0;   // the chip has sampled it this cycle
            end
        end
    end

    logic signed [13:0] oki_snd;
    // INTERPOL(0): the 4x upsampling FIR (INTERPOL 1/2) instantiates
    // jtframe_fir_mono from jotego's jtframe library, which we don't
    // vendor. Raw output is authentic-enough OKI; to upgrade later,
    // vendor jtframe/hdl/sound/jtframe_fir_mono.v (+ jtframe_fir and
    // deps) and flip this back to 1 — the coefficient hex files are
    // already present in rtl/ext/jt6295/.
    jt6295 #(.INTERPOL(0)) u_oki (
        .rst    (rst | ~wrio[2]),
        .clk    (clk),
        .cen    (cen_1m19),
        .ss     (wrio[3]),       // pin 7
        .wrn    (~oki_wr_pend),
        .din    (oki_wdata),
        .dout   (oki_dout),
        .rom_addr(oki_rom_addr),
        .rom_data(oki_rom_data),
        .rom_ok (oki_rom_ready),
        .sound  (oki_snd),
        .sample ()
    );
    assign oki_rom_rd = 1'b1;

    // =================================================================
    // Mixer: ym_vol = MIX[3:1]/7, oki_vol = MIX[0] ? 1.0 : 0.5
    // Implemented as integer gains: ym * vol * 2/7ths approximated via
    // multiply; oki shifted into range. JSA II is MONO.
    // =================================================================
    wire signed [15:0] ym_mono = (ym_l >>> 1) + (ym_r >>> 1);
    wire        [2:0]  ymv     = mix[3:1];
    wire signed [19:0] ym_scaled  = ym_mono * $signed({1'b0, ymv});   // /7 below
    wire signed [15:0] ym_out     = 16'(ym_scaled / 7);
    wire signed [15:0] oki_full   = {oki_snd, 2'b00};
    wire signed [15:0] oki_out    = mix[0] ? oki_full : (oki_full >>> 1);

    // v98: SATURATE, DON'T WRAP. ym_out and oki_out are each full-scale 16-bit,
    // and the old 16-bit sum WRAPPED whenever both were loud together: a wave
    // swinging hard positive flipped to hard negative, which is the harsh
    // "splat" at the start of the victory music (fanfare + crowd at once).
    // Real hardware mixes the two chips through resistors in the analog domain
    // and can at worst clip; MAME sums at 0.60/0.75 and clamps the final mix.
    // Sum in 17 bits (cannot overflow), then clamp to the 16-bit rails.
    // Both operands are explicitly signed - see v92 for what happens otherwise.
    wire signed [16:0] mix_sum = $signed({ym_out[15],  ym_out})
                               + $signed({oki_out[15], oki_out});
    assign audio_out = (mix_sum >  17'sd32767) ?  16'sd32767 :
                       (mix_sum < -17'sd32768) ? -16'sd32768 :
                                                 mix_sum[15:0];

endmodule
