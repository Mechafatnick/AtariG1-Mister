// =============================================================================
// Atari slapstic, SLAPSTIC-2 family (137412-111 to -118)
// =============================================================================
//
// Behaviour follows MAME's slapstic.cpp (Aaron Giles / O. Galibert,
// BSD-3-Clause). The slapstic is a bank-switching protection chip: it watches
// the 68000's address bus and selects which of four 8KB banks of program ROM
// appears in its window. Sequences of accesses to particular addresses move it
// between states and change the bank.
//
// The chip and its window are chosen at run time (from the MRA), so one core
// serves every game and revision on this board:
//   chip     0 = none (window reads ROM directly), 1..8 = 137412-111..-118
//   bootleg  the cheap replacement used on the Pit-Fighter bootleg
//   win_hi   window at 078000-07FFFF (Hydra) instead of 038000-03FFFF
//
// The chip sees EVERY bus cycle, not just accesses inside its window: the
// "alternate banking" sequence starts with an address matched anywhere in
// memory. Feed it one cs pulse per 68000 bus cycle with the full address.
//
// Tests (MAME terms; addresses are 14-bit word offsets A1..A14):
//   in range   the access is inside the window (with its mirrors)
//   reset      in range, word offset 0
//   bank[i]    in range, word offset = bank select value i
//   in(m,v)    in range and (offset & m) == v
//   any(m,v)   (offset & m) == v, anywhere on the bus
// States:
//   IDLE        reset -> ACTIVE
//   ACTIVE      bank[i] -> bank i, IDLE;  any(alt1) -> ALT_VALID;  in(add1) -> ADD_LOAD
//   ALT_VALID   reset -> ACTIVE;  in(alt2) -> ALT_SELECT;  in(add1) -> ADD_LOAD;  else ACTIVE
//   ALT_SELECT  reset -> ACTIVE;  any(alt3) -> remember bank from the address, ALT_COMMIT;  else ACTIVE
//   ALT_COMMIT  reset -> ACTIVE;  in(alt4) -> commit the remembered bank, IDLE  (otherwise wait)
//   ADD_LOAD    reset -> ACTIVE;  in(add2) -> remembered = current bank, ADD_SET;  else ACTIVE
//   ADD_SET     reset -> ACTIVE;  in(+1) -> remembered + 1;  in(+2) -> remembered + 2;
//               in(add3) -> ALT_COMMIT  (otherwise stay)
// =============================================================================

module slapstic (
    input  logic        clk,
    input  logic        reset,

    input  logic [3:0]  chip,        // 0 = none, 1..8 = 137412-111..-118
    input  logic        bootleg,     // Pit-Fighter bootleg replacement
    input  logic        win_hi,      // window at 078000 (Hydra) rather than 038000

    input  logic        cs,          // one pulse per 68000 bus cycle
    input  logic [23:0] addr,        // byte address of the access

    output logic [1:0]  bank
);

    // ---- chip tables: {mask, value} pairs as in MAME ----
    logic [15:0] bank0, bank1, bank2, bank3;
    logic [31:0] alt1, alt2, alt3, alt4, add1, add2, addp1, addp2, add3;
    logic [1:0]  altshift;

    // From MAME slapstic.cpp, slapstic111..118. Every chip in this family
    // starts in bank 0 and has no bitwise banking.
    always_comb begin
        bank0 = 16'h0000; bank1 = 16'h0000; bank2 = 16'h0000; bank3 = 16'h0000;
        alt1 = '0; alt2 = '0; alt3 = '0; alt4 = '0; altshift = 2'd0;
        add1 = '0; add2 = '0; addp1 = '0; addp2 = '0; add3 = '0;
        case (chip)
            4'd1: begin    // 137412-111
                bank0 = 16'h0042; bank1 = 16'h0052; bank2 = 16'h0062; bank3 = 16'h0072;
                alt1  = {16'h007f, 16'h000a}; alt2  = {16'h3fff, 16'h28a4};
                alt3  = {16'h0784, 16'h0080}; alt4  = {16'h3fcf, 16'h0042}; altshift = 2'd0;
                add1  = {16'h3fff, 16'h00a1}; add2  = {16'h3fff, 16'h00a2};
                addp1 = {16'h3c4f, 16'h284d}; addp2 = {16'h3a5f, 16'h285d}; add3 = {16'h3ff8, 16'h2800};
            end
            4'd2: begin    // 137412-112
                bank0 = 16'h002c; bank1 = 16'h003c; bank2 = 16'h006c; bank3 = 16'h007c;
                alt1  = {16'h007f, 16'h0014}; alt2  = {16'h3fff, 16'h29a0};
                alt3  = {16'h0073, 16'h0010}; alt4  = {16'h3faf, 16'h002c}; altshift = 2'd2;
                add1  = {16'h3fff, 16'h2dce}; add2  = {16'h3fff, 16'h2dcf};
                addp1 = {16'h3def, 16'h15e2}; addp2 = {16'h3fbf, 16'h15a2}; add3 = {16'h3ffc, 16'h1450};
            end
            4'd3: begin    // 137412-113
                bank0 = 16'h0008; bank1 = 16'h0018; bank2 = 16'h0028; bank3 = 16'h0038;
                alt1  = {16'h007f, 16'h0059}; alt2  = {16'h3fff, 16'h11a5};
                alt3  = {16'h0860, 16'h0800}; alt4  = {16'h3fcf, 16'h0008}; altshift = 2'd3;
                add1  = {16'h3fff, 16'h049b}; add2  = {16'h3fff, 16'h049c};
                addp1 = {16'h3fcf, 16'h3ec7}; addp2 = {16'h3edf, 16'h3ed7}; add3 = {16'h3fff, 16'h3fb2};
            end
            4'd4: begin    // 137412-114
                bank0 = 16'h0040; bank1 = 16'h0048; bank2 = 16'h0050; bank3 = 16'h0058;
                alt1  = {16'h007f, 16'h0016}; alt2  = {16'h3fff, 16'h24de};
                alt3  = {16'h3871, 16'h0000}; alt4  = {16'h3fe7, 16'h0040}; altshift = 2'd1;
                add1  = {16'h3fff, 16'h0ab7}; add2  = {16'h3fff, 16'h0ab8};
                addp1 = {16'h3f63, 16'h0d40}; addp2 = {16'h3fd9, 16'h0dc8}; add3 = {16'h3fff, 16'h0ab0};
            end
            4'd5: begin    // 137412-115
                bank0 = 16'h0020; bank1 = 16'h0022; bank2 = 16'h0024; bank3 = 16'h0026;
                alt1  = {16'h007f, 16'h0054}; alt2  = {16'h3fff, 16'h3e01};
                alt3  = {16'h3879, 16'h0029}; alt4  = {16'h3ff9, 16'h0020}; altshift = 2'd1;
                add1  = {16'h3fff, 16'h2591}; add2  = {16'h3fff, 16'h2592};
                addp1 = {16'h3fe6, 16'h3402}; addp2 = {16'h3fb4, 16'h3410}; add3 = {16'h3fff, 16'h34a2};
            end
            4'd6: begin    // 137412-116
                bank0 = 16'h0044; bank1 = 16'h004c; bank2 = 16'h0054; bank3 = 16'h005c;
                alt1  = {16'h007f, 16'h0069}; alt2  = {16'h3fff, 16'h2bab};
                alt3  = {16'h387c, 16'h0808}; alt4  = {16'h3fe7, 16'h0044}; altshift = 2'd0;
                add1  = {16'h3fff, 16'h3f7c}; add2  = {16'h3fff, 16'h3f7d};
                addp1 = {16'h3db2, 16'h3c12}; addp2 = {16'h3fe3, 16'h3e43}; add3 = {16'h3fff, 16'h2ba8};
            end
            4'd7: begin    // 137412-117
                bank0 = 16'h0008; bank1 = 16'h001a; bank2 = 16'h002c; bank3 = 16'h003e;
                alt1  = {16'h007f, 16'h007d}; alt2  = {16'h3fff, 16'h3580};
                alt3  = {16'h0079, 16'h0020}; alt4  = {16'h3fc9, 16'h0008}; altshift = 2'd1;
                add1  = {16'h3fff, 16'h0676}; add2  = {16'h3fff, 16'h0677};
                addp1 = {16'h3e62, 16'h1a42}; addp2 = {16'h3e35, 16'h1a11}; add3 = {16'h3fff, 16'h1a42};
            end
            4'd8: begin    // 137412-118
                bank0 = 16'h0014; bank1 = 16'h0034; bank2 = 16'h0054; bank3 = 16'h0074;
                alt1  = {16'h007f, 16'h0002}; alt2  = {16'h3fff, 16'h1950};
                alt3  = {16'h0067, 16'h0020}; alt4  = {16'h3f9f, 16'h0014}; altshift = 2'd3;
                add1  = {16'h3fff, 16'h1958}; add2  = {16'h3fff, 16'h1959};
                addp1 = {16'h3f73, 16'h3052}; addp2 = {16'h3f67, 16'h3042}; add3 = {16'h3ff8, 16'h30e0};
            end
            default: ;
        endcase
    end

    // ---- address tests ----
    // Window: 038000-039FFF or 078000-079FFF, mirrored every 2000 up to 3FFFF / 7FFFF.
    wire        in_range = win_hi ? (addr[23:15] == 9'b0_0000_1111)
                                  : (addr[23:15] == 9'b0_0000_0111);
    wire [15:0] wofs     = {2'b00, addr[14:1]};

    function automatic logic t_in(input logic [31:0] mv);
        return in_range && ((wofs & mv[31:16]) == mv[15:0]);
    endfunction
    function automatic logic t_any(input logic [31:0] mv);
        return ((wofs & mv[31:16]) == mv[15:0]);
    endfunction

    wire t_reset = in_range && (wofs == 16'h0000);
    wire t_bank0 = in_range && (wofs == bank0);
    wire t_bank1 = in_range && (wofs == bank1);
    wire t_bank2 = in_range && (wofs == bank2);
    wire t_bank3 = in_range && (wofs == bank3);

    // ---- state machine ----
    typedef enum logic [3:0] {
        S_IDLE, S_ACTIVE, S_ALT_VALID, S_ALT_SELECT, S_ALT_COMMIT,
        S_ADD_LOAD, S_ADD_SET
    } state_t;

    state_t     st;
    logic [1:0] loaded_bank;
    logic       bl_primed;          // bootleg: saw word offset 0

    always_ff @(posedge clk) begin
        if (reset) begin
            st          <= S_IDLE;
            bank        <= 2'd0;
            loaded_bank <= 2'd0;
            bl_primed   <= 1'b0;
        end else if (cs && chip != 4'd0) begin
            if (bootleg) begin
                // The bootleg's replacement chip only does simple banking:
                // an access to offset 0 primes it, and the next access picks
                // the bank from one of four offsets.
                if (in_range) begin
                    if (wofs == 16'h0000) bl_primed <= 1'b1;
                    else if (bl_primed) begin
                        bl_primed <= 1'b0;
                        case (wofs)
                            16'h0042: bank <= 2'd0;
                            16'h0052: bank <= 2'd1;
                            16'h0062: bank <= 2'd2;
                            16'h0072: bank <= 2'd3;
                            default : bl_primed <= 1'b1;
                        endcase
                    end
                end
            end else begin
                case (st)
                    S_IDLE: if (t_reset) st <= S_ACTIVE;

                    S_ACTIVE: begin
                        if      (t_bank0) begin bank <= 2'd0; st <= S_IDLE; end
                        else if (t_bank1) begin bank <= 2'd1; st <= S_IDLE; end
                        else if (t_bank2) begin bank <= 2'd2; st <= S_IDLE; end
                        else if (t_bank3) begin bank <= 2'd3; st <= S_IDLE; end
                        else if (t_any(alt1)) st <= S_ALT_VALID;
                        else if (t_in(add1))  st <= S_ADD_LOAD;
                    end

                    S_ALT_VALID: begin
                        if      (t_reset)    st <= S_ACTIVE;
                        else if (t_in(alt2)) st <= S_ALT_SELECT;
                        else if (t_in(add1)) st <= S_ADD_LOAD;
                        else                 st <= S_ACTIVE;
                    end

                    S_ALT_SELECT: begin
                        if (t_reset) st <= S_ACTIVE;
                        else if (t_any(alt3)) begin
                            // bank = (byte address >> (1 + shift)) & 3
                            loaded_bank <= 2'((addr >> (5'd1 + {3'd0, altshift})) & 24'h3);
                            st <= S_ALT_COMMIT;
                        end
                        else st <= S_ACTIVE;
                    end

                    S_ALT_COMMIT: begin
                        if (t_reset) st <= S_ACTIVE;
                        else if (t_in(alt4)) begin
                            bank <= loaded_bank;
                            st   <= S_IDLE;
                        end
                    end

                    S_ADD_LOAD: begin
                        if (t_reset) st <= S_ACTIVE;
                        else if (t_in(add2)) begin
                            loaded_bank <= bank;
                            st <= S_ADD_SET;
                        end
                        else st <= S_ACTIVE;
                    end

                    S_ADD_SET: begin
                        if      (t_reset)     st <= S_ACTIVE;
                        else if (t_in(addp1)) loaded_bank <= loaded_bank + 2'd1;
                        else if (t_in(addp2)) loaded_bank <= loaded_bank + 2'd2;
                        else if (t_in(add3))  st <= S_ALT_COMMIT;
                    end

                    default: st <= S_IDLE;
                endcase
            end
        end
    end

endmodule
