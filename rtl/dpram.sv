// True dual-port RAM, registered outputs, both ports read/write.
//
// Coded as Intel's canonical WRITE-FIRST true-dual-port template.
// Do not "simplify" this into `q <= mem[addr]` beside the write:
// that is read-OLD-data semantics, which Cyclone V M10Ks cannot do in
// true dual-port mode — Quartus then silently implements the RAM as
// registers + mux trees (the 2KB object-list RAM became 54,656 ALUTs
// and overflowed the device by 25%). The no_rw_check attribute also
// tells Quartus not to add duplicate RAMs / bypass logic to preserve
// read-during-write ordering, which was doubling M10K usage on the
// single-write instances.
//
// Consequence: q_x returns NEW data when we_x writes the same address
// that cycle. No user of this module reads and writes the same address
// in the same cycle, so this is behaviorally transparent here.
//
// DEPTH may be set below 2^AW for a RAM whose top addresses are never used;
// Quartus then allocates only the block RAMs the words need (an M10K holds
// 1K words of up to 10 bits, so depth is what counts).
module dpram #(
    parameter int AW = 10,
    parameter int DW = 16,
    parameter int DEPTH = (1 << AW),  // words actually present (addresses 0..DEPTH-1)
    parameter     INIT_FILE = ""      // optional $readmemh preload (small tables only)
)(
    input  logic          clk,

    input  logic [AW-1:0] addr_a,
    input  logic [DW-1:0] d_a,
    input  logic          we_a,
    output logic [DW-1:0] q_a,

    input  logic [AW-1:0] addr_b,
    input  logic [DW-1:0] d_b,
    input  logic          we_b,
    output logic [DW-1:0] q_b
);

    (* ramstyle = "no_rw_check" *) logic [DW-1:0] mem [0:DEPTH-1];

    initial begin
        if (INIT_FILE != "") $readmemh(INIT_FILE, mem);
    end

    always_ff @(posedge clk) begin
        if (we_a) begin
            mem[addr_a] <= d_a;
            q_a         <= d_a;
        end else
            q_a <= mem[addr_a];
    end

    always_ff @(posedge clk) begin
        if (we_b) begin
            mem[addr_b] <= d_b;
            q_b         <= d_b;
        end else
            q_b <= mem[addr_b];
    end

endmodule
