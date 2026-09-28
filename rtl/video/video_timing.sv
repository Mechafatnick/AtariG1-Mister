// Atari G1 video timing — COMPLETE
// MAME: set_raw(14.318181MHz/2, 456, 0, 336, 262, 0, 240)
// 456x262 total, 336x240 visible, ~59.92Hz / 15.699kHz.
// Sync positions inside the blank interval are not specified by set_raw;
// the values below place hsync/vsync conventionally within blanking and can
// be tuned for analog output without affecting emulation.

module video_timing (
    input  logic        clk,        // system clock
    input  logic        cen_pix,    // 7.159 MHz pixel clock enable
    input  logic        reset,

    output logic [8:0]  hcnt,       // 0..455
    output logic [8:0]  vcnt,       // 0..261
    output logic        hblank,
    output logic        vblank,
    output logic        hsync,
    output logic        vsync,
    output logic        vblank_rise, // 1-clk pulse at start of vblank (68K IRQ1)
    output logic        scan8_tick   // 1-clk pulse every 8th scanline, line 0 first
);

    localparam H_TOTAL  = 456;
    localparam H_VIS    = 336;
    localparam HS_START = 372;   // tunable
    localparam HS_END   = 404;   // tunable
    localparam V_TOTAL  = 262;
    localparam V_VIS    = 240;
    localparam VS_START = 248;   // tunable
    localparam VS_END   = 251;   // tunable

    logic vblank_q;

    always_ff @(posedge clk) begin
        vblank_rise <= 1'b0;
        scan8_tick  <= 1'b0;

        if (reset) begin
            hcnt <= '0; vcnt <= '0; vblank_q <= 1'b0;
        end else if (cen_pix) begin
            if (hcnt == H_TOTAL-1) begin
                automatic logic [8:0] vnext =
                    (vcnt == V_TOTAL-1) ? 9'd0 : vcnt + 9'd1;
                hcnt <= '0;
                vcnt <= vnext;
                // scanline timer: every 8 lines starting at line 0
                if (vnext[2:0] == 3'd0) scan8_tick <= 1'b1;
            end else begin
                hcnt <= hcnt + 9'd1;
            end

            // Registered edge-detect on the vblank LEVEL. vblank_q tracks
            // the vblank level (updated every cen_pix), and vblank_rise is
            // a 1-clk pulse on the rising edge. This cannot drop pulses the
            // way the old stateful set/clear could (which starved the clear
            // and only fired ~1 frame in 7).
            vblank_q <= (vcnt >= V_VIS);
            if ((vcnt >= V_VIS) && !vblank_q)
                vblank_rise <= 1'b1;
        end
    end

    assign hblank = (hcnt >= H_VIS);
    assign vblank = (vcnt >= V_VIS);
    assign hsync  = (hcnt >= HS_START) && (hcnt < HS_END);
    assign vsync  = (vcnt >= VS_START) && (vcnt < VS_END);

endmodule
