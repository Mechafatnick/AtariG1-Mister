// =============================================================================
// NVRAM (EEPROM) save requests
// =============================================================================
//
// hps_io saves the EEPROM to the SD card (the MRA's <nvram index="2">) when
// save_req rises. Two triggers:
//   (a) the OSD opening - the MiSTer convention, and a way to force a save
//   (b) automatically, once the game's EEPROM writes have been quiet for about
//       2.3 s (2^27 clocks). The game rewrites its EEPROM in a burst at
//       power-on and afterwards only occasionally (statistics, high scores),
//       so this saves each burst once without hammering the SD card.
//
// Never in the middle of a record. The game's EEPROM driver keeps its data in
// 15-byte records (10 data bytes and an error-correcting code), each with a
// backup copy, and writes them one byte per frame. A copy saved part-way
// through fails its check at the next power-on: the game repairs it from the
// other copy, but adds the failed bits to the ERROR COUNT in its statistics
// (and shows "EEPROM ERROR" once that passes 75), as a real board does after
// a power cut in the middle of a write. So:
//   - (a) waits until the driver has been idle for 2^22 clocks (73 ms, about
//     4 frames; while it has records queued it writes every frame)
//   - a write between a save request and the end of that upload may be in the
//     file half-done, so the core saves again once the driver is idle
//   - a download (the EEPROM being loaded) cancels anything pending
// =============================================================================

module nv_save #(
    parameter int AUTO_BITS = 27,       // (b): 2^27 clocks = 2.3 s at 57.27 MHz
    parameter int IDLE_BITS = 22        // (a): 2^22 clocks = 73 ms
) (
    input  logic clk,
    input  logic ee_write,              // the game wrote an EEPROM byte (one-clock pulse)
    input  logic osd,                   // OSD_STATUS
    input  logic download,              // ioctl_download
    input  logic upload,                // ioctl_upload
    output logic save_req               // to hps_io's ioctl_upload_req (a pulse)
);

    logic                 dirty  = 1'b0;    // written since the last save: (b)
    logic                 pend   = 1'b0;    // save once the driver is idle: (a)
    logic                 saving = 1'b0;    // from a request to the end of its upload
    logic [AUTO_BITS-1:0] quiet  = '0;      // clocks since the game's last write (saturates)
    logic                 osd_d  = 1'b0, upload_d = 1'b0;
    logic                 req    = 1'b0;
    assign save_req = req;

    wire idle = (quiet >= (AUTO_BITS'(1) << IDLE_BITS));

    always_ff @(posedge clk) begin
        osd_d    <= osd;
        upload_d <= upload;
        req      <= 1'b0;

        if (ee_write) begin
            dirty <= 1'b1;
            quiet <= '0;
            if (saving) pend <= 1'b1;       // that save may hold this record half-written
        end else if (!(&quiet)) quiet <= quiet + 1'b1;

        if (osd && !osd_d) pend <= 1'b1;
        if (upload_d && !upload) saving <= 1'b0;

        if (download) begin                 // the EEPROM is being loaded
            pend  <= 1'b0;
            dirty <= 1'b0;
        end else if (!upload && !ee_write && ((pend && idle) || (dirty && &quiet))) begin
            req      <= 1'b1;
            saving   <= 1'b1;
            pend     <= 1'b0;
            dirty    <= 1'b0;
        end
    end

endmodule
