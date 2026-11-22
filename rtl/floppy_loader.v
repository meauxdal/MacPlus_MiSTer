// Mount-time floppy image loader: on img_mounted the whole image is streamed
// in via sd_rd, sector by sector, to the SDRAM offsets the read side expects
// (dskReadAddrInt/Ext in addrController_top.v). Each sector is staged in a
// local BRAM, then drained to SDRAM one word at a time through the shared
// extra-slot-3 port. `done` fires only once the whole image is resident.
module floppy_loader
(
	input         clk_sys,
	input         reset,

	input         img_mounted,  // this slot's one-shot mount pulse
	input  [63:0] img_size,     // valid at img_mounted
	input         img_readonly, // valid at img_mounted

	output reg [31:0] sd_lba,
	output reg        sd_rd,
	input              sd_ack,

	input        [7:0] sd_buff_addr,
	input       [15:0] sd_buff_dout,
	input               sd_buff_wr,

	// shared extra-slot-3 SDRAM write port (see addrController_top.v)
	output reg  [21:0] wr_addr, // byte offset within THIS image, word-aligned
	output reg  [15:0] wr_data,
	output reg          wr_req,
	input                wr_ack,

	output reg          done,          // one clk_sys pulse: image now fully resident
	output reg  [63:0]  loaded_size,   // img_size, latched at this slot's own mount
	output reg           readonly_latched,

	// medium sidedness from the volume header, latched with `done`
	output reg          media_ds,

	// the image is a MOOF bitstream (floppy_moof.v plays it), latched with
	// `done`; read-only for now, so it also sets readonly_latched
	output reg          is_moof,
	output              busy
);

localparam IDLE         = 3'd0,
           SD_ASSERT    = 3'd1,
           SD_WAIT_ACK  = 3'd2,
           SD_WAIT_DONE = 3'd3,
           DRAIN_FETCH  = 3'd4,
           DRAIN_ASSERT = 3'd5,
           DRAIN_WAIT   = 3'd6,
           DONE_PULSE   = 3'd7;
reg [2:0] state;

assign busy = (state != IDLE);

reg [15:0] buf_mem [0:255];
reg [15:0] buf_rd;
always @(posedge clk_sys) buf_rd <= buf_mem[word_idx];

reg  [7:0] word_idx;   // 0..255 within the current sector
reg [12:0] sector;     // sector index within the image (up to 4096)
reg [12:0] nsect;      // total sectors, latched at mount

// latch every mount request; one arriving during a drain must not be dropped
reg mount_pending;
always @(posedge clk_sys) begin
	if (reset) mount_pending <= 1'b0;
	else if (img_mounted && img_size != 0) mount_pending <= 1'b1;
	else if (state == IDLE && mount_pending) mount_pending <= 1'b0;
end

// medium sniff: volume size from the Master Directory Block in sector 2
// (drNmAlBlks * drAlBlkSiz, MFS and HFS alike); no MDB = double-sided
localparam [15:0] MDB_SIG_MFS = 16'hD2D7;
localparam [15:0] MDB_SIG_HFS = 16'h4244;
// volume size in 512-byte blocks, midway between 800 and 1600
localparam [23:0] SIDEDNESS_THRESHOLD = 24'd1200;

wire [15:0] mdb_word = {sd_buff_dout[7:0], sd_buff_dout[15:8]}; // as staged above
wire        mdb_wr   = (state == SD_WAIT_DONE) && sd_buff_wr && sd_ack &&
                       (sector == 13'd2);
wire        sniff_rst = reset || (state == IDLE && mount_pending); // this mount's first cycle

reg [15:0] mdb_sig;     // word 0:      drSigWord
reg [15:0] mdb_nalbk;   // word 9:      drNmAlBlks
reg [15:0] mdb_absz_h;  // words 10-11: drAlBlkSiz, big-endian
reg [15:0] mdb_absz_l;
reg        mdb_seen;    // sector 2 went by, so the four words above are this image's

always @(posedge clk_sys) begin
	if (sniff_rst) mdb_seen <= 1'b0;
	else if (mdb_wr) begin
		case (sd_buff_addr)
		8'd0:  mdb_sig    <= mdb_word;
		8'd9:  mdb_nalbk  <= mdb_word;
		8'd10: mdb_absz_h <= mdb_word;
		8'd11: begin mdb_absz_l <= mdb_word; mdb_seen <= 1'b1; end
		default: ;
		endcase
	end
end

// drAlBlkSiz is a non-zero multiple of 512, well under 64K on a floppy
wire mdb_ok = mdb_seen &&
              ((mdb_sig == MDB_SIG_MFS) || (mdb_sig == MDB_SIG_HFS)) &&
              (mdb_absz_h == 16'd0) && (mdb_absz_l != 16'd0) &&
              (mdb_absz_l[8:0] == 9'd0) && (mdb_nalbk != 16'd0);

// drNmAlBlks * (drAlBlkSiz / 512), shift-add over seven cycles
reg [23:0] vol_blocks;
reg [23:0] mul_cand;
reg  [6:0] mul_mult;
reg  [2:0] mul_step;
reg        mul_busy;

always @(posedge clk_sys) begin
	if (sniff_rst) begin
		mul_busy   <= 1'b0;
		vol_blocks <= 24'd0;
	end
	else if (mdb_wr && sd_buff_addr == 8'd11) begin
		vol_blocks <= 24'd0;
		mul_cand   <= {8'd0, mdb_nalbk};
		mul_mult   <= mdb_word[15:9];
		mul_step   <= 3'd0;
		mul_busy   <= 1'b1;
	end
	else if (mul_busy) begin
		if (mul_mult[0]) vol_blocks <= vol_blocks + mul_cand;
		mul_cand <= {mul_cand[22:0], 1'b0};
		mul_mult <= {1'b0, mul_mult[6:1]};
		mul_step <= mul_step + 3'd1;
		if (mul_step == 3'd6) mul_busy <= 1'b0;
	end
end

// MOOF sniff: the 8-byte signature "MOOF" FF 0A 0D 0A, then INFO at byte 20
// (+1 disk type: 1 = 400K GCR, 2 = 800K GCR; +2 write protected). Words
// arrive little-endian: the low byte is the even file offset.
wire       sig_wr = (state == SD_WAIT_DONE) && sd_buff_wr && sd_ack &&
                    (sector == 13'd0);
reg  [3:0] sig_ok;     // one bit per signature word
reg  [7:0] moof_type;

always @(posedge clk_sys) begin
	if (sniff_rst) sig_ok <= 4'd0;
	else if (sig_wr) begin
		case (sd_buff_addr)
		8'd0:  sig_ok[0] <= (sd_buff_dout == 16'h4F4D); // "MO"
		8'd1:  sig_ok[1] <= (sd_buff_dout == 16'h464F); // "OF"
		8'd2:  sig_ok[2] <= (sd_buff_dout == 16'h0AFF);
		8'd3:  sig_ok[3] <= (sd_buff_dout == 16'h0A0D);
		8'd10: moof_type <= sd_buff_dout[15:8];         // byte 21
		default: ;
		endcase
	end
end

wire moof_seen = (sig_ok == 4'hF);

// published with `done`; double-sided until the medium says otherwise
always @(posedge clk_sys) begin
	if (reset) begin
		media_ds <= 1'b1;
		is_moof  <= 1'b0;
	end
	else if (state == DONE_PULSE) begin
		is_moof  <= moof_seen;
		media_ds <= moof_seen ? (moof_type == 8'd2) :
		            (!mdb_ok || (vol_blocks > SIDEDNESS_THRESHOLD));
	end
end

always @(posedge clk_sys) begin
	done <= 1'b0; // default; pulsed explicitly below

	if (reset) begin
		state   <= IDLE;
		sd_rd   <= 1'b0;
		wr_req  <= 1'b0;
	end else begin
		case (state)
		IDLE: if (mount_pending) begin
			loaded_size      <= img_size;
			readonly_latched <= img_readonly;
			// whole 512-byte blocks, rounding up (a MOOF ends with a
			// META chunk of any length); the slot holds 2MB
			nsect            <= (img_size >= 64'h200000) ? 13'd4096 :
			                    img_size[21:9] + (img_size[8:0] != 9'd0);
			sector           <= 13'd0;
			state            <= SD_ASSERT;
		end

		SD_ASSERT: begin
			sd_lba <= {19'd0, sector};
			sd_rd  <= 1'b1;
			state  <= SD_WAIT_ACK;
		end

		SD_WAIT_ACK: if (sd_ack) state <= SD_WAIT_DONE;

		SD_WAIT_DONE: begin
			// byte-swap as the ROM download path does; the read side expects it
			if (sd_buff_wr && sd_ack) buf_mem[sd_buff_addr] <= {sd_buff_dout[7:0], sd_buff_dout[15:8]};
			if (!sd_ack) begin
				sd_rd    <= 1'b0;
				word_idx <= 8'd0;
				state    <= DRAIN_FETCH;
			end
		end

		// one cycle for the registered buf_rd to catch up with word_idx
		DRAIN_FETCH: state <= DRAIN_ASSERT;

		DRAIN_ASSERT: begin
			wr_addr <= {sector, 9'd0} + {word_idx, 1'b0};
			wr_data <= buf_rd;
			wr_req  <= 1'b1;
			state   <= DRAIN_WAIT;
		end

		DRAIN_WAIT: if (wr_ack) begin
			wr_req <= 1'b0;
			if (word_idx == 8'd255) begin
				if (sector == nsect - 13'd1) state <= DONE_PULSE;
				else begin
					sector <= sector + 13'd1;
					state  <= SD_ASSERT;
				end
			end else begin
				word_idx <= word_idx + 8'd1;
				state    <= DRAIN_FETCH;
			end
		end

		DONE_PULSE: begin
			done  <= 1'b1;
			// MOOF writes are not supported yet: refuse them at the
			// drive and never write the image back to SD
			if (moof_seen) readonly_latched <= 1'b1;
			state <= IDLE;
		end

		default: state <= IDLE;
		endcase
	end
end

endmodule
