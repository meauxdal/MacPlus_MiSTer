// MOOF (Applesauce bitstream image) playback for one 3.5" drive.
//
// The whole .moof file sits in SDRAM at the drive's image offset, in file
// byte order (floppy_loader.v). On a mount, a step or a head change the
// player looks up the track (TMAP at byte 88, one entry per track and side;
// TRK records at byte 256, 8 bytes each), then walks the track's bitstream
// MSB first, one bit cell per INFO "optimal bit timing" (in 125 ns units,
// i.e. cep clocks: 16 = 2 us for GCR). Each one bit becomes a 0.5 us pulse
// on `transition`, the same interface the sector-image serializer drives.
//
// Reads go through the drive's extra-slot SDRAM port: the address is held,
// and the slot (every 16 cep clocks) returns that byte. A read counts only if
// it was issued before the slot's cycle began, see rdPending below.
//
// Blank tracks (TMAP = $FF) give random transitions, as the spec asks.
// FLUX tracks are not played (none of the GCR test images use them); they
// read as blank since their TMAP entries are $FF.
//
// The head keeps its bit position across a track change, wrapped into the
// new track's length. Real drives keep angular position; scaling the index
// by the length ratio would be closer for cross-track synchronised images.
//
// Writes: the IWM reports each bit cell it shifts out (iwm.v writeCell, one
// clk pulse per cell, writeBit its value). While `writing`, each cell
// overwrites the bit under the head in curByte. The IWM cell and the bit
// cell here are both 16 clocks at 8 MHz, so each cell lands on its own bit.
// A modified byte goes back to SDRAM when the head moves past it, or as soon
// as the write ends or the track changes. A byte write is a single-lane
// write on the extra-slot-3 port (wrDs), so no read-modify-write is needed.
//
// For SD persistence the block of the last byte written is kept dirty, and
// handed to floppy_sd_writer (commitDone) when a byte lands in another block
// or the write has ended. The first write after a mount also zeroes the
// header's CRC32 (bytes 8-11, block 0): a zero CRC means "not computed", as
// the image no longer matches the one it had.
//
// Bitstream length never changes. Writes to blank tracks ($FF in TMAP, no
// storage) are dropped.

module floppy_moof
(
	input             clk,
	input             cep,       // 8 MHz enable, same as the floppy logic
	input             rst,       // mount/eject or non-MOOF image

	input             active,    // MOOF image inserted
	input             spinning,  // motor on: the disk turns
	input       [6:0] track,
	input             side,

	output reg [21:0] addr,      // byte offset within the image
	input             rdAck,     // dskReadAckD: a slot cycle is in progress
	input       [7:0] rdData,    // valid at cep while rdAck

	output reg        transition,// 0.5 us pulse per one bit

	// write path
	input             writing,   // drive selected, IWM in write mode, unlocked
	input             wrCell,    // one clk pulse per IWM write cell
	input             wrBit,     // that cell's bit, valid with wrCell

	// extra-slot-3 write port: hold wrReq, wrAddr, wrData and wrDs until
	// the one-clk wrAck
	output reg [21:0] wrAddr,    // byte offset within the image
	output reg [15:0] wrData,    // even byte in the high half
	output reg  [1:0] wrDs,      // {high, low} byte lane enables
	output reg        wrReq,
	input             wrAck,

	// to floppy_sd_writer: one clk pulse per block to write back
	output reg        commitDone,
	output reg [21:0] commitAddr // image byte offset of the block
);

	localparam [3:0] S_IDLE  = 4'd0,
	                 S_OBT   = 4'd1,
	                 S_TMAP  = 4'd2,
	                 S_BLK0  = 4'd3,
	                 S_BLK1  = 4'd4,
	                 S_BC0   = 4'd5,
	                 S_BC1   = 4'd6,
	                 S_BC2   = 4'd7,
	                 S_WRAP  = 4'd8,
	                 S_CUR   = 4'd9,
	                 S_NEXT  = 4'd10,
	                 S_RUN   = 4'd11,
	                 S_BLANK = 4'd12;
	reg [3:0] state;

	// one outstanding SDRAM read
	reg       rdPending; // issued at an earlier cep, waiting for a slot
	reg       rdValid;   // rdByte holds the answer
	reg [7:0] rdByte;

	reg [7:0]  obt;        // cep clocks per bit cell
	reg [7:0]  cellCount;
	reg [7:0]  trkIndex;
	reg [15:0] startBlock;
	reg [19:0] bitCount;
	reg [19:0] bitIdx;     // head position, in bits from the track start
	reg [21:0] trackBase;  // startBlock * 512

	reg [7:0]  curByte;
	reg [7:0]  nextByte;
	reg        nextValid;
	reg        nextFetching;
	reg        needNext;

	reg [6:0]  curTrack;
	reg        curSide;
	reg        relook;     // track or side changed since the last lookup

	reg [2:0]  pulseCount;
	reg [15:0] lfsr;

	wire [16:0] curIdx  = bitIdx[19:3];
	wire [16:0] nextIdx = ({curIdx + 17'd1, 3'b000} >= bitCount) ? 17'd0 :
	                      curIdx + 17'd1;

	// write path state
	reg        cellPend;   // an IWM write cell since the last cep
	reg        cellBit;
	reg        curDirty;   // curByte differs from its SDRAM copy

	// one queued byte write: set by the player (cep), cleared by the writer
	reg        wqValid;
	reg [21:0] wqAddr;
	reg  [7:0] wqByte;

	localparam [1:0] W_IDLE = 2'd0,
	                 W_CRC0 = 2'd1,
	                 W_CRC1 = 2'd2,
	                 W_BYTE = 2'd3;
	reg  [1:0] wstate;
	reg        crcCleared; // header CRC zeroed since the mount
	reg [12:0] dirtyBlk;   // block of the last byte written, not yet committed
	reg        dirtyValid;

	// the pending cell, applied to the bit under the head
	wire       patchNow   = cellPend && writing && state == S_RUN && !relook;
	wire [7:0] bitMask    = 8'h80 >> bitIdx[2:0];
	wire [7:0] curPatched = !patchNow ? curByte :
	                        cellBit   ? (curByte |  bitMask) :
	                                    (curByte & ~bitMask);
	wire       curDirtyNext = curDirty || patchNow;

	// the head leaves curByte at this cep. A dirty byte needs the write
	// queue, so the head stalls (like a late fetch) while it is full.
	wire cellEnd    = state == S_RUN && !relook && spinning &&
	                  cellCount == obt - 8'd1;
	wire atBoundary = (bitIdx + 20'd1 == bitCount) || (bitIdx[2:0] == 3'd7);
	wire nextReady  = nextValid || (nextFetching && rdValid);
	wire switchNow  = cellEnd && atBoundary && nextReady &&
	                  !(curDirtyNext && wqValid);

	// issue a read: hold addr until the slot answers
	task issue(input [21:0] a);
		begin
			addr      <= a;
			rdPending <= 1'b1;
			rdValid   <= 1'b0;
		end
	endtask

	always @(posedge clk) begin
		if (rst || !active) begin
			state        <= S_IDLE;
			rdPending    <= 1'b0;
			rdValid      <= 1'b0;
			addr         <= 22'd0;
			bitIdx       <= 20'd0;
			cellCount    <= 8'd0;
			nextValid    <= 1'b0;
			nextFetching <= 1'b0;
			needNext     <= 1'b0;
			relook       <= 1'b1;
			curTrack     <= 7'd0;
			curSide      <= 1'b0;
			pulseCount   <= 3'd0;
			transition   <= 1'b0;
			if (lfsr == 16'd0) lfsr <= 16'hACE1;
			cellPend     <= 1'b0;
			curDirty     <= 1'b0;
			wqValid      <= 1'b0;
			wstate       <= W_IDLE;
			wrReq        <= 1'b0;
			crcCleared   <= 1'b0;
			dirtyValid   <= 1'b0;
			commitDone   <= 1'b0;
		end else begin
			// Writer, every clk: wrAck and commitDone are one clk wide. A
			// request is raised one clk after the last one was acked, as
			// floppy_write_committer does.
			commitDone <= 1'b0;
			case (wstate)
			W_IDLE: begin
				if (wqValid)
					wstate <= crcCleared ? W_BYTE : W_CRC0;
				else if (dirtyValid && !curDirty && !writing) begin
					// the write is over: persist the last block
					commitDone <= 1'b1;
					commitAddr <= {dirtyBlk, 9'd0};
					dirtyValid <= 1'b0;
				end
			end

			// header CRC32, bytes 8-11, as two whole words of zero
			W_CRC0: if (!wrReq) begin
				wrAddr <= 22'd8;
				wrData <= 16'd0;
				wrDs   <= 2'b11;
				wrReq  <= 1'b1;
			end else if (wrAck) begin
				wrReq  <= 1'b0;
				wstate <= W_CRC1;
			end

			W_CRC1: if (!wrReq) begin
				wrAddr <= 22'd10;
				wrData <= 16'd0;
				wrDs   <= 2'b11;
				wrReq  <= 1'b1;
			end else if (wrAck) begin
				wrReq      <= 1'b0;
				crcCleared <= 1'b1;
				commitDone <= 1'b1;
				commitAddr <= 22'd0;
				wstate     <= W_BYTE;
			end

			W_BYTE: if (!wrReq) begin
				wrAddr <= wqAddr;
				wrData <= {wqByte, wqByte};
				wrDs   <= wqAddr[0] ? 2'b01 : 2'b10;
				wrReq  <= 1'b1;
			end else if (wrAck) begin
				wrReq   <= 1'b0;
				wqValid <= 1'b0;
				// leaving a block: persist it
				if (dirtyValid && wqAddr[21:9] != dirtyBlk) begin
					commitDone <= 1'b1;
					commitAddr <= {dirtyBlk, 9'd0};
				end
				dirtyBlk   <= wqAddr[21:9];
				dirtyValid <= 1'b1;
				wstate     <= W_IDLE;
			end

			default: wstate <= W_IDLE;
			endcase

			if (cep) begin
			// each IWM cell is applied (patchNow) or dropped at the next cep
			cellPend <= 1'b0;

			// a slot answer for a read issued before this slot cycle began
			if (rdPending && rdAck) begin
				rdByte    <= rdData;
				rdValid   <= 1'b1;
				rdPending <= 1'b0;
			end

			// 0.5 us pulse stretcher
			transition <= 1'b0;
			if (pulseCount != 3'd0) begin
				transition <= 1'b1;
				pulseCount <= pulseCount - 3'd1;
			end

			if (track != curTrack || side != curSide) begin
				curTrack <= track;
				curSide  <= side;
				relook   <= 1'b1;
			end

			lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};

			case (state)
			S_IDLE: if (relook && !rdPending) begin
				relook <= 1'b0;
				issue(22'd24);                       // INFO optimal bit timing
				state  <= S_OBT;
			end

			S_OBT: if (rdValid) begin
				obt   <= (rdByte == 8'd0) ? 8'd16 : rdByte;
				issue(22'd88 + {curTrack, curSide}); // TMAP entry
				state <= S_TMAP;
			end

			S_TMAP: if (rdValid) begin
				trkIndex <= rdByte;
				rdValid  <= 1'b0;
				if (rdByte == 8'hFF || rdByte >= 8'd160) begin
					state <= S_BLANK;
				end else begin
					issue(22'd256 + {rdByte, 3'b000});   // TRK.startBlock lo
					state <= S_BLK0;
				end
			end

			S_BLK0: if (rdValid) begin
				startBlock[7:0] <= rdByte;
				issue(22'd257 + {trkIndex, 3'b000});
				state <= S_BLK1;
			end

			S_BLK1: if (rdValid) begin
				startBlock[15:8] <= rdByte;
				issue(22'd260 + {trkIndex, 3'b000});  // TRK.bitCount
				state <= S_BC0;
			end

			S_BC0: if (rdValid) begin
				bitCount[7:0] <= rdByte;
				issue(22'd261 + {trkIndex, 3'b000});
				state <= S_BC1;
			end

			S_BC1: if (rdValid) begin
				bitCount[15:8] <= rdByte;
				issue(22'd262 + {trkIndex, 3'b000});
				state <= S_BC2;
			end

			S_BC2: if (rdValid) begin
				// bit counts fit 20 bits; a GCR track is well under 2^17
				bitCount[19:16] <= rdByte[3:0];
				trackBase       <= {startBlock[12:0], 9'd0};
				rdValid         <= 1'b0;
				state           <= S_WRAP;
			end

			S_WRAP: begin
				if (bitCount == 20'd0 || startBlock < 16'd3)
					state <= S_BLANK;
				else begin
					// keep the head position, wrapped into this track
					if (bitIdx >= bitCount)
						bitIdx <= (bitIdx - bitCount >= bitCount) ? 20'd0 :
						          bitIdx - bitCount;
					state <= S_CUR;
				end
			end

			S_CUR: if (!rdPending && !rdValid) begin
				issue(trackBase + curIdx);
			end else if (rdValid) begin
				curByte      <= rdByte;
				issue(trackBase + nextIdx);
				nextValid    <= 1'b0;
				nextFetching <= 1'b1;
				cellCount    <= 8'd0;
				state        <= S_RUN;
			end

			S_RUN: begin
				// the write cell under the head; a byte switch below
				// overrides both with the next byte
				curByte  <= curPatched;
				curDirty <= curDirtyNext;

				if (nextFetching && rdValid) begin
					nextByte     <= rdByte;
					nextValid    <= 1'b1;
					nextFetching <= 1'b0;
					rdValid      <= 1'b0;
				end

				// one cep after a byte switch: fetch the byte after the new
				// current one (nextIdx now follows the updated bitIdx)
				if (needNext && !rdPending) begin
					needNext     <= 1'b0;
					issue(trackBase + nextIdx);
					nextFetching <= 1'b1;
				end

				if (relook) begin
					// stay in place until the outstanding fetch lands and a
					// dirty byte is queued (by the flush below)
					if (!rdPending && !curDirty) begin
						rdValid      <= 1'b0;
						nextFetching <= 1'b0;
						needNext     <= 1'b0;
						state        <= S_IDLE;
					end
				end else if (spinning) begin
					if (cellCount == 8'd0) begin
						if (curByte[3'd7 - bitIdx[2:0]]) begin
							transition <= 1'b1;
							pulseCount <= 3'd3;
						end
					end

					if (cellCount == obt - 8'd1) begin
						// last bit of the byte, or of the track (which wraps to
						// byte 0): the next cell needs the next byte; stall
						// in place if it is late, or if a dirty byte finds
						// the write queue full
						if (atBoundary) begin
							if (switchNow) begin
								if (curDirtyNext) begin
									wqValid <= 1'b1;
									wqAddr  <= trackBase + curIdx;
									wqByte  <= curPatched;
								end
								curDirty     <= 1'b0;
								curByte      <= nextValid ? nextByte : rdByte;
								bitIdx       <= (bitIdx + 20'd1 == bitCount) ? 20'd0 :
								                bitIdx + 20'd1;
								cellCount    <= 8'd0;
								nextValid    <= 1'b0;
								nextFetching <= 1'b0;
								rdValid      <= 1'b0;
								needNext     <= 1'b1;
							end
						end else begin
							bitIdx    <= bitIdx + 20'd1;
							cellCount <= 8'd0;
						end
					end else begin
						cellCount <= cellCount + 8'd1;
					end
				end
			end

			S_BLANK: begin
				if (relook) begin
					state <= S_IDLE;
				end else if (spinning) begin
					// random flux: roughly one cell in four
					if (cellCount == obt - 8'd1) begin
						cellCount <= 8'd0;
						if (lfsr[0] & lfsr[5]) begin
							transition <= 1'b1;
							pulseCount <= 3'd3;
						end
					end else begin
						cellCount <= cellCount + 8'd1;
					end
				end
			end

			default: state <= S_IDLE;
			endcase

			// the write ended, or the head is about to leave the track:
			// queue the dirty byte now rather than when the head moves on.
			// Not on a byte switch, which queues it itself.
			if (curDirty && !wqValid && (!writing || relook) && !switchNow) begin
				wqValid  <= 1'b1;
				wqAddr   <= trackBase + curIdx;
				wqByte   <= curByte;
				curDirty <= 1'b0;
			end
			end

			// after the cep step, so a cell arriving on a cep clk is kept
			if (wrCell) begin
				cellPend <= 1'b1;
				cellBit  <= wrBit;
			end
		end
	end

endmodule
