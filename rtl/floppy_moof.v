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
// Writes: not handled here yet. A MOOF write path would use bitIdx and the
// fetched byte addresses below to patch bits in SDRAM as the IWM shifts
// them out (iwm.v writeFluxStrobe).

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

	output reg        transition // 0.5 us pulse per one bit
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
		end else if (cep) begin
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
					// stay in place until the outstanding fetch lands
					if (!rdPending) begin
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
						// in place if it is late
						if (bitIdx + 20'd1 == bitCount || bitIdx[2:0] == 3'd7) begin
							if (nextValid || (nextFetching && rdValid)) begin
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
		end
	end

endmodule
