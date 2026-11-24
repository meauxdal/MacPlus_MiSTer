/* PRAM - RTC implementation for plus_too
 *
 * Models the 343-0042 RTC of the Mac Plus: seconds counter plus 256 bytes
 * of extended parameter RAM (XPRAM).
 *
 * Command byte: {R/W, scmd[4:0], 0, 1}, R/W 1 = read.
 *   scmd 0x00-0x07  seconds bytes 0-3 (reads alias 0x04-0x07)
 *   scmd 0x08-0x0B  classic PRAM group 1 -> XPRAM 0x08-0x0B
 *   scmd 0x10-0x1F  classic PRAM group 2 -> XPRAM 0x10-0x1F
 * Extended command, cmd[6:3] == 4'b0111 (MAME: (cmd & 0x78) == 0x38):
 *   byte 1 {R/W, 0111, addr[7:5]}
 *   byte 2 {x, addr[4:0], x, x}
 *   byte 3 data (writes only)
 * The classic 20 bytes are a window onto the same array, as on the real
 * chip, so the old ROMs (128K/512K) see the behavior they always did.
 *
 * The array is one dual-port block RAM. Port A serves the serial engine
 * (writes pulse ser_we, reads land in dout two clocks after ser_addr is
 * set, far below one serial bit period); port B the .NVR image. 256 bytes
 * in fabric would cost ~2K registers plus wide muxes.
 */

module rtc (
	input         clk,
	input         reset,

	input  [32:0] timestamp, // unix timestamp
	input         _cs,
	input         ck,
	input         dat_i,
	output reg    dat_o,

	// host port for the .NVR image (MacPlus.sv): host_dout is ram[host_addr]
	// one clock after the address is presented
	input         host_we,
	input   [7:0] host_addr,
	input   [7:0] host_din,
	output reg [7:0] host_dout,
	output        guest_wr    // pulse: the Mac wrote a PRAM byte
);

reg   [2:0] bit_cnt;
reg         ck_d;
reg   [7:0] din;
reg   [7:0] cmd;
reg   [7:0] dout;
reg   [1:0] stage;     // 0 command, 1 extended address, 2 data
reg         receiving;
reg  [31:0] secs;
reg  [24:0] clocktoseconds;

// Port A: serial engine
(* ramstyle = "no_rw_check" *) reg [7:0] ram[256];
reg   [7:0] ser_addr;
reg         ser_we;
reg   [7:0] ser_wdata;
reg   [7:0] ser_q;
reg   [1:0] rd_pend;   // 2 address set, 1 ser_q valid

always @(posedge clk) begin
	if (ser_we) ram[ser_addr] <= ser_wdata;
	ser_q <= ram[ser_addr];
end

assign guest_wr = ser_we;

// Port B: host. Loads run only while the Mac is held in reset, so the two
// ports never write the same byte at once.
always @(posedge clk) begin
	if (host_we) ram[host_addr] <= host_din;
	host_dout <= ram[host_addr];
end

// Defaults from the original 20-byte table, now at their XPRAM addresses.
// The rest starts zeroed. The Plus ROM finds no 'NuMc' signature at
// 0x0C-0x0F and initializes the extended area itself.
integer i;
initial begin
	for (i = 0; i < 256; i = i + 1)
		ram[i] = 8'h00;
	ram[8'h08] = 8'h03;
	ram[8'h09] = 8'h88;
	ram[8'h0A] = 8'h00;
	ram[8'h0B] = 8'h6C;
	ram[8'h10] = 8'hA8;
	ram[8'h11] = 8'h00;
	ram[8'h12] = 8'h00;
	ram[8'h13] = 8'h22;
	ram[8'h14] = 8'hCC;
	ram[8'h15] = 8'h0A;
	ram[8'h16] = 8'hCC;
	ram[8'h17] = 8'h0A;
	ram[8'h18] = 8'h00;
	ram[8'h19] = 8'h00;
	ram[8'h1A] = 8'h00;
	ram[8'h1B] = 8'h00;
	ram[8'h1C] = 8'h00;
	ram[8'h1D] = 8'h02;
	ram[8'h1E] = 8'h63;
	ram[8'h1F] = 8'h00;
end

initial secs = 0;

// The full byte on the 8th rising edge
wire [7:0] byte_in = {din[6:0], dat_i};

always @(posedge clk) begin
	ser_we <= 0;

	// Read pipeline: two clocks after ser_addr is set, ser_q holds the byte
	if (rd_pend != 0) begin
		rd_pend <= rd_pend - 1'd1;
		if (rd_pend == 1)
			dout <= ser_q;
	end

	if (reset) begin
		bit_cnt <= 0;
		receiving <= 1;
		stage <= 0;
		rd_pend <= 0;
		dat_o <= 1;
	end
	else begin

		// timestamp is only sent at core load
		if (secs==0)
				secs <= timestamp[31:0] + 2082844800; // difference between unix epoch and mac epoch

		// we need to add one to the seconds
		clocktoseconds<= clocktoseconds + 1'd1;
		if (31334399==clocktoseconds) // every 31,334,400 clk (clk_sys) we increment secs by one
		begin
			clocktoseconds<=0;
			secs<=secs+1;
		end

		if (_cs) begin
			bit_cnt <= 0;
			receiving <= 1;
			stage <= 0;
			dat_o <= 1;
		end
		else begin
			ck_d <= ck;

			// transmit at the falling edge
			if (ck_d & ~ck & !receiving)
				dat_o <= dout[7-bit_cnt];
			// receive at the rising edge of ck
			if (~ck_d & ck) begin
				bit_cnt <= bit_cnt + 1'd1;
				if (receiving)
					din <= byte_in;

				if (bit_cnt == 7 && receiving) begin
					case (stage)
					0: begin
						// command byte
						cmd <= byte_in;
						if (byte_in[6:3] == 4'b0111)
							stage <= 1;
						else begin
							stage <= 2;
							receiving <= ~byte_in[7];
							if (byte_in[7]) casez (byte_in[6:0])
								7'b00?0001: dout <= secs[7:0];
								7'b00?0101: dout <= secs[15:8];
								7'b00?1001: dout <= secs[23:16];
								7'b00?1101: dout <= secs[31:24];
								7'b010??01,
								7'b1????01: begin
									ser_addr <= {3'b000, byte_in[6:2]};
									rd_pend <= 2;
								end
								default: ;
							endcase
						end
					end
					1: begin
						// extended address byte
						stage <= 2;
						ser_addr <= {cmd[2:0], byte_in[6:2]};
						if (cmd[7]) begin
							receiving <= 0;
							rd_pend <= 2;
						end
					end
					default: begin
						// data byte of a write
						if (cmd[6:3] == 4'b0111) begin
							ser_wdata <= byte_in;
							ser_we <= 1;
						end
						else casez (cmd[6:0])
							7'b0000001: secs[7:0] <= byte_in;
							7'b0000101: secs[15:8] <= byte_in;
							7'b0001001: secs[23:16] <= byte_in;
							7'b0001101: secs[31:24] <= byte_in;
							7'b010??01,
							7'b1????01: begin
								ser_addr <= {3'b000, cmd[6:2]};
								ser_wdata <= byte_in;
								ser_we <= 1;
							end
							default: ;
						endcase
					end
					endcase
				end
			end
		end
	end
end

endmodule
