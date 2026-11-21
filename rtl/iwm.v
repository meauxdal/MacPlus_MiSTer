/* IWM 

   Mapped to $DFE1FF - $DFFFFF
	
	The 16 IWM one-bit registers are {8'hDF, 8'b111xxxx1, 8'hFF}:
		0	$0		ca0L		CA0 off (0)
		1	$200	ca0H		CA0 on (1)
		2	$400	ca1L		CA1 off (0)
		3	$600	ca1H		CA1 on (1)
		4	$800	ca2L		CA2 off (0)
		5	$A00	ca2H		CA2 on (1)
		6	$C00	ph3L		LSTRB off (low)
		7	$E00	ph3H		LSTRB on (high)
		8	$1000	mtrOff	ENABLE disk enable off
		9	$1200	mtrOn		ENABLE disk enable on
		10	$1400	intDrive	SELECT select internal drive
		11	$1600	extDrive	SELECT select external drive
		12	$1800	q6L		Q6 off
		13	$1A00	q6H		Q6 on
		14	$1C00	q7L		Q7 off, read register
		15	$1E00	q7H		Q7 on, write register
	
	Notes from IWM manual:
	Serial data is shifted in/out MSB first, with a bit transferred every 2 microseconds.
	When writing data, a 1 is written as a transition on writeData at a bit cell boundary time, and a 0 is written as no transition.
	When reading data, a falling transition within a bit cell window is considered to be a 1, and no falling transition is considered a 0.
	When reading data, the read data register will latch the shift register when a 1 is shifted into the MSB.
	The read data register will be cleared 14 fclk periods (about 2 microseconds) after a valid data read takes place-- a valid data read 
	   being defined as both /DEV being low and D7 (the MSB) outputting a one from the read data register for at least one fclk period.
*/		

module iwm
(
	input clk,
	input cep,
	input cen,
	// clk16_en_n and turbo scale only the read-latch clear interval; the 16 us
	// byte rate stays at 8 MHz
	input cen16,
	input turbo,

	input _reset,
	input selectIWM,
	input _cpuRW,
	input _cpuLDS,	
	input [15:0] dataIn,
	input [3:0] cpuAddrRegHi,
	input SEL, // from VIA
	input driveSel, // internal drive select, 0 - upper, 1 - lower
	output [15:0] dataOut,
	input [1:0] insertDisk,
	output [1:0] diskEject,
	input [1:0] img800k,    // mounted file is 819,200 bytes: see floppy.v's port comment
	input drive800k,        // drive mechanism: see floppy.v's port comment
	input [1:0] mediaSides, // what the medium said at mount: see floppy.v
	input [8:0] disk_pwm,   // spindle duty index 0..399: see floppy.v's tachometer
	
	output [1:0] diskMotor,
	output [1:0] diskAct,
	
	// interface to fetch data for internal drive
	output [21:0] dskReadAddrInt,
	input dskReadAckInt,
	output [21:0] dskReadAddrExt,
	input dskReadAckExt,
	input [7:0] dskReadData,

	// floppy write path
	input [1:0] writeProtect, // {ext,int} - OSD toggle ANDed with img_readonly, per drive

	output [21:0] dskWriteAddrInt,
	output [15:0] dskWriteDataInt,
	output        dskWriteReqInt,
	input         dskWriteAckInt,
	output [21:0] dskWriteAddrExt,
	output [15:0] dskWriteDataExt,
	output        dskWriteReqExt,
	input         dskWriteAckExt,

	// DCD (Apple HD20) block-device slot, passed through to rtl/dcd.v
	output [31:0] dcd_sd_lba,
	output        dcd_sd_rd,
	output        dcd_sd_wr,
	input         dcd_sd_ack,
	input   [7:0] dcd_sd_buff_addr,
	input  [15:0] dcd_sd_buff_dout,
	output [15:0] dcd_sd_buff_din,
	input         dcd_sd_buff_wr,
	input         dcd_img_mounted,
	input  [63:0] dcd_img_size,
	input         dcd_img_readonly,

	// commit notice, per drive - see floppy.v's dskCommit* ports
	output        dskCommitDoneInt,
	output [21:0] dskCommitAddrInt,
	output        dskCommitDoneExt,
	output [21:0] dskCommitAddrExt
);

	wire [7:0] dataInLo = dataIn[7:0];
	reg [7:0] dataOutLo;
	assign dataOut = { 8'hBE, dataOutLo };
	
	// IWM state
	reg ca0, ca1, ca2, lstrb, selectExternalDrive, q6, q7;
	reg ca0Next, ca1Next, ca2Next, lstrbNext, selectExternalDriveNext, q6Next, q7Next;
	wire advanceDriveHead; // prevents overrun when debugging, does not exit on a real Mac!
	reg [7:0] readDataLatch;
	// write handshake register bits 7 (buffer empty) and 6 (no underrun), owned
	// by the serial write machine below
	reg writeBufferEmpty, writeNoUnderrun;

	// floppy disk drives 
	reg diskEnableExt, diskEnableInt;
	reg diskEnableExtNext, diskEnableIntNext;
	wire newByteReadyInt;
	wire [7:0] readDataInt;
	wire readTransitionInt;
	wire senseInt = readDataInt[7]; // bit 7 doubles as the sense line here
	wire newByteReadyExt;
	wire [7:0] readDataExt;
	wire readTransitionExt;
	// a DCD image is mounted (rtl/dcd.v)
	wire dcdPresent;

	// daisy chain (Apple's flow-through flip-flop): LSTRB with /ENBL2 asserted
	// advances past the DCD to the external floppy; releasing /ENBL2 rewinds
	reg  chainSel;          // 1 = the chain has advanced past the DCD
	reg  lstrbPrevChain;
	always @(posedge clk) if (cep) lstrbPrevChain <= lstrb;
	wire chainAdvance = cep && lstrbPrevChain && ~lstrb;

	always @(posedge clk or negedge _reset) begin
		if (_reset == 1'b0)
			chainSel <= 1'b0;
		else if (~diskEnableExt)              // /ENBL2 released - rewind
			chainSel <= 1'b0;
		else if (chainAdvance & dcdPresent)   // hand the port to the floppy
			chainSel <= 1'b1;
	end

	// the DCD still holds the external port
	wire dcdOwnsPort = dcdPresent & ~chainSel;

	// a CPU write to the data register: Q7=Q6=1 with a drive enabled
	wire dataRegWrite = (_cpuRW == 1'b0) && selectIWM && (_cpuLDS == 1'b0) &&
	                    ({q7Next, q6Next} == 2'b11) && (diskEnableExt | diskEnableInt);

	// dataRegWrite is a level held for three cen samples; take its first one
	reg dataRegWriteSeen;
	always @(posedge clk or negedge _reset) begin
		if (_reset == 1'b0)
			dataRegWriteSeen <= 1'b0;
		else if (cen)
			dataRegWriteSeen <= dataRegWrite;
	end
	wire dataRegWriteStrobe = cen && dataRegWrite && !dataRegWriteSeen;

	// The DCD still takes bytes as the CPU writes them, not as the IWM shifts
	// them out; see the write machine below.
	wire writeReqDcd = dataRegWriteStrobe && selectExternalDriveNext;

	// Floppies take each byte as the write machine moves it from the data
	// register into the shifter (see writeLoadStrobe below).
	wire       writeLoadStrobe;
	wire [7:0] writeLoadByte;
	wire writeReqInt = writeLoadStrobe && !selectExternalDrive;
	wire writeReqExt = writeLoadStrobe &&  selectExternalDrive;

	floppy floppyInt
	(
		.clk(clk),
		.cep(cep),
		.cen(cen),

		._reset(_reset),
		.ca0(ca0),
		.ca1(ca1),
		.ca2(ca2),
		.SEL(SEL),
		.lstrb(lstrb),
		// a real IWM enables one drive at a time; keeps the chain walk's LSTRB
		// (EJECT in state 7) from the internal drive
		._enable(~(diskEnableInt & driveSel & ~selectExternalDrive)),
		.writeData(writeLoadByte),
		.readData(readDataInt),
		.readTransition(readTransitionInt),
		.advanceDriveHead(advanceDriveHead),
		.newByteReady(newByteReadyInt),
		.insertDisk(insertDisk[0]),
		.img800k(img800k[0]),
		.drive800k(drive800k),
		.mediaSides(mediaSides[0]),
		.disk_pwm(disk_pwm),
		.diskEject(diskEject[0]),

		.motor(diskMotor[0]),
		.act(diskAct[0]),

		.dskReadAddr(dskReadAddrInt),
		.dskReadAck(dskReadAckInt),
		.dskReadData(dskReadData),

		.writeReq(writeReqInt),
		.writeProtect(writeProtect[0]),
		.writeMode(q7),
		.dskWriteAddr(dskWriteAddrInt),
		.dskWriteData(dskWriteDataInt),
		.dskWriteReq(dskWriteReqInt),
		.dskWriteAck(dskWriteAckInt),

		.dskCommitDone(dskCommitDoneInt),
		.dskCommitAddr(dskCommitAddrInt)
	);

	floppy floppyExt
	(
		.clk(clk),
		.cep(cep),
		.cen(cen),

		._reset(_reset),
		.ca0(ca0),
		.ca1(ca1),
		.ca2(ca2),
		.SEL(SEL),
		.lstrb(lstrb),
		// disabled while the DCD owns the port
		._enable(~(diskEnableExt & ~dcdOwnsPort)),
		.writeData(writeLoadByte),
		.readData(readDataExt),
		.readTransition(readTransitionExt),
		.advanceDriveHead(advanceDriveHead),
		.newByteReady(newByteReadyExt),
		.insertDisk(insertDisk[1]),
		.img800k(img800k[1]),
		.drive800k(drive800k),
		.mediaSides(mediaSides[1]),
		.disk_pwm(disk_pwm),
		.diskEject(diskEject[1]),

		.motor(diskMotor[1]),
		.act(diskAct[1]),

		.dskReadAddr(dskReadAddrExt),
		.dskReadAck(dskReadAckExt),
		.dskReadData(dskReadData),

		.writeReq(writeReqExt),
		.writeProtect(writeProtect[1]),
		.writeMode(q7),
		.dskWriteAddr(dskWriteAddrExt),
		.dskWriteData(dskWriteDataExt),
		.dskWriteReq(dskWriteReqExt),
		.dskWriteAck(dskWriteAckExt),

		.dskCommitDone(dskCommitDoneExt),
		.dskCommitAddr(dskCommitAddrExt)
	);

	// DCD (Apple HD20) at the head of the chain; it owns the port until the
	// Mac advances the chain
	wire  [7:0] readDataDcd;
	wire        newByteReadyDcd;

	dcd dcd0
	(
		.clk(clk),
		.cep(cep),
		.cen(cen),
		.turbo(turbo),
		._reset(_reset),
		.ca0(ca0),
		.ca1(ca1),
		.ca2(ca2),
		._enable(~(diskEnableExt & ~chainSel)),
		.writeData(dataIn[7:0]),
		.writeReq(writeReqDcd), // one-shot: see dataRegWriteSeen above
		.readData(readDataDcd),
		.newByteReady(newByteReadyDcd),
		.sd_lba(dcd_sd_lba),
		.sd_rd(dcd_sd_rd),
		.sd_wr(dcd_sd_wr),
		.sd_ack(dcd_sd_ack),
		.sd_buff_addr(dcd_sd_buff_addr),
		.sd_buff_dout(dcd_sd_buff_dout),
		.sd_buff_din(dcd_sd_buff_din),
		.sd_buff_wr(dcd_sd_buff_wr),
		.img_mounted(dcd_img_mounted),
		.img_size(dcd_img_size),
		.img_readonly(dcd_img_readonly),
		.present(dcdPresent)
	);

	wire [7:0] readDataExtSel      = dcdOwnsPort ? readDataDcd     : readDataExt;
	wire       newByteReadyExtSel  = dcdOwnsPort ? newByteReadyDcd : newByteReadyExt;

	// sense comes through the same mux, so the ROM's ID probe sees the DCD
	wire senseExt = readDataExtSel[7];

	wire [7:0] readData = selectExternalDrive ? readDataExtSel : readDataInt;
	wire newByteReady = selectExternalDrive ? newByteReadyExtSel : newByteReadyInt;
	wire readTransition = selectExternalDrive ? readTransitionExt : readTransitionInt;
	wire dcdByteRead = selectExternalDrive && dcdOwnsPort;
	wire floppyReadActive = !q7 && (selectExternalDrive ? (diskEnableExt && !dcdOwnsPort) :
	                                                      (diskEnableInt && driveSel));
	
	// IWM mode register, written while the controller is disabled.
	reg [4:0] iwmMode;
	/* IWM mode register: S C M H L
 	 S	Clock speed:
			0 = 7 MHz
			1 = 8 MHz
		Should always be 1 for Macintosh.
	 C	Bit cell time:
			0 = 4 usec/bit (for 5.25 drives)
			1 = 2 usec/bit (for 3.5 drives) (Macintosh mode)
	 M	Motor-off timer:
			0 = leave drive on for 1 sec after program turns
			    it off
			1 = no delay (Macintosh mode)
		Should be 0 for 5.25 and 1 for 3.5.
	 H	Handshake protocol:
			0 = synchronous (software must supply proper
			    timing for writing data)
			1 = asynchronous (IWM supplies timing) (Macintosh Mode)
		Should be 0 for 5.25 and 1 for 3.5.
	 L	Latch mode:
			0 = read-data stays valid for about 7 usec
			1 = read-data stays valid for full byte time (Macintosh mode)
		Should be 0 for 5.25 and 1 for 3.5.
	*/

	// any read/write access to IWM bit registers will change their values
	always @(*) begin
		ca0Next <= ca0;
		ca1Next <= ca1;
		ca2Next <= ca2;
		lstrbNext <= lstrb;
		diskEnableExtNext <= diskEnableExt;
		diskEnableIntNext <= diskEnableInt;
		selectExternalDriveNext <= selectExternalDrive;
		q6Next <= q6;
		q7Next <= q7;

		if (selectIWM == 1'b1 && _cpuLDS == 1'b0) begin
			case (cpuAddrRegHi[3:1])
				3'h0: // ca0
					ca0Next <= cpuAddrRegHi[0];
				3'h1: // ca1
					ca1Next <= cpuAddrRegHi[0];
				3'h2: // ca2
					ca2Next <= cpuAddrRegHi[0];
				3'h3: // lstrb
					lstrbNext <= cpuAddrRegHi[0];
				3'h4: // disk enable
					if (selectExternalDrive)
						diskEnableExtNext <= cpuAddrRegHi[0];
					else
						diskEnableIntNext <= cpuAddrRegHi[0];
				3'h5: // external drive
					selectExternalDriveNext <= cpuAddrRegHi[0];
				3'h6: // Q6 
					q6Next <= cpuAddrRegHi[0];
				3'h7: // Q7 
					q7Next <= cpuAddrRegHi[0];
			endcase
		end
	end

	// update IWM bit registers
	always @(posedge clk or negedge _reset) begin
		if (_reset == 1'b0) begin
			ca0 <= 0;
			ca1 <= 0;
			ca2 <= 0;
			lstrb <= 0;
			diskEnableExt <= 0;
			diskEnableInt <= 0;
			selectExternalDrive <= 0;
			q6 <= 0;
			q7 <= 0;
		end
		else begin
			ca0 <= ca0Next;
			ca1 <= ca1Next;
			ca2 <= ca2Next;
			lstrb <= lstrbNext;
			diskEnableExt <= diskEnableExtNext;
			diskEnableInt <= diskEnableIntNext;
			selectExternalDrive <= selectExternalDriveNext;
			q6 <= q6Next;
			q7 <= q7Next;
		end
	end
	
	// read IWM state
	always @(*) begin
		dataOutLo = 8'hEF;
		
		// reading any IWM address returns state as selected by Q7 and Q6
		// the access's own switch takes effect before the read, as in MAME's
		// control() then read()
		case ({q7Next,q6Next})
			2'b00: // data-in register (from disk drive) - MSB is 1 when data is valid
				// all ones while no drive is enabled
				dataOutLo <= (diskEnableExtNext | diskEnableIntNext) ? readDataLatch : 8'hFF;
			2'b01: // IWM status register - read only
				dataOutLo <= { (selectExternalDriveNext ? senseExt : senseInt), 1'b0, diskEnableExtNext | diskEnableIntNext, iwmMode };
			2'b10: // handshake - read only; bits 5-0 read as ones
				dataOutLo <= { writeBufferEmpty, writeNoUnderrun, 6'b111111 };
			2'b11: // IWM mode register when not enabled (write-only), or (write?) data register when enabled
				dataOutLo <= 0;
		endcase
	end

	// write IWM state
	// the write-data register is handled by writeReqInt/Ext and dataInLo above
	always @(posedge clk or negedge _reset) begin
		if (_reset == 1'b0) begin
			iwmMode <= 0;
		end
		else if(cen) begin
			if (_cpuRW == 0 && selectIWM == 1'b1 && _cpuLDS == 1'b0) begin
				// writing to any IWM address modifies state as selected by Q7 and Q6
				case ({q7Next,q6Next})
					2'b11: begin
						if (~(diskEnableExt | diskEnableInt))
							iwmMode <= dataInLo[4:0];
					end
				endcase
			end
		end
	end

	// The IWM reads a serial pulse stream using a rolling transition window.
	// EDGE0 accepts flux within a window and emits zero when the window expires;
	// EDGE1 emits one after the half-window following an accepted transition.
	wire iwmRead = (_cpuRW == 1'b1 && selectIWM == 1'b1 && _cpuLDS == 1'b0);
	reg [3:0] readLatchClearTimer;
	reg [7:0] readShiftRegister;
	reg [5:0] readWindowTimer;
	reg [1:0] readWindowState;
	reg       floppyReadActiveD;
	reg       syncReadUpdatePending;
	reg [3:0] syncReadUpdateTimer;
	reg [5:0] readWindowCycles;
	reg [5:0] readHalfWindowCycles;
	reg [3:0] readRegisterUpdateDelay;
	localparam [1:0] READ_WINDOW_IDLE = 2'd0;
	localparam [1:0] READ_WINDOW_EDGE0 = 2'd1;
	localparam [1:0] READ_WINDOW_EDGE1 = 2'd2;

	always @(*) begin
		case (iwmMode[4:3])
			2'b00: begin readWindowCycles = 6'd28; readHalfWindowCycles = 6'd14; end
			2'b01: begin readWindowCycles = 6'd14; readHalfWindowCycles = 6'd7;  end
			2'b10: begin readWindowCycles = 6'd36; readHalfWindowCycles = 6'd16; end
			default: begin readWindowCycles = 6'd16; readHalfWindowCycles = 6'd8; end
		endcase
		readRegisterUpdateDelay = iwmMode[3] ? 4'd4 : 4'd8;
	end

	wire readShiftOne = readWindowState == READ_WINDOW_EDGE1 &&
	                     readWindowTimer >= readHalfWindowCycles - 6'd1;
	// floppy.v's transition pulse is 0.5 us wide; the window takes its leading
	// edge, so one pulse is one transition however the windows fall
	reg  readTransitionD;
	always @(posedge clk or negedge _reset)
		if (_reset == 1'b0) readTransitionD <= 1'b0;
		else if (cen)       readTransitionD <= readTransition;
	wire readTransitionEdge = readTransition && !readTransitionD;

	wire readShiftZero = readWindowState == READ_WINDOW_EDGE0 && !readTransitionEdge &&
	                     readWindowTimer >= readWindowCycles - 6'd1;
	wire readShiftStrobe = cen && floppyReadActive && (readShiftOne || readShiftZero);
	wire readCellBit = readWindowState == READ_WINDOW_EDGE1;
	wire [7:0] readShiftNext = {readShiftRegister[6:0], readCellBit};

	// The latch-clear countdown scales with CPU speed: the .Sony driver polls
	// bit 7 about 2.0 us apart at 8 MHz and 1.0 us at 16 MHz, and a latch that
	// has not cleared reads as a fresh byte. Ticks on cen16 under turbo, cen
	// otherwise; the clear runs on the same enable as the countdown.
	wire latchClearCe = turbo ? cen16 : cen;

	always @(posedge clk or negedge _reset) begin
		if (_reset == 1'b0) begin
			readDataLatch <= 0;
			readLatchClearTimer <= 0;
			readShiftRegister <= 8'd0;
			readWindowTimer <= 6'd0;
			readWindowState <= READ_WINDOW_IDLE;
			floppyReadActiveD <= 1'b0;
			syncReadUpdatePending <= 1'b0;
			syncReadUpdateTimer <= 4'd0;
		end
		else begin
			// a countdown timer governs how long after a data latch read before the latch is cleared
			if (latchClearCe) begin
				if (readLatchClearTimer != 0) begin
					readLatchClearTimer <= readLatchClearTimer - 1'b1;
				end
			end

			// reload after a valid CPU read, on latchClearCe rather than cen so the
			// countdown start is not snapped to the 2-cycle grid at 16 MHz
			if (latchClearCe) begin
				if (iwmMode[1] && iwmRead && !q7Next && !q6Next && readDataLatch[7]) begin
					readLatchClearTimer <= 4'hE; // clear latch 14 clocks after the conclusion of a valid read
				end
			end

			if (cen) begin
				floppyReadActiveD <= floppyReadActive;
				if (!floppyReadActive) begin
					readWindowTimer <= 6'd0;
					readWindowState <= READ_WINDOW_IDLE;
					syncReadUpdatePending <= 1'b0;
					syncReadUpdateTimer <= 4'd0;
				end else if (!floppyReadActiveD) begin
					// Start in S_IDLE, then wait one full window before looking
					// for transitions in the following window.
					readWindowTimer <= 6'd0;
					readWindowState <= READ_WINDOW_IDLE;
					readShiftRegister <= 8'd0;
					readDataLatch <= 8'd0;
					syncReadUpdatePending <= 1'b0;
					syncReadUpdateTimer <= 4'd0;
				end else begin
					case (readWindowState)
						READ_WINDOW_IDLE: begin
							if (readWindowTimer >= readWindowCycles - 6'd1) begin
								readWindowTimer <= 6'd0;
								readWindowState <= READ_WINDOW_EDGE0;
							end else
								readWindowTimer <= readWindowTimer + 6'd1;
						end
						READ_WINDOW_EDGE0: begin
							if (readTransitionEdge) begin
								readWindowTimer <= 6'd0;
								readWindowState <= READ_WINDOW_EDGE1;
							end else if (readShiftZero) begin
								readWindowTimer <= 6'd0;
							end else
								readWindowTimer <= readWindowTimer + 6'd1;
						end
						READ_WINDOW_EDGE1: begin
							if (readShiftOne) begin
								readWindowTimer <= 6'd0;
								readWindowState <= READ_WINDOW_EDGE0;
							end else
								readWindowTimer <= readWindowTimer + 6'd1;
						end
						default: begin
							readWindowTimer <= 6'd0;
							readWindowState <= READ_WINDOW_IDLE;
						end
					endcase

					if (readShiftStrobe) begin
						readShiftRegister <= readShiftNext;
						if (readShiftNext[7]) begin
							readDataLatch <= readShiftNext;
							readShiftRegister <= 8'd0;
							syncReadUpdatePending <= 1'b0;
							// a fresh byte cancels a pending clear of the last one
							// (MAME zeroes m_async_update here)
							readLatchClearTimer <= 4'd0;
						end else if (!iwmMode[1] && readShiftNext >= 8'd4) begin
							// Synchronous mode exposes partial data once the shifter
							// reaches the IWM's 0x04 threshold.
							readDataLatch <= readShiftNext;
							syncReadUpdatePending <= 1'b0;
						end else if (!iwmMode[1] && readShiftNext >= 8'd2) begin
							syncReadUpdatePending <= 1'b1;
							syncReadUpdateTimer <= readRegisterUpdateDelay;
						end
					end

					if (syncReadUpdatePending &&
					    !(readShiftStrobe && readShiftNext >= 8'd2)) begin
						if (syncReadUpdateTimer <= 4'd1) begin
							readDataLatch <= readShiftStrobe ? readShiftNext : readShiftRegister;
							syncReadUpdatePending <= 1'b0;
							syncReadUpdateTimer <= 4'd0;
						end else
							syncReadUpdateTimer <= syncReadUpdateTimer - 4'd1;
					end
				end
			end

			// Reading status with Q6 high clears the read shifter.
			if (iwmRead && !q7Next && q6Next) begin
				readShiftRegister <= 8'd0;
				syncReadUpdatePending <= 1'b0;
				syncReadUpdateTimer <= 4'd0;
			end

			// The HD20 block device still presents byte-ready data; retain that
			// adapter while floppy drives use the serial transition receiver.
			if (cen && dcdByteRead && newByteReady)
				readDataLatch <= readData;
			else if (iwmMode[1] && latchClearCe && readLatchClearTimer == 4'd1 &&
			         !(readShiftStrobe && readShiftNext[7]) &&
			         !(cen && dcdByteRead && newByteReady)) begin
				readDataLatch <= 0;
			end
		end
	end

	// Serial write machine, after MAME's iwm_device MODE_WRITE states. In
	// asynchronous mode the CPU fills a one-byte data register; 7 clocks before
	// the first bit cell of each byte the IWM moves it into the shifter and
	// raises handshake bit 7 (buffer empty) so the CPU can supply the next byte
	// while this one shifts out. Finding the register still empty at that point
	// is an underrun: handshake bit 6 drops and the write stops until write
	// mode is left and entered again. Synchronous mode has no buffer: CPU
	// writes go straight to the shifter, which runs on without byte framing.
	//
	// Bit cells are two half-windows long (16 clocks in the Mac's mode $1F, 2
	// us at 8 MHz). A one is a flux transition at mid-cell (writeFluxStrobe).
	// The sector-image floppies take whole bytes at the load instead.
	localparam [2:0] WRITE_IDLE     = 3'd0;
	localparam [2:0] WRITE_LOAD     = 3'd1;
	localparam [2:0] WRITE_MIDDLE   = 3'd2;
	localparam [2:0] WRITE_END      = 3'd3;
	localparam [2:0] WRITE_UNDERRUN = 3'd4;

	wire iwmActive = diskEnableExt | diskEnableInt;
	wire writeMode = iwmActive && q7;

	reg [2:0] writeState;
	reg [4:0] writeTimer;    // clocks until the next state action
	reg [3:0] writeBitCount;
	reg [7:0] writeDataReg;
	reg [7:0] writeShift;
	reg       writeModeD;
	reg       writeLoadStrobeReg;
	reg [7:0] writeLoadByteReg;
	reg       writeFluxStrobe; // for a flux-level drive; unused by DSK images

	// one clk wide at the cen after the load: floppy.v samples it every clk
	assign writeLoadStrobe = cen && writeLoadStrobeReg;
	assign writeLoadByte   = writeLoadByteReg;

	// Half-window less one, for the down-counter. The 7 MHz fast mode's 7-clock
	// half-window leaves no time between the load and the first mid-cell; that
	// mode is only meaningful with a 7 MHz fclk and is clamped to one clock.
	wire [4:0] writeHalfM1  = readHalfWindowCycles[4:0] - 5'd1;
	wire [4:0] writeAfterLoadM1 = (readHalfWindowCycles[4:0] > 5'd8) ?
	                              readHalfWindowCycles[4:0] - 5'd8 : 5'd0;

	always @(posedge clk or negedge _reset) begin
		if (_reset == 1'b0) begin
			writeState         <= WRITE_IDLE;
			writeTimer         <= 5'd0;
			writeBitCount      <= 4'd0;
			writeDataReg       <= 8'd0;
			writeShift         <= 8'd0;
			writeModeD         <= 1'b0;
			writeLoadStrobeReg <= 1'b0;
			writeLoadByteReg   <= 8'd0;
			writeFluxStrobe    <= 1'b0;
			writeBufferEmpty   <= 1'b1; // MAME resets the handshake to $BF
			writeNoUnderrun    <= 1'b0;
		end else if (cen) begin
			writeModeD         <= writeMode;
			writeLoadStrobeReg <= 1'b0;
			writeFluxStrobe    <= 1'b0;

			if (!iwmActive)
				writeNoUnderrun <= 1'b0; // motor off clears bit 6; Q7 low alone does not

			if (!writeMode) begin
				writeState <= WRITE_IDLE;
				writeTimer <= 5'd0;
			end else if (!writeModeD) begin
				// entering write mode, which also takes MAME's S_IDLE step at once;
				// the buffer-empty flag is left as it was
				writeNoUnderrun <= 1'b1;
				if (iwmMode[1]) begin
					writeState    <= WRITE_LOAD;
					writeBitCount <= 4'd8;
					writeTimer    <= 5'd6; // load 7 clocks after entry
				end else begin
					writeShift <= writeDataReg;
					writeState <= WRITE_MIDDLE;
					writeTimer <= writeHalfM1;
				end
			end else if (writeTimer != 5'd0) begin
				writeTimer <= writeTimer - 5'd1;
			end else begin
				case (writeState)
					WRITE_IDLE: begin
						if (iwmMode[1]) begin
							writeState    <= WRITE_LOAD;
							writeBitCount <= 4'd8;
							writeTimer    <= 5'd6;
						end else begin
							writeShift <= writeDataReg;
							writeState <= WRITE_MIDDLE;
							writeTimer <= writeHalfM1;
						end
					end
					WRITE_LOAD: begin
						if (writeBufferEmpty) begin
							writeNoUnderrun <= 1'b0;
							writeState      <= WRITE_UNDERRUN;
						end else begin
							writeShift         <= writeDataReg;
							writeLoadByteReg   <= writeDataReg;
							writeBufferEmpty   <= 1'b1;
							writeLoadStrobeReg <= 1'b1;
							writeState         <= WRITE_MIDDLE;
							writeTimer         <= writeAfterLoadM1;
						end
					end
					WRITE_MIDDLE: begin
						writeFluxStrobe <= writeShift[7];
						writeShift      <= {writeShift[6:0], 1'b0};
						writeState      <= WRITE_END;
						writeTimer      <= writeHalfM1;
					end
					WRITE_END: begin
						if (iwmMode[1] && writeBitCount == 4'd1) begin
							writeState    <= WRITE_LOAD;
							writeBitCount <= 4'd8;
							writeTimer    <= 5'd6;
						end else begin
							writeBitCount <= writeBitCount - 4'd1;
							writeState    <= WRITE_MIDDLE;
							writeTimer    <= writeHalfM1;
						end
					end
					default: ; // WRITE_UNDERRUN holds until write mode is left
				endcase
			end

			// The data register write lands after the state step, as MAME's
			// data_w follows control()'s sync(). MAME clears bit 7 only in latch
			// mode, which would underrun every unlatched async write; it is
			// cleared on every write here.
			if (dataRegWriteStrobe) begin
				writeDataReg     <= dataInLo;
				writeBufferEmpty <= 1'b0;
				if (!iwmMode[1] && writeMode)
					writeShift <= dataInLo;
			end
		end
	end

	// inert: floppy.v hardwires readyToAdvanceHead to 1
	assign advanceDriveHead = readLatchClearTimer == 1'b1; // prevents overrun when debugging, does not exist on a real Mac!
endmodule
