/****************************************************************************
*
* By John.Gulbrandsen@SummitSoftConsulting.com, 2/27/2007.
*
* This file contains the program store used by the PIC10 RISC CPU core.
*
****************************************************************************/

module pic10_program_store(
	
	///////////////////////////////////////////////////////////////
	/////////////////////////// Outputs ///////////////////////////
	///////////////////////////////////////////////////////////////
	
	output [11:0] program_bus,	// Current instruction from the program memory at
								// the address input at pc_bus.

	///////////////////////////////////////////////////////////////
	/////////////////////////// Inputs ////////////////////////////
	///////////////////////////////////////////////////////////////

	input [8:0] pc_bus,			// The program memory address. Will result in
								// program instructions to be received on the
								// 'program_bus' bus.
	);
	
	////////////////////////////////////////////////////////////////////////////////////
	// Hook up the Program Store.
	////////////////////////////////////////////////////////////////////////////////////
	
	// The encoded instructions are stored here.
	reg [11:0] ProgramStore [512];
	
	// Wire up the Program Store.
	assign program_bus = ProgramStore[pc_bus];
	
	// The instructions are stored here in string format. This is so we can display the
	// current instructions and operands in the Wave window while debugging the code.
	// We here declare storage for 512 20-byte strings.
	reg [20*7:0]  ProgramStoreText [512];
	
	////////////////////////////////////////////////////////////////////////////////////	
	// Initialize the program store with instructions to be executed.
	////////////////////////////////////////////////////////////////////////////////////
   
	initial
	begin
		// Address 000h: Load an operand into the 'W' Register with the MOVLW Instruction.
		ProgramStore[0] = 12'b1100_0100_0100; // C44h. Format 1100_kkkk_kkkk.
		ProgramStoreText[0] = "MOVLW 44h";
		
		// Address 001h: Move the 'W' register into an 'f' register with the "MOVWF f"
		// instruction. This is needed so that we later can add the 'W' register with the 
		// 'f' register. We choose to use 'f' register 8 which is the first RAM register.
		// NOTE that we are using a direct address to address the target register.
		ProgramStore[1] = 12'b0000_0010_1000; // 028h. Format 0000_001f_ffff.
		ProgramStoreText[1] = "MOVWF 8h";
		
		// Address 002h: ADDWF: Add the 'W' register (data: 44h) with 'f' register 8 (data: 44h)
		// NOTE: The 'd' and 'fffff' fields determine the source and destination registers.
		// We store the result (88h) in the 'W' register since d == 0.		
		ProgramStore[2] = 12'b0001_1100_1000; // 1C8h. Format 0001_11df_ffff.
		ProgramStoreText[2] = "ADDWF 8, d";

		// Address 004h: NOP. No 'load' signals should be asserted.
		ProgramStore[3] = 12'b000_0000_0000;
		ProgramStoreText[3] = "NOP";
		
		// Address 004h: ANDWF: AND the 'W' register (data: 88h) with 'f' register 8 (data: 44h).
		// NOTE: The 'd' and 'fffff' fields determine the source and destination registers.
		// We store the result (00h) in the 'W' register since d == 0.		
		ProgramStore[4] = 12'b0001_0100_1000; // 148h. Format 0001_01df_ffff.
		ProgramStoreText[4] = "ANDWF 8, d";
		
		// Address 005h: IORWF: OR the 'W' register (data: 00h) with 'f' register 8 (data: 44h).
		// NOTE: The 'd' and 'fffff' fields determine the source and destination registers.
		// We store the result (44h) in the 'W' register since d == 0.		
		ProgramStore[5] = 12'b0001_0000_1000; // 108h. Format 0001_00df_ffff.
		ProgramStoreText[5] = "IORWF 8, d";
		
		// Address 006h: CLRF: Clear the 'f' register (data: 44h => 00h).
		ProgramStore[6] = 12'b0000_0110_1000; // 068h. Format 0000 011f ffff.
		ProgramStoreText[6] = "CLRF 8";

		// Address 007h: MOVF f, d: Moves the 'f' register (data: 00h) to
		// either the same 'f' register (if d==1) or to the 'W' register 
		// (if d==0). We move f=>W. 'W' should change from 44h to 00h.
		ProgramStore[7] = 12'b0010_0000_1000; // 208h. Format 0010_00df_ffff.
		ProgramStoreText[7] = "MOVF 8, d";
		
		// Address 008h: COMF f, d: Complements the 'f' register 4 (data: 00h) and
		// stores the result in either the same 'f' register (if d==1) or to the 
		// 'W' register (if d==0). We here store the result in 'W'. 'W' should 
		// change from 00h to FFh since 'f' register number 4 is cleared (never
		// touched since reset). Note that 'f' register 4 is the FSR register.
		ProgramStore[8] = 12'b0010_0100_0100; // 244h. Format 0010_01df_ffff.
		ProgramStoreText[8] = "COMF 4, d";

		// Address 009h: CLRW: Clears the 'W' register (FFh => 00h)
		ProgramStore[9] = 12'b0000_0100_0000; // 040h. Format 0000_0100_0000.
		ProgramStoreText[9] = "CLRW";
		
		// Address 00Ah: DECF f, d: Decrement register 'f' and store result in 
		// either the 'f' register (d==1) or in the 'W' register (d==0). We
		// here decrement the FSR register ('f' register 4) and store the result
		// back into the FSR register. FSR should change from 00h to ffh.
		ProgramStore[12'h00A] = 12'b0000_1110_0100; // 0E4h. Format 0000_11df_ffff.
		ProgramStoreText[12'h00A] = "DECF FSR, f";
		
		////////////////////////////////////////////////////////////////////////////////////	
		// Test code to validate the incf instruction.
		// These instructions loads 01h into 'f' register 8 and then decrements the register.
		// Since the result is 00h the "MOVLW 55h" should be skipped over and the
		// "MOVLW AAh" instruction should instead be executed.
		////////////////////////////////////////////////////////////////////////////////////
		
		ProgramStore[12'h00B] = 12'b1100_0000_0001; // C01h. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h00B] = "MOVLW 1";

		ProgramStore[12'h00C] = 12'b0000_0010_1000; // 028h. Format: 0000 001f ffff
		ProgramStoreText[12'h00C] = "MOVWF 8";
		
		ProgramStore[12'h00D] = 12'b0010_1110_1000; // 1E8h. Format: 0010 11df ffff
		ProgramStoreText[12'h00D] = "DECFSZ 8, f";

		ProgramStore[12'h00E] = 12'b1100_0101_0101;	// C55h. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h00E] = "MOVLW 55h";

		ProgramStore[12'h00F] = 12'b1100_1010_1010;	// CAAh. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h00F] = "MOVLW AAh";

		////////////////////////////////////////////////////////////////////////////////////	
		// Test code to validate the INCF instruction.
		// These instructions loads FFh into 'f' register 4 (FSR) and then increments the 
		// register. The resulting FSE should be 00h and the Z flag should be set in the 
		// STATUS register.
		////////////////////////////////////////////////////////////////////////////////////
		
		ProgramStore[12'h010] = 12'b1100_1111_1111; // CFFh. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h010] = "MOVLW FF";

		ProgramStore[12'h011] = 12'b0000_0010_0100; // 024h. Format: 0000 001f ffff
		ProgramStoreText[12'h011] = "MOVWF 4";
		
		ProgramStore[12'h012] = 12'b0010_1010_0100; // 2A4h. Format: 0010 10df ffff
		ProgramStoreText[12'h012] = "INCF 4, f";

		////////////////////////////////////////////////////////////////////////////////////	
		// Test code to validate the INCFSZ instruction.
		// These instructions loads FFh into 'f' register 31 and then increments the register.
		// Since the result is 00h the "MOVLW 55h" should be skipped over and the
		// "MOVLW AAh" instruction should instead be executed.
		////////////////////////////////////////////////////////////////////////////////////
		
		ProgramStore[12'h013] = 12'b1100_1111_1111; // C01h. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h013] = "MOVLW FFh";

		ProgramStore[12'h014] = 12'b0000_0011_1111; // 03Fh. Format: 0000 001f ffff
		ProgramStoreText[12'h014] = "MOVWF 1Fh";
		
		ProgramStore[12'h015] = 12'b0011_1111_1111; // 3FFh. Format: 0011 11df ffff
		ProgramStoreText[12'h015] = "INCFSZ 1Fh, f";

		ProgramStore[12'h016] = 12'b1100_0101_0101;	// C55h. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h016] = "MOVLW 55h";

		ProgramStore[12'h017] = 12'b1100_1010_1010;	// CAAh. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h017] = "MOVLW AAh";
		
		////////////////////////////////////////////////////////////////////////////////////	
		// Test code to validate the RLF instruction (Rotate Left 'f' through carry).
		// These instructions load 80h into 'f' register A and rotates it left twice.
		// Register A should contain 00h with Carry set after the first RLF instruction.
		// Register A should contain 01h with Carry clear after the second RLF instruction.
		////////////////////////////////////////////////////////////////////////////////////	

		ProgramStore[12'h018] = 12'b1100_1000_0000; // C80h. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h018] = "MOVLW 80h";

		ProgramStore[12'h019] = 12'b0000_0010_1010; // 02Ah. Format: 0000 001f ffff
		ProgramStoreText[12'h019] = "MOVWF Ah";
		
		ProgramStore[12'h01A] = 12'b0011_0110_1010; // 36Ah. Format: 0011 01df ffff
		ProgramStoreText[12'h01A] = "RLF Ah, f"; // 'f' register A should contain 00h, C should be set.

		ProgramStore[12'h01B] = 12'b0011_0110_1010; // 36Ah. Format: 0011 01df ffff
		ProgramStoreText[12'h01B] = "RLF Ah, f"; // 'f' register A should contain 01h, C should be clear.

		////////////////////////////////////////////////////////////////////////////////////	
		// Test code to validate the RRF instruction (Rotate Right 'f' through carry).
		// These instructions load 01h into 'f' register A and rotates it right twice.
		// Register A should contain 00h with Carry set after the first RRF instruction.
		// Register A should contain 10h with Carry clear after the second RRF instruction.
		////////////////////////////////////////////////////////////////////////////////////	

		ProgramStore[12'h01C] = 12'b1100_0000_0001; // C01h. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h01C] = "MOVLW 01h";

		ProgramStore[12'h01D] = 12'b0000_0010_1010; // 02Ah. Format: 0000 001f ffff
		ProgramStoreText[12'h01D] = "MOVWF Ah";
		
		ProgramStore[12'h01E] = 12'b0011_0010_1010; // 32Ah. Format: 0011 00df ffff
		ProgramStoreText[12'h01E] = "RRF Ah, f"; // 'f' register A should contain 00h, C should be set.

		ProgramStore[12'h01F] = 12'b0011_0010_1010; // 32Ah. Format: 0011 00df ffff
		ProgramStoreText[12'h01F] = "RRF Ah, f"; // 'f' register A should contain 01h, C should be clear.

		////////////////////////////////////////////////////////////////////////////////////	
		// Test code to validate the SUBWF instruction (subtract 'W' from 'f').
		// These instructions loads 88h into 'f' register Fh, 44h into the 'W' register 
		// and then subtracts the 'W' register from 'f' register Fh. The result (44h) is
		// stored into the 'W' register.
		////////////////////////////////////////////////////////////////////////////////////	
		
		ProgramStore[12'h020] = 12'b1100_0100_0100; // C44h. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h020] = "MOVLW 44h";

		ProgramStore[12'h021] = 12'b0000_0010_1111; // 02Fh. Format: 0000 001f ffff
		ProgramStoreText[12'h021] = "MOVWF Fh";
		
		ProgramStore[12'h022] = 12'b0011_0110_1111; // 36Ah. Format: 0011 01df ffff
		ProgramStoreText[12'h022] = "RLF Fh, f";	// Multiply by 2 to put 88h into f.
		
		ProgramStore[12'h023] = 12'b0000_1000_1111; // 08Fh. Format: 0000 10df ffff
		ProgramStoreText[12'h023] = "SUBWF Fh, w";	// W = f - W = 44h
		
		////////////////////////////////////////////////////////////////////////////////////	
		// Test code to validate the SWAPF instruction (swap nybbles in 'f').
		// These instructions loads 5Ah into 'f' register Ch and then swaps the nybbles 
		// in the 'f' register. The result (A5h) is stored back into 'f' register Ch.
		////////////////////////////////////////////////////////////////////////////////////	
		
		ProgramStore[12'h024] = 12'b1100_0101_1010; // C5Ah. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h024] = "MOVLW 5Ah";

		ProgramStore[12'h025] = 12'b0000_0010_1100; // 02Ch. Format: 0000 001f ffff
		ProgramStoreText[12'h025] = "MOVWF Ch";
		
		ProgramStore[12'h026] = 12'b0011_1010_1100; // 3ACh. Format: 0011 10df ffff
		ProgramStoreText[12'h026] = "SWAPF Ch, f";	// Swap nybbles of 'f' register Ch.

		////////////////////////////////////////////////////////////////////////////////////	
		// Test code to validate the XORWF instruction (XOR 'W' with 'f').
		// These instructions loads 55h into 'f' register Ch, F0h into the 'W' register
		// and then XOR's 'W' with 'f' register Ch. The result (A5h) is stored into
		// the 'W' register.
		////////////////////////////////////////////////////////////////////////////////////	
		
		ProgramStore[12'h027] = 12'b1100_0101_0101; // C55h. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h027] = "MOVLW 55h";

		ProgramStore[12'h028] = 12'b0000_0010_1100; // 02Ch. Format: 0000 001f ffff
		ProgramStoreText[12'h028] = "MOVWF Ch";

		ProgramStore[12'h029] = 12'b1100_1111_0000; // CF0h. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h029] = "MOVLW F0h";
		
		ProgramStore[12'h02A] = 12'b0001_1000_1100; // 18Ch. Format: 0001 10df ffff
		ProgramStoreText[12'h02A] = "XORWF Ch, w";	// 'W' = 'W' XOR 'f' = A5h.

		////////////////////////////////////////////////////////////////////////////////////	
		// Test code to validate the BCF instruction (clear bit 'b' in register 'f').
		// These instructions load FFh into 'f' register 8 and then clears bit 7 in register 
		// 'f'. The result (7F5h) is stored back into the 'f' register. NOTE that we are using
		// the 'FSR' register to generate an INDIRECT address to address the 'f' register.
		////////////////////////////////////////////////////////////////////////////////////	
		
		ProgramStore[12'h02B] = 12'b1100_0000_1000; // C08h. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h02B] = "MOVLW 8";		// Prepare to load FSR with 8.

		ProgramStore[12'h02C] = 12'b0000_0010_0100; // 024h. Format: 0000 001f ffff
		ProgramStoreText[12'h02C] = "MOVWF 4h";		// Load FSR with 8.
		
		ProgramStore[12'h02D] = 12'b1100_1111_1111; // CFFh. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h02D] = "MOVLW FFh";	// Prepare to load register 8 with FFh.
		
		ProgramStore[12'h02E] = 12'b0000_0010_0000; // 020h. Format: 0000 001f ffff
		ProgramStoreText[12'h02E] = "MOVWF 0";		// Load register 8 with FFh by using indirect address in FSR.
		
		ProgramStore[12'h02F] = 12'b0100_1110_1000; // 4E8h. Format: 0100 bbbf ffff
		ProgramStoreText[12'h02F] = "BCF 0, 7";		// Clear bit 7 of register 8 by using indirect address in FSR.
		
		////////////////////////////////////////////////////////////////////////////////////	
		// Test code to validate the BSF instruction (clear bit 'b' in register 'f').
		// This instruction sets the same bit (7) in 'f' register 8 that was cleared above.
		////////////////////////////////////////////////////////////////////////////////////	
			
		ProgramStore[12'h030] = 12'b0101_1110_1000; // 5E8h. Format: 0101 bbbf ffff
		ProgramStoreText[12'h030] = "BSF 8, 7";		// Set bit 7 of register 8. Note: Now using direct address.

		////////////////////////////////////////////////////////////////////////////////////	
		// Test code to validate the BTFSC instruction (test bit 'b' in 'f', skip next 
		// instruction if bit is clear). These instructions load FFh into 'f' register Dh
		// and then executes the BTFSC instruction to check if bit 6 is clear in 'f' register
		// Dh. Since we just loaded FFh into 'f' register Dh the bit is not clear so the next
		// instruction is NOT skipped). We then clear bit 6 in 'f' register D and execute
		// the same BTFSC instruction again. This time bit 6 is clear so the next instruction
		// is skipped. We have put an DECF instruction at the end of the instruction sequence 
		// to verify that the next instruction (DECF in this case) is really skipped. By
		// 'skipping' an instruction we mean that the datapath will execute a NOP instruction
		// instead of the instruction currently on the Program Bus.
		////////////////////////////////////////////////////////////////////////////////////	
		
		ProgramStore[12'h031] = 12'b1100_1111_1111; // CFFh. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h031] = "MOVLW FFh";	// Constant to below load into 'f' register Dh.

		ProgramStore[12'h032] = 12'b0000_0010_1101; // 02Dh. Format: 0000 001f ffff
		ProgramStoreText[12'h032] = "MOVWF Dh";		// Store the FFh constant in 'f' register Dh.

		ProgramStore[12'h033] = 12'b0110_1100_1101; // 6CDh. Format: 0110 bbbf ffff
		ProgramStoreText[12'h033] = "BTFSC Dh, 6";	// Will NOT skip next instruction since SFR[Dh].6 is 1.
		
		ProgramStore[12'h034] = 12'b0100_1100_1101; // 4CDh. Format: 0100 bbbf ffff
		ProgramStoreText[12'h034] = "BCF Dh, 6";	// Clear bit 6 of register Dh.

		ProgramStore[12'h035] = 12'b0110_1100_1101; // 6CDh. Format: 0110 bbbf ffff
		ProgramStoreText[12'h035] = "BTFSC Dh, 6";	// WILL skip next instruction since SFR[Dh].6 is now 0.

		ProgramStore[12'h036] = 12'b0000_1110_1101; // 0EDh. Format 0000_11df_ffff.
		ProgramStoreText[12'h036] = "DECF Dh, f";	// Dummy instruction that should be replaced by NOP.

		////////////////////////////////////////////////////////////////////////////////////	
		// Test code to validate the BTFSS instruction (test bit 'b' in 'f', skip next 
		// instruction if bit is set). These instructions load FEh into 'f' register Dh
		// and then executes the BTFSS instruction to check if bit 0 is set in 'f' register
		// Dh. Since we just loaded FEh into 'f' register Dh the bit is not set so the next
		// instruction is NOT skipped). We then set bit 0 in 'f' register D and execute
		// the same BTFSS instruction again. This time bit 0 is set so the next instruction
		// is skipped. We have put an DECF instruction at the end of the instruction sequence 
		// to verify that the next instruction (DECF in this case) is really skipped. By
		// 'skipping' an instruction we mean that the datapath will execute a NOP instruction
		// instead of the instruction currently on the Program Bus.
		////////////////////////////////////////////////////////////////////////////////////	
		
		ProgramStore[12'h037] = 12'b1100_1111_1110; // CFEh. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h037] = "MOVLW FEh";	// Constant to below load into 'f' register Dh.

		ProgramStore[12'h038] = 12'b0000_0010_1101; // 02Dh. Format: 0000 001f ffff
		ProgramStoreText[12'h038] = "MOVWF Dh";		// Store the FEh constant in 'f' register Dh.

		ProgramStore[12'h039] = 12'b0111_0000_1101; // 70Dh. Format: 0111 bbbf ffff
		ProgramStoreText[12'h039] = "BTFSS Dh, 0";	// Will NOT skip next instruction since SFR[Dh].0 is 0.
		
		ProgramStore[12'h03A] = 12'b0101_0000_1101; // 50Dh. Format: 0101 bbbf ffff
		ProgramStoreText[12'h03A] = "BSF Dh, 0";	// Set bit 6 of register Dh.

		ProgramStore[12'h03B] = 12'b0111_0000_1101; // 70Dh. Format: 0111 bbbf ffff
		ProgramStoreText[12'h03B] = "BTFSS Dh, 0";	// WILL skip next instruction since SFR[Dh].0 is now 1.

		ProgramStore[12'h03C] = 12'b0000_1110_1101; // 0EDh. Format 0000_11df_ffff.
		ProgramStoreText[12'h03C] = "DECF Dh, f";	// Dummy instruction that should be replaced by NOP.
			
		////////////////////////////////////////////////////////////////////////////////////	
		// Code to validate the 'ANDLW k' instruction.
		////////////////////////////////////////////////////////////////////////////////////	
		
		ProgramStore[12'h03D] = 12'b1100_1010_1111; // CFEh. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h03D] = "MOVLW AFh";	// 'W' = AFh.
		
		ProgramStore[12'h03E] = 12'b1110_0101_0101; // E55h. Format: 1110 kkkk kkkk.
		ProgramStoreText[12'h03E] = "ANDLW 55h";	// 'W' = 'W' & 55h = AFh & 55F = 05h.

		////////////////////////////////////////////////////////////////////////////////////	
		// Code to validate the 'IORLW k' instruction.
		////////////////////////////////////////////////////////////////////////////////////	
		
		ProgramStore[12'h03F] = 12'b1100_1010_1010; // CAAh. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h03F] = "MOVLW AAh";	// 'W' = AAh.
		
		ProgramStore[12'h040] = 12'b1101_0101_0101; // D55h. Format: 1101 kkkk kkkk.
		ProgramStoreText[12'h040] = "IORLW 55h";	// 'W' = 'W' | 55h = AAh | 55h = FFh.

		////////////////////////////////////////////////////////////////////////////////////	
		// Code to validate the 'XORLW k' instruction.
		////////////////////////////////////////////////////////////////////////////////////	
		
		ProgramStore[12'h041] = 12'b1100_1010_0101; // CA5h. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h041] = "MOVLW A5h";	// 'W' = A5h.
		
		ProgramStore[12'h042] = 12'b1111_1111_1111; // FFFh. Format: 1111 kkkk kkkk.
		ProgramStoreText[12'h042] = "XORLW FFh";	// 'W' = 'W' ^ FFh = A5h | FFh = 5Ah.

		////////////////////////////////////////////////////////////////////////////////////	
		// Code to validate the 'GOTO' instruction. We simply jump forward to the next
		// instruction sequence. 
		////////////////////////////////////////////////////////////////////////////////////	

		ProgramStore[12'h043] = 12'b1010_0100_0110; // A46h. Format: 101k kkkk kkkk.
		ProgramStoreText[12'h043] = "GOTO 046h";

		ProgramStore[12'h044] = 12'b000_0000_0000;
		ProgramStoreText[12'h044] = "NOP";			// This should never be executed.

		////////////////////////////////////////////////////////////////////////////////////	
		// This is a subroutine used to validate the CALL instruction in the next instruction
		// sequence. It simply returns to the called with 55h placed into the 'W' register.
		////////////////////////////////////////////////////////////////////////////////////	

		ProgramStore[12'h045] = 12'b1000_0101_0101; // 855h. Format: 1000 kkkk kkkk.
		ProgramStoreText[12'h045] = "RETLW 55h";

		////////////////////////////////////////////////////////////////////////////////////	
		// Code to validate the 'CALL' instruction. This code sequence should result in
		// a call to the subroutine at address 045h. After the subroutine returns we should 
		// continue the execution at address 047h after this instruction.
		////////////////////////////////////////////////////////////////////////////////////	

		ProgramStore[12'h046] = 12'b1001_0100_0101; // 945h. Format: 1001 kkkk kkkk.
		ProgramStoreText[12'h046] = "CALL 045h";

		////////////////////////////////////////////////////////////////////////////////////	
		// Code to validate the 'TRIS f' instruction. This code sequence should result in
		// the TRIS5 register being loaded with 55h, the TRIS6 register being loaded with
		// AAh and the TRIS7 register being loaded with 5Ah. NOTE that the standard PIC
		// only has a single I/O port (6) while we implement two more (5, 7) because we don't
		// implement the SFR registers 5 and 7 (OSCCAL and CMCON0 registers respectively).
		////////////////////////////////////////////////////////////////////////////////////	
		
		ProgramStore[12'h047] = 12'b1100_0101_0101; // C55h. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h047] = "MOVLW 55h";	// Prepare to load TRIS5.

		ProgramStore[12'h048] = 12'b0000_0000_0101; // 005h. Format: 0000 0000 0fff.
		ProgramStoreText[12'h048] = "TRIS 5h";		// Load TRIS5.
		
		ProgramStore[12'h049] = 12'b1100_1010_1010; // CAAh. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h049] = "MOVLW AAh";	// Prepare to load TRIS6.

		ProgramStore[12'h04A] = 12'b0000_0000_0110; // 006h. Format: 0000 0000 0fff.
		ProgramStoreText[12'h04A] = "TRIS 6h";		// Load TRIS6.

		ProgramStore[12'h04B] = 12'b1100_0101_1010; // C5Ah. Format: 1100 kkkk kkkk.
		ProgramStoreText[12'h04B] = "MOVLW 5Ah";	// Prepare to load TRIS7.

		ProgramStore[12'h04C] = 12'b0000_0000_0111; // 007h. Format: 0000 0000 0fff.
		ProgramStoreText[12'h04C] = "TRIS 7h";		// Load TRIS7.

		////////////////////////////////////////////////////////////////////////////////////	
		// End of simulation: Hang in a loop.
		////////////////////////////////////////////////////////////////////////////////////	
		
		ProgramStore[12'h04D] = 12'b1010_0100_1101; // A4Dh. Format: 101k kkkk kkkk.
		ProgramStoreText[12'h04D] = "GOTO 04Dh";
	
			
	end	

	
endmodule
