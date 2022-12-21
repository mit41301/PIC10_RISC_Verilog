/****************************************************************************
*
* By John.Gulbrandsen@SummitSoftConsulting.com, 2/26/2007.
*
* This file contains the controller used in the PIC10 RISC CPU core.
*
* The controller is software compatible with the PIC10F100 microcontroller
* series from Microchip. Note that the controller is not cycle-accurate 
* because this implementation is not pipelined in order to keep the imple-
* mentation small. The higher clock frequency of a CPLD/FPGA will more than
* compensate for this.
*
* Register differences: TMR0 is not implemented. OSCCAL and CMCON0 registers
* have been replaced by two more GPIO registers. PC is 9-bits (no banking 
* support in the PIC10F100-series so there's no need to implement more bits).
*
****************************************************************************/

module pic10_controller(
	
	///////////////////////////////////////////////////////////////
	/////////////////////////// Outputs ///////////////////////////
	///////////////////////////////////////////////////////////////
	
	output reg load_status_reg,	// Loads the alu_bus into STATUS register on next
								// posedge(clk).
								
	output wire alu_mux_sel,	// Selects the 2nd operand to ALU to be 
								// sft_data_bus (0) or ram_data_bus(1).
								
	output reg load_w_reg,		// Loads alu_bus into W register on next 
								// posedge(clk).
								
	output reg load_fsr_reg,	// Loads alu_bus into FSR register on next 
								// posedge(clk). 
								
	output reg load_ram_reg,	// Loads alu_bus into the addressed general purpose
								// register on next posedge(clk). 
								
	output wire reg_addr_mux_sel,// Selects the register address to be used when reading or writing a
								// register. Can either be a direct address (0) or an indirect address (1).
								
	output reg load_tris5_reg,	// Used to load the TRIS registers. A '1' in a
	output reg load_tris6_reg,	// bit position makes the corresponding bit in 
	output reg load_tris7_reg,	// the related GPIO register tri-stated (inputs).
								// The TRIS registers are reset as FFh (inputs).
								
	output reg load_gpio5_reg,	// Used to load the GPIO registers. Output data
	output reg load_gpio6_reg,	// is only driven out on the I/O pin if the
	output reg load_gpio7_reg,	// corresponding TRIS bit is '0' (output).

	output reg inc_stack,		// Increments or decrements the stack pointer on 
	output reg dec_stack,		// next posedge(clk). 
								
	output reg load_stack,		// Loads the current stack location with the
								// data on the pc_bus (used by the CALL instruction
								// to store the return address onto the stack).			
								
	output reg load_pc_reg,		// loads PC register from pc_mux_bus (used by
								// the CALL instruction to load the jump address
								// or used to directly load a new value into PCL
								// when PCL is used as the target register).
								
	output reg inc_pc,			// Increments the program counter (PC). Used
								// after instruction fetch as well as in 'skip'
								// instructions (DECFSZ, INCFSZ, BTFSC and BTFSS).
								
	output reg [1:0] pc_mux_sel,// Chooses whether the load data to PC should come
								// from the stack_bus (0), alu_bus (1) or 
								// ir_reg_bus (2).
								
	output reg load_ir_reg,		// Loads IR with the contents of the program 
								// memory word currently being addressed by PC.

	output reg skip_next_instruction, 
								// If the previous instruction was DECFSZ, INCFSZ, 
								// BTFSC or BTFSS and the result was zero we should 
								// not execute the next instruction (i.e. we should 
								// treat the next instruction as a NOP).

	///////////////////////////////////////////////////////////////
	/////////////////////////// Inputs ////////////////////////////
	///////////////////////////////////////////////////////////////

	input zero_result,			// Lets the controller know the state of Z-flag 
								// updates so it knows when to increment the PC
								// during execution of the DECFSZ, INCFSZ, BTFSZ
								// and BTFSS instructions.
	
	input [4:0] reg_addr_bus,	// Used by the controller so it knows which 
								// load_XXX_reg signal to assert when writing to
								// a register via a direct or indirect address.
								
	input [11:0] ir_reg_bus,	// Passes the current instruction code to the
								// controller so it knows how to direct the
								// datapath to execute the instruction.
								
	input reset,				// System reset.
	input clk					// System clock.
								
	);
	
	////////////////////////////////////////////////////////////////////////////////////	
	// We make these mux select signals continuously assigned to ensure the muxes are
	// updated before the 'load' signals are asserted as instructions are executed.
	////////////////////////////////////////////////////////////////////////////////////

	// Determine if a direct or indirect address is used (a direct address is encoded
	// in the 'fffff' field in the instruction word while an indirect address is
	// taken from the FSR register). An indirect address should be used if the 'fffff'
	// field in the instruction word is 00000. The 'reg_addr_mux_sel' signal should be
	// asserted if an indirect address is used so the Register Address Mux will drive
	// the target address onto the 'reg_addr_bus' bus.
	
	assign reg_addr_mux_sel = (ir_reg_bus[4:0] == 0);
	
	// Setup the datapath for the 2nd ALU operand via the ALU mux (the 1st operand is
	// the 'W' register). The 2nd operand is taken from the sfr_bus if the 'fffff' 
	// instruction field is less than 8 (there are eight Special Function Registers). 
	// If 'fffff' > 7 the operand data is taken from the ram_data_bus which is fed 
	// from the currently addressed RAM register (addressed by the 'fffff' instruction
	// field).

	assign alu_mux_sel = (ir_reg_bus[4:0] > 7);

	////////////////////////////////////////////////////////////////////////////////////
	// Control the datapath as each instruction is being executed.
	// NOTE: The controller signals are setup on the falling edge of the clock signal
	// so they are stable when the datapath clocks in data on the rising clock edge.
	////////////////////////////////////////////////////////////////////////////////////

	reg initial_ir_load_done = 0;
	
	always @(negedge clk)
	begin
	
		// Deassert all controller signals. The relevant signal 
		// for the current instruction will be asserted below 
		// unless reset is asserted in which case they will remain
		// deasserted until the next falling clock edge.
		
		load_status_reg = 0;
		load_w_reg = 0;
		load_fsr_reg = 0;
		load_ram_reg = 0;
		load_tris5_reg = 0;
		load_tris6_reg = 0;
		load_tris7_reg = 0;
		load_gpio5_reg = 0;
		load_gpio6_reg = 0;
		load_gpio7_reg = 0;
		inc_stack = 0;
		dec_stack = 0;
		load_stack = 0;
		load_pc_reg = 0;
		inc_pc = 0;
		pc_mux_sel = 0;			
		load_ir_reg = 0;		
	
		skip_next_instruction = 0;
	
		if(reset == 0 && initial_ir_load_done == 0) begin
		
			// Load the instruction word at the current address into the Instruction Register.
			// NOTE: We also increment the PC so the next instruction word is loaded while the
			// current instruction is being executed.
			load_ir_reg = 1;
			inc_pc = 1;

			// The initial 'IR' Register load will be done on the next positive edge of clock.
			@(posedge clk);
		
			initial_ir_load_done = 1;
			
		end
		else begin

			// Assert the control signals accordingly to the instruction in the Instruction Register.
			casex(ir_reg_bus)
				// Mnemonic	Operands	Description				Cycles	12-Bit Opcode	Status Affected		Notes

				// ADDWF	f, d		Add W and f				1		0001 11df ffff		C, DC, Z		1, 2, 4
				12'b0001_11xx_xxxx: addwf();
					
				// ANDWF	f, d		AND W with f			1		0001 01df ffff		Z				2, 4
				12'b0001_01xx_xxxx: andwf();
				
				// CLRF		f			Clear f					1		0000 011f ffff		Z				4
				12'b0000_011x_xxxx:	crlf();
				
				// CLRW		—			Clear W					1		0000 0100 0000		Z	
				12'b0000_0100_0000:	clrw();
				
				// COMF		f, d		Complement f			1		0010 01df ffff		Z	
				12'b0010_01xx_xxxx: comf();
				
				// MOVWF	f			Move W to f				1		0000 001f ffff		None			1, 4
				12'b0000_001x_xxxx: movwf();

				// OPTION	—			Load OPTION register	1		0000 0000 0010		None
				12'b0000_0000_0010: option();
				
				// DECF		f, d		Decrement f				1		0000 11df ffff		Z				2, 4
				12'b0000_11xx_xxxx:	decf();
				
				// DECFSZ	f, d		Decrement f, Skip if 0	1(2)	0010 11df ffff		None			2, 4
				12'b0010_11xx_xxxx: decfsz();
				
				// INCF		f, d		Increment f				1		0010 10df ffff		Z				2, 4
				12'b0010_10xx_xxxx: incf();
				
				// INCFSZ	f, d		Increment f, Skip if 0	1(2)	0011 11df ffff		None			2, 4
				12'b0011_11xx_xxxx: incfsz();
				
				// IORWF	f, d		Inclusive OR W with f	1		0001 00df ffff		Z				2, 4
				12'b0001_00xx_xxxx: iorwf();
				
				// MOVF		f, d		Move f					1		0010 00df ffff		Z				2, 4
				12'b0010_00xx_xxxx: movf();
				
				// NOP		—			No Operation			1		0000 0000 0000		None	
				12'b0000_0000_0000: nop();
				
				// RLF		f, d		Rotate left f			1		0011 01df ffff		C				2, 4
				//						through Carry	
				12'b0011_01xx_xxxx: rlf();
				
				// RRF		f, d		Rotate right f			1		0011 00df ffff		C				2, 4
				//						through Carry	
				12'b0011_00xx_xxxx: rrf();						
										
				// SUBWF	f, d		Subtract W from f		1		0000 10df ffff		C, DC, Z		1, 2, 4
				12'b0000_10xx_xxxx: subwf();
				
				// SWAPF	f, d		Swap f					1		0011 10df ffff		None			2, 4
				12'b0011_10xx_xxxx: swapf();
				
				// XORWF	f, d		Exclusive OR W with f	1		0001 10df ffff		Z				2, 4
				12'b0001_10xx_xxxx: xorwf();

				// BCF		f, b		Bit Clear f				1		0100 bbbf ffff		None			2, 4
				12'b0100_xxxx_xxxx: bcf();
				
				// BSF		f, b		Bit Set f				1		0101 bbbf ffff		None			2, 4
				12'b0101_xxxx_xxxx: bsf();
				
				// BTFSC	f, b		Bit Test f,				1(2)	0110 bbbf ffff		None	
				//						Skip if Clear	
				12'b0110_xxxx_xxxx: btfsc();
		
				// BTFSS	f, b		Bit Test f,				1(2)	0111 bbbf ffff		None	
				//						Skip if Set	
				12'b0111_xxxx_xxxx: btfss();

				// ANDLW	k			AND literal with W		1		1110 kkkk kkkk		Z
				12'b1110_xxxx_xxxx: andlw();
					
				// CALL		k			Call Subroutine			2		1001 kkkk kkkk		None			1
				12'b1001_xxxx_xxxx: call();
				
				// CLRWDT	-			Clear Watchdog Timer	1		0000 0000 0100		TO, PD
				// NOTE: Not implemented instruction.
					
				// GOTO		k			Unconditional branch	2		101k kkkk kkkk		None
				12'b101x_xxxx_xxxx: goto();
					
				// IORLW	k			Inclusive OR literal	1		1101 kkkk kkkk		Z	
				//						with W	
				12'b1101_xxxx_xxxx: iorlw();
										
				// MOVLW	k			Move literal to W		1		1100 kkkk kkkk		None
				12'b1100_xxxx_xxxx: movlw();
					
				// RETLW	k			Return, place Literal	2		1000 kkkk kkkk		None	
				//						in W	
				12'b1000_xxxx_xxxx: retlw();
				
				// TRIS		f			Load TRIS register		1		0000 0000 0fff		None			3
				12'b0000_0000_0101: tris();		// Load TRIS5
				12'b0000_0000_0110: tris();		// Load TRIS6
				12'b0000_0000_0111: tris();		// Load TRIS7
				
				// SLEEP	—			Go into Standby mode	1		0000 0000 0011		TO, PD	
				// NOTE: Not implemented instruction.
				
				// XORLW	k			Exclusive OR literal	1		1111 kkkk kkkk		Z	
				// 						to W	
				12'b1111_xxxx_xxxx: xorlw();

			endcase

			// Load the next instruction into IR by asserting the 'load_ir_reg' signal.
			load_ir_reg = 1;
			
			// Increment the PC on the next posedge(clk) by asserting the 'inc_pc' signal.
			inc_pc = 1;

			// Wait for the datapath to clock in the data.
			@(posedge clk);
			
		end
	end
   
	task LoadTargetRegister();
	begin
		
		// The ALU is driving the result of the instruction onto the alu_bus.
		// We need to assert the correct load_XXX_register to store the result into
		// either the 'W' register (d == 0), into a SFR (f < 8) or into a RAM Register 
		// (f > 7) by using an indirect address in FSR (f == 0) or into a SFR or 
		// RAM register by using a direct address encoded into the instruction word. 

		// If (d == 0) we store the result back in the 'W' register.
		if(ir_reg_bus[5] == 0)
		   load_w_reg = 1;
		else begin
			// The result should be stored back into an 'f' register. 
			LoadTargetFRegister();
		end
	end
	endtask

	task LoadTargetFRegister();
	begin
		
		// The ALU is driving the result of the instruction onto the alu_bus.
		// We need to assert the correct load_XXX_register to store the result into
		// a SFR (f < 8) or into a RAM Register (f > 7) by using an indirect address 
		// in FSR (f == 0) or into a SFR or RAM register by using a direct address 
		// encoded into the instruction word. 

		// Assert the target 'f' register's load signal.
		case(reg_addr_bus)
			5'd0:
				// INDF. Do Nothing since INDF is not writable.
				;
			5'd1:
				// TMR0. Do Nothing since TMR0 is not implemented.
				;
			5'd2:
				// PCL. Load PC[7:0] from alu_bus. PC[8] will be cleared.
				begin
					pc_mux_sel = 0;
					load_pc_reg = 1;
				end
			5'd3:
				// STATUS. Load STATUS register from alu_bus on next posedge(clk).
				load_status_reg = 1;
			5'd4:
				// FSR. Load FSR register from alu_bus on next posedge(clk).
				load_fsr_reg = 1;
			5'd5:
				// GPIO5. Load GPIO5 register from alu_bus on next posedge(clk).
				load_gpio5_reg = 1;
			5'd6:
				// GPIO6. Load GPIO6 register from alu_bus on next posedge(clk).
				load_gpio6_reg = 1;
			5'd7:
				// GPIO7. Load GPIO7 register from alu_bus on next posedge(clk).
				load_gpio7_reg = 1;					
			default:
				// RAM Register.
				load_ram_reg = 1;					
		endcase

	end
	endtask
	   
	task addwf();
	begin
	    
	    ////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'ADDWF f,d' instruction. This instruction adds the content in the 
		// 'W' register with the content of the register indicated by 'f'. The result goes
		// into the register specified by 'd' in the instruction word (0=>W, 1=>f).
		// The instruction format is: 0001_11df_ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: addwf");

		// Load the target register from the alu_bus.
		 LoadTargetRegister();

	end
	endtask
	
	task andwf();
	begin

	    ////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'ANDWF f,d' instruction. This instruction ANDs the content in the 
		// 'W' register with the content of the register indicated by 'f'. The result goes
		// into the register specified by 'd' in the instruction word (0=>W, 1=>f).
		// The instruction format is: 0001_01df_ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: andwf");

		// Load the target register from the alu_bus.
		 LoadTargetRegister();

	end
	endtask
	
	task crlf();
	begin
   
	    ////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'CLRF f' instruction. This instruction clears the content of the 
		// 'f' register by loading 00h from the alu_bus.
		// The instruction format is: 0000_011f_ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: crlf");

		// Load the target register from the alu_bus.
		 LoadTargetRegister();

	end
	endtask	
	
	task clrw();
	begin
      
	    ////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'CLRW' instruction. This instruction clears the content of the 
		// 'W' register by loading 00h from the alu_bus.
		// The instruction format is: 0000_0100_0000.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: clrw");

		// Load the target register from the alu_bus.
		LoadTargetRegister();

	end
	endtask	
	
	task comf();
	begin
      
	    ////////////////////////////////////////////////////////////////////////////////////
		// COMF f, d: Complements the 'f' register and stores the result in either 
		// the same 'f' register (if d==1) or to the 'W' register (if d==0).
		// The instruction format is: 0010_01df_ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: comf");

		// Load the target register from the alu_bus.
		LoadTargetRegister();

	end
	endtask
	
	task decf();
	begin

	    ////////////////////////////////////////////////////////////////////////////////////
		// DECF f, d: Decrement register 'f' and store result in either 
		// the 'f' register (if d==1) or in the 'W' register (if d==0).
		// The instruction format is: 0000_11df_ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: decf");

		// Load the target register from the alu_bus.
		LoadTargetRegister();

	end
	endtask	

	task decfsz();
	begin

	    ////////////////////////////////////////////////////////////////////////////////////
		// DECFSZ f, d: Decrement register 'f' and store result in either 
		// the 'f' register (if d==1) or in the 'W' register (if d==0).
		// Skip the next instruction if the result is 00h.
		// The instruction format is: 0010 11df ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: decfsz");
		
		// If the result of the DECFSZ instruction 
		// is zero we skip the next instruction.
		skip_next_instruction = zero_result;

		// Load the target register from the alu_bus.
		LoadTargetRegister();

	end
	endtask	

	
	task incf();
	begin
      
	    ////////////////////////////////////////////////////////////////////////////////////
		// INCF f, d: Increment register 'f' and store result in either 
		// the 'f' register (if d==1) or in the 'W' register (if d==0).
		// The instruction format is: 0010 10df ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: incf");

		// Load the target register from the alu_bus.
		LoadTargetRegister();

	end
	endtask	

	task incfsz();
	begin
      
	    ////////////////////////////////////////////////////////////////////////////////////
		// INCFSZ f, d: Increment register 'f' and store the result in either 
		// the 'f' register (if d==1) or in the 'W' register (if d==0).
		// Skip the next instruction if the result is 00h.
		// The instruction format is: 0011 11df ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: incfsz");
		
		// If the result of the INCFSZ instruction 
		// is zero we skip the next instruction.
		skip_next_instruction = zero_result;

		// Load the target register from the alu_bus.
		LoadTargetRegister();

	end
	endtask	
	
	task iorwf();
	begin
       
       	////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'IORWF f,d' instruction. This instruction ORs the content in the 
		// 'W' register with the content of the register indicated by 'f'. The result goes
		// into the register specified by 'd' in the instruction word (0=>W, 1=>f).
		// The instruction format is: 0001_00df_ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: iorwf");

		// Load the target register from the alu_bus.
		LoadTargetRegister();

	end
	endtask

	task movf();
	begin
      
   	  ////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'MOVF f,d' instruction. This instruction Moves the 'f' register to
		// either the same 'f' register (if d==1) or to the 'W' register (if d==0).
		// The instruction format is: 0010_00df_ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	   $display("pic10_controller: movf");

		// Load the target register from the alu_bus.
		LoadTargetRegister();
		
	end
	endtask	

	task nop();
	begin
      
		$display("pic10_controller: nop");
		
	end
	endtask	
	
	task movwf();
	begin
	    
   		////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'MOVWF f,d' instruction. This instruction Moves the 'W' register to
		// either an 'f' register (if d==1) or to the 'W' register (if d==0).
		// The instruction format is: 0000_001f_ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: movf");

		// Load the target register from the alu_bus.
		LoadTargetRegister();
    
	end
	endtask

	
	task rlf();
	begin
      
   		////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'RLF f,d' instruction. This instruction rotates 'f' left through 
		// the carry bit. The result is either stored back to the same 'f' register 
		// (if d==1) or to the 'W' register (if d==0).
		// The instruction format is: 0011 01df ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: rlf");

		// Load the target register from the alu_bus.
		LoadTargetRegister();

	end
	endtask

	task rrf();
	begin
      
   		////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'RRF f,d' instruction. This instruction rotates 'f' right through 
		// the carry bit. The result is either stored back to the same 'f' register 
		// (if d==1) or to the 'W' register (if d==0).
		// The instruction format is: 0011 00df ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: rrf");

		// Load the target register from the alu_bus.
		LoadTargetRegister();

	end
	endtask
	
	task subwf();
	begin
	    
   		////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'SUBWF f,d' instruction. This instruction subtracts 'W' from 'f'.
		// The result is either stored back to the same 'f' register (if d==1) or to the 
		// 'W' register (if d==0). All flags are affected by this instruction.
		// The instruction format is: 0000 10df ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: subwf");

		// Load the target register from the alu_bus.
		LoadTargetRegister();
	    
	end
	endtask

	task swapf();
	begin
      
   		////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'SWAPF f,d' instruction. This instruction swaps the nybbles in 'f'.
		// The result is either stored back to the same 'f' register (if d==1) or to the 
		// 'W' register (if d==0). No flags are affected by this instruction.
		// The instruction format is: 0011 10df ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: swapf");

		// Load the target register from the alu_bus.
		LoadTargetRegister();

	end
	endtask
	
	task xorwf();
	begin
      
   		////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'XORWF f,d' instruction. This XOR's the 'W' register with the 'f'
		// register. The result is either stored back to the same 'f' register (if d==1) or 
		// to the 'W' register (if d==0). The 'Z' flag is affected by this instruction.
		// The instruction format is: 0001 10df ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: xorwf");

		// Load the target register from the alu_bus.
		LoadTargetRegister();

	end
	endtask
	
	task bcf();
	begin
      
   		////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'BCF f, b' instruction. This instruction clears bit 'b' in register
		// 'f'. The result is stored back to the same 'f' register. No flags are affected.
		// The instruction format is: 0100 bbbf ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: bcf");

		// Load the target register from the alu_bus.
		LoadTargetFRegister();

	end
	endtask	
		
	task bsf();
	begin
	    
   		////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'BSF f, b' instruction. This instruction sets bit 'b' in register
		// 'f'. The result is stored back to the same 'f' register. No flags are affected.
		// The instruction format is: 0101 bbbf ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: bsf");

		// Load the target register from the alu_bus.
		LoadTargetFRegister();

	end
	endtask	
	
	task btfsc();
	begin
      
   		////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'BTFSC f, b' instruction. This instruction tests bit 'b' in register
		// 'f'. If the bit is clear the next instruction is treated as a NOP.
		// The instruction format is: 0110 bbbf ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: btfsc");

		// If the tested bit is cleared we skip the next instruction.
		skip_next_instruction = zero_result;

		// NOTE: No register is modified by this instruction 
		// so we do not assert any load_xxx_reg signal.

	end
	endtask	

	task btfss();
	begin
      
   		////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'BTFSS f, b' instruction. This instruction tests bit 'b' in register
		// 'f'. If the bit is set the next instruction is treated as a NOP.
		// The instruction format is: 0111 bbbf ffff.
		////////////////////////////////////////////////////////////////////////////////////
	    
	    $display("pic10_controller: btfss");

		// If the tested bit is set we skip the next instruction.
		skip_next_instruction = !zero_result;

		// NOTE: No register is modified by this instruction 
		// so we do not assert any load_xxx_reg signal.

	end
	endtask	
		
	task andlw();
	begin

   		////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'ANDLW k' instruction. This instruction ANDs the 'W' register with
		// the literal 'k' which is encoded in the instruction word. The result is stored
		// back into the 'W' register.
		// The instruction format is: 1110 kkkk kkkk.
		////////////////////////////////////////////////////////////////////////////////////
      
		$display("pic10_controller: andlw");
	      
		// The ALU outputs the result on 'alu_bus' so we only need to load 
		// the result of the AND operation back into the 'W' register.
		load_w_reg = 1;

	end
	endtask	
	
	task call();
	begin
      
   		////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'CALL' instruction. This is a 2-cycle instruction because two 
		// separate tasks need to be done in sequence: 1) The current stack location is
		// loaded with the current Program Counter (PC) contents. Note that PC contains the
		// address of THE NEXT instruction to be executed. The PC is then loaded with the 
		// address of the instruction being jumped to. 2) The IR is loaded with the instruction 
		// word addressed by the (newly updated) updated PC. The stack pointer is also updated
		// in the second clock cycle.
		// The instruction format is: 1001 kkkk kkkk.
		////////////////////////////////////////////////////////////////////////////////////
      
		$display("pic10_controller: call");

		// Setup the datapath to load the PC register with 
		// the 8-bit call address from the 'alu_bus'.
		pc_mux_sel = 1;
		load_pc_reg = 1;
		
		// Also push PC onto the stack.
		load_stack = 1;
		
		// Wait for the Program Counter (PC) to clock in the new address to jump to.
		// The OLD PC value will also be loaded into the current stack location.
		// NOTE: We continue the controller execution on the NEXT falling edge of clk.
		// By doing this we have executed a complete cycle in this routine.
		@(negedge clk);

		// Deassert the 'load' control signals asserted in the first phase of this instruction.
		load_pc_reg = 0;
		load_stack = 0;

		// Increment the current stack location on the second clock cycle.
		inc_stack = 1;

		// NOTE: The main instruction execution loop will at the next rising edge of clock 
		// load the IR with the currently addressed instruction word. Since we above updated
		// the PC with the address we should jump to the IR will be loaded with the instruction
		// being jumped to. The execution will therefore continue at the jumped to instruction.

	end
	endtask		

	task goto();
	begin

   		////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'GOTO' instruction. This is a 2-cycle instruction because two 
		// separate tasks need to be done in sequence: 1) The PC is loaded with the address 
		// of the instruction being jumped to. 2) The IR is loaded with the instruction word
		// addressed by the (newly updated) updated PC. 
		// The instruction format is: 101k kkkk kkkk.
		////////////////////////////////////////////////////////////////////////////////////
      
		$display("pic10_controller: goto");

		// Setup the datapath to load the PC register with 
		// the 9-bit literal from the instruction word.
		pc_mux_sel = 2;
		load_pc_reg = 1;

		// Wait for the Program Counter (PC) to clock in the new address to jump to.
		// NOTE: We continue the controller execution on the NEXT falling edge of clk.
		// By doing this we have executed a complete cycle in this routine.
		@(negedge clk);

		// Deassert the 'load' control signals asserted in the first phase of this instruction.
		load_pc_reg = 0;
		
		// NOTE: The main instruction execution loop will at the next rising edge of clock 
		// load the IR with the currently addressed instruction word. Since we above updated
		// the PC with the address we should jump to the IR will be loaded with the instruction
		// being jumped to. The execution will therefore continue at the jumped to instruction.

	end
	endtask		
	
	task iorlw();
	begin
      
   		////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'IORLW k' instruction. This instruction ORs the 'W' register with
		// the literal 'k' which is encoded in the instruction word. The result is stored
		// back into the 'W' register.
		// The instruction format is: 1101 kkkk kkkk.
		////////////////////////////////////////////////////////////////////////////////////
      
		$display("pic10_controller: iorlw");
	      
		// The ALU outputs the result on 'alu_bus' so we only need to load 
		// the result of the AND operation back into the 'W' register.
		load_w_reg = 1;

	end
	endtask		
	
	task movlw();
	begin

		////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'MOVLW k' instruction. The constant ('literal') to load into the 
		// 'W' Register is encoded in the 8 least significant bits of the instruction. 
		////////////////////////////////////////////////////////////////////////////////////

		$display("pic10_controller: movlw");
		
		// The ALU knows (from the instruction word) to pass through the literal to the 
		// alu_bus so all we need to do to load the literal into the 'W' Register is to 
		// assert the load signal.
		load_w_reg = 1;

	end
	endtask
	
	task option();
	begin
      
		$display("pic10_controller: option - NOT IMPLEMENTED");

		// NOTE: We don't implement the option register because we currently do not support
		// any of the features controlled by the option register bits.

	end
	endtask			
	
	task retlw();
	begin
      
		////////////////////////////////////////////////////////////////////////////////////
		// Execute the "RETLW k" instruction. This instruction is implemented as a 3-cycle
		// instruction because it needs to perform the following tasks sequencially: 1) 
		// Decrement the stack pointer so the previously stored return address is output on 
		// 'pc_bus'. 2) Load PC with the previous saved return address on the stack. 3) Load
		// the 'IR' register with the instruction addressed by PC. Load the 'W' register
		// with the 8-bit literal from the RETLW instruction word. NOTE that the original
		// PIC CPU implements the RETLW instruction in 2-cycles, likely due to a more complex
		// stack implementation. It was decided to keep things simple so we settled for a
		// 3-cycle instruction. The much higher speed of a CPLD/FPGA implementation more than
		// compensates for the extra execution cycle.
		////////////////////////////////////////////////////////////////////////////////////
		
		$display("pic10_controller: retlw");

		// Cycle 1: Decrement the stack pointer so the previously stored return address 
		// is output on 'pc_bus'.
		dec_stack = 1;
		@(negedge clk);

		// Cycle 2: Load PC with the previous saved return address on the stack.
		dec_stack = 0;	// Deassert - was asserted in Cycle 1.
		pc_mux_sel = 0;
		load_pc_reg = 1;
		@(negedge clk);
		
		// Cycle 3: Load the 'IR' register with the instruction addressed by PC. 
		// NOTE: 'load_ir_reg' is asserted by the main always block.
		// Load the 'W' register with the 8-bit literal from the RETLW instruction word.
		load_pc_reg = 0; // Deassert - was asserted in Cycle 2.
		load_w_reg = 1;			
		
		// NOTE: The main always block will wait until posedge(clk).
		
	end
	endtask		
	
	task tris();
	begin
      
		$display("pic10_controller: tris");

		////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'TRIS f' instruction. The 'W' register is loaded into the TRIS
		// register indicated by 'f' (where f is 5, 6 or 7). No flags are updated.
		////////////////////////////////////////////////////////////////////////////////////

		// The ALU outputs the contents of the 'W' register onto 'alu_bus
		// so all we have to do is to assert the correct load_trisX_reg signal.
		// We get the TRIS register index from bits 2:0 of the instruction word 
		// on the 'ir_reg_bus'.
		case(ir_reg_bus[2:0])
			3'd5: load_tris5_reg = 1;
			3'd6: load_tris6_reg = 1;
			3'd7: load_tris7_reg = 1;
		endcase

	end
	endtask	

	task xorlw();
	begin
      
   		////////////////////////////////////////////////////////////////////////////////////
		// Execute the 'XORLW k' instruction. This instruction XORs the 'W' register with
		// the literal 'k' which is encoded in the instruction word. The result is stored
		// back into the 'W' register.
		// The instruction format is: 1111 kkkk kkkk.
		////////////////////////////////////////////////////////////////////////////////////
      
		$display("pic10_controller: xorlw");
	      
		// The ALU outputs the result on 'alu_bus' so we only need to load 
		// the result of the AND operation back into the 'W' register.
		load_w_reg = 1;

	end
	endtask	
	
endmodule
