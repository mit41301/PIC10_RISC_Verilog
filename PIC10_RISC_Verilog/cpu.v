/****************************************************************************
*
* By John.Gulbrandsen@SummitSoftConsulting.com, 1/17/2007.
*
* This file contains the top-level 'pic10_cpu' module for the PIC10 RISC Core.
* It contains the controller and datapath modules. 
* The program store is external.
*
****************************************************************************/
module pic10_cpu(

	///////////////////////////////////////////////////////////////
	/////////////////////////// Inouts ////////////////////////////
	///////////////////////////////////////////////////////////////
	
	inout [7:0] gpio5_pin_bus,	// Inout: Port 5 pins connected to the outside world.
	inout [7:0] gpio6_pin_bus,	// Inout: Port 6 pins connected to the outside world.
	inout [7:0] gpio7_pin_bus,	// Inout: Port 7 pins connected to the outside world.

	///////////////////////////////////////////////////////////////
	/////////////////////////// Inputs ////////////////////////////
	///////////////////////////////////////////////////////////////

	input reset,				// System reset.
	input clk					// System clock.

	);

	///////////////////////////////////////////////////////////////
	///////////////////Internal Wires and buses ///////////////////
	///////////////////////////////////////////////////////////////
	
	wire zero_result;			// Lets the controller know the state of Z-flag 
								// updates so it knows when to increment the PC
								// during execution of the DECFSZ, INCFSZ, BTFSZ
								// and BTFSS instructions.
	
	wire [4:0] reg_addr_bus;	// Used by the controller so it knows which 
								// load_XXX_reg signal to assert when writing to
								// a register via a direct or indirect address.
								
	wire [11:0] ir_reg_bus;		// Passes the current instruction code to the
								// controller so it knows how to direct the
								// datapath to execute the instruction.

	wire [8:0] pc_bus;			// The program memory address. Will result in
								// program instructions to be received on the
								// 'program_bus' bus.
	
	wire load_status_reg;		// Loads the alu_bus into STATUS register on next
								// posedge(clk).
								
	wire alu_mux_sel;			// Selects the 2nd operand to ALU to be 
								// sft_data_bus (0) or ram_data_bus(1).
								
	wire load_w_reg;			// Loads alu_bus into W register on next 
								// posedge(clk).
								
	wire load_fsr_reg;			// Loads alu_bus into FSR register on next 
								// posedge(clk). 
								
	wire load_ram_reg;			// Loads alu_bus into the addressed general purpose
								// register on next posedge(clk). 
								
	wire reg_addr_mux_sel;		// Input: Selects the register address to be used when reading or writing a
								// register. Can either be a direct address (0) or an indirect address (1).
								
	wire load_tris5_reg;		// Used to load the TRIS registers. A '1' in a
	wire load_tris6_reg;		// bit position makes the corresponding bit in 
	wire load_tris7_reg;		// the related GPIO register tri-stated (inputs).
								// The TRIS registers are reset as FFh (inputs).
								
	wire load_gpio5_reg;		// Used to load the GPIO registers. Output data
	wire load_gpio6_reg;		// is only driven out on the I/O pin if the
	wire load_gpio7_reg;		// corresponding TRIS bit is '0' (output).

	wire inc_stack;				// Increments or decrements the stack pointer on 
	wire dec_stack;				// next posedge(clk). 
									
	wire load_stack;			// Loads the current stack location with the
								// data on the pc_bus (used by the CALL instruction
								// to store the return address onto the stack).			
								
	wire load_pc_reg;			// loads PC register from pc_mux_bus (used by
								// the CALL instruction to load the jump address
								// or used to directly load a new value into PCL
								// when PCL is used as the target register).
								
	wire inc_pc;				// Increments the program counter (PC). Used
								// after instruction fetch as well as in 'skip'
								// instructions (DECFSZ, INCFSZ, BTFSC and BTFSS).
								
	wire [1:0] pc_mux_sel;		// Chooses whether the load data to PC should come
								// from the stack_bus (0), alu_bus (1) or 
								// ir_reg_bus (2).
								
	wire load_ir_reg;			// Loads IR with the contents of the program 
								// memory word currently being addressed by PC.
								
	wire [11:0] program_bus;	// Current instruction from the program memory at
								// the address output at pc_bus.

	wire skip_next_instruction;

	/////////////////////////////////////////////////////////////////
	// This is the datapath module.
	/////////////////////////////////////////////////////////////////
	
	pic10_datapath datapath(
	
		/////////////////////////// Inouts ////////////////////////////
	
		gpio5_pin_bus,			// Inout: Port 5 pins connected to the outside world.
		gpio6_pin_bus,			// Inout: Port 6 pins connected to the outside world.
		gpio7_pin_bus,			// Inout: Port 7 pins connected to the outside world.
	
	
		/////////////////////////// Outputs ///////////////////////////
	
		zero_result,			// Lets the controller know the state of Z-flag 
								// updates so it knows when to increment the PC
								// during execution of the DECFSZ, INCFSZ, BTFSZ
								// and BTFSS instructions.
	
		reg_addr_bus,			// Used by the controller so it knows which 
								// load_XXX_reg signal to assert when writing to
								// a register via a direct or indirect address.
								
		ir_reg_bus,				// Passes the current instruction code to the
								// controller so it knows how to direct the
								// datapath to execute the instruction.

		pc_bus,					// The program memory address. Will result in
								// program instructions to be received on the
								// 'program_bus' bus.

		/////////////////////////// Inputs ////////////////////////////

		load_status_reg,		// Loads the alu_bus into STATUS register on next
								// posedge(clk).
								
		alu_mux_sel,			// Selects the 2nd operand to ALU to be 
								// sft_data_bus (0) or ram_data_bus(1).
								
		load_w_reg,				// Loads alu_bus into W register on next 
								// posedge(clk).
									
		load_fsr_reg,			// Loads alu_bus into FSR register on next 
								// posedge(clk). 
									
		load_ram_reg,			// Loads alu_bus into the addressed general purpose
								// register on next posedge(clk). 
									
		reg_addr_mux_sel,		// Input: Selects the register address to be used when reading or writing a
								// register. Can either be a direct address (0) or an indirect address (1).
								
		load_tris5_reg,			// Used to load the TRIS registers. A '1' in a
		load_tris6_reg,			// bit position makes the corresponding bit in 
		load_tris7_reg,			// the related GPIO register tri-stated (inputs).
								// The TRIS registers are reset as FFh (inputs).
								
		load_gpio5_reg,			// Used to load the GPIO registers. Output data
		load_gpio6_reg,			// is only driven out on the I/O pin if the
		load_gpio7_reg,			// corresponding TRIS bit is '0' (output).

		inc_stack,				// Increments or decrements the stack pointer on 
		dec_stack,				// next posedge(clk). 
								
		load_stack,				// Loads the current stack location with the
								// data on the pc_bus (used by the CALL instruction
								// to store the return address onto the stack).			
								
		load_pc_reg,			// loads PC register from pc_mux_bus (used by
								// the CALL instruction to load the jump address
								// or used to directly load a new value into PCL
								// when PCL is used as the target register).
								
		inc_pc,					// Increments the program counter (PC). Used
								// after instruction fetch as well as in 'skip'
								// instructions (DECFSZ, INCFSZ, BTFSC and BTFSS).
								
		pc_mux_sel,				// Chooses whether the load data to PC should come
								// from the stack_bus (0), alu_bus (1) or 
								// ir_reg_bus (2).
								
		load_ir_reg,			// Loads IR with the contents of the program 
								// memory word currently being addressed by PC.
								
		program_bus,			// Current instruction from the program memory at
								// the address output at pc_bus.

		skip_next_instruction,	// If the previous instruction was DECFSZ, INCFSZ, 
								// BTFSC or BTFSS and the result was zero we should 
								// not execute the next instruction (i.e. we should 
								// treat the next instruction as a NOP).
								
		reset,					// System reset.
		clk);					// System clock.	
	
	/////////////////////////////////////////////////////////////////
	// This is the controller module.
	/////////////////////////////////////////////////////////////////

	pic10_controller controller(
		
		/////////////////////////// Outputs ///////////////////////////
		
		load_status_reg,		// Loads the alu_bus into STATUS register on next
								// posedge(clk).
									
		alu_mux_sel,			// Selects the 2nd operand to ALU to be 
								// sft_data_bus (0) or ram_data_bus(1).
									
		load_w_reg,				// Loads alu_bus into W register on next 
								// posedge(clk).
									
		load_fsr_reg,			// Loads alu_bus into FSR register on next 
								// posedge(clk). 
									
		load_ram_reg,			// Loads alu_bus into the addressed general purpose
								// register on next posedge(clk). 
									
		reg_addr_mux_sel,		// Selects the register address to be used when reading or writing a
								// register. Can either be a direct address (0) or an indirect address (1).
									
		load_tris5_reg,			// Used to load the TRIS registers. A '1' in a
		load_tris6_reg,			// bit position makes the corresponding bit in 
		load_tris7_reg,			// the related GPIO register tri-stated (inputs).
								// The TRIS registers are reset as FFh (inputs).
									
		load_gpio5_reg,			// Used to load the GPIO registers. Output data
		load_gpio6_reg,			// is only driven out on the I/O pin if the
		load_gpio7_reg,			// corresponding TRIS bit is '0' (output).

		inc_stack,				// Increments or decrements the stack pointer on 
		dec_stack,				// next posedge(clk). 
						
		load_stack,				// Loads the current stack location with the
								// data on the pc_bus (used by the CALL instruction
								// to store the return address onto the stack).			
						
		load_pc_reg,			// loads PC register from pc_mux_bus (used by
								// the CALL instruction to load the jump address
								// or used to directly load a new value into PCL
								// when PCL is used as the target register).
						
		inc_pc,					// Increments the program counter (PC). Used
								// after instruction fetch as well as in 'skip'
								// instructions (DECFSZ, INCFSZ, BTFSC and BTFSS).
						
		pc_mux_sel[1:0],		// Chooses whether the load data to PC should come
								// from the stack_bus (0), alu_bus (1) or 
								// ir_reg_bus (2).
						
		load_ir_reg,			// Loads IR with the contents of the program 
								// memory word currently being addressed by PC.

		skip_next_instruction,	// If the previous instruction was DECFSZ, INCFSZ, 
								// BTFSC or BTFSS and the result was zero we should 
								// not execute the next instruction (i.e. we should 
								// treat the next instruction as a NOP).

		/////////////////////////// Inputs ////////////////////////////

		zero_result,			// Lets the controller know the state of Z-flag 
								// updates so it knows when to increment the PC
								// during execution of the DECFSZ, INCFSZ, BTFSZ
								// and BTFSS instructions.
		
		reg_addr_bus[4:0],		// Used by the controller so it knows which 
								// load_XXX_reg signal to assert when writing to
								// a register via a direct or indirect address.
									
		ir_reg_bus[11:0],		// Passes the current instruction code to the
								// controller so it knows how to direct the
								// datapath to execute the instruction.
									
		reset,					// System reset.
		clk						// System clock.
									
		);

	/////////////////////////////////////////////////////////////////
	// This is the program store module.
	/////////////////////////////////////////////////////////////////
	
	pic10_program_store program_store(
		
		/////////////////////////// Outputs ///////////////////////////
		
		program_bus[11:0],		// Current instruction from the program memory at
								// the address input at pc_bus.

		/////////////////////////// Inputs ////////////////////////////

		pc_bus[8:0],			// The program memory address. Will result in
								// program instructions to be received on the
								// 'program_bus' bus.
		);


endmodule
