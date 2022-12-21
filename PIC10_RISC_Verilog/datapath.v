/****************************************************************************
*
* By John.Gulbrandsen@SummitSoftConsulting.com, 1/17/2007.
*
* This file contains all the modules used by the datapath in the PIC10 RISC 
* CPU core.
*
* The datapath is software compatible with the PIC10F100 microcontroller
* series from Microchip. Note that the datapath is not cycle-accurate 
* because this implementation is not pipelined in order to keep the imple-
* mentation small. The higher clock frequency of a CPLD/FPGA will more than
* compensate for this.
*
* Register differences: TMR0 is not implemented. OSCCAL and CMCON0 registers
* have been replaced by two more GPIO registers. PC is 9-bits (no banking 
* support in the PIC10F100-series so there's no need to implement more bits).
*
****************************************************************************/

// These constants determine the signals on the status_bus (output from ALU)
// as well as on the status_reg_bus (output from STATUS register).
`define STATUS_C	0
`define STATUS_DC	1
`define STATUS_Z	2	

// This constant defines the number of stack positions.
`define STACK_DEPTH 2

module pic10_datapath(
	
	///////////////////////////////////////////////////////////////
	/////////////////////////// Inouts ////////////////////////////
	///////////////////////////////////////////////////////////////
	
	inout [7:0] gpio5_pin_bus,	// Inout: Port 5 pins connected to the outside world.
	inout [7:0] gpio6_pin_bus,	// Inout: Port 6 pins connected to the outside world.
	inout [7:0] gpio7_pin_bus,	// Inout: Port 7 pins connected to the outside world.
	
	
	///////////////////////////////////////////////////////////////
	/////////////////////////// Outputs ///////////////////////////
	///////////////////////////////////////////////////////////////
	
	output zero_result,			// Lets the controller know the state of Z-flag 
								// updates so it knows when to increment the PC
								// during execution of the DECFSZ, INCFSZ, BTFSZ
								// and BTFSS instructions.
	
	output [4:0] reg_addr_bus,	// Used by the controller so it knows which 
								// load_XXX_reg signal to assert when writing to
								// a register via a direct or indirect address.
								
	output [11:0] ir_reg_bus,	// Passes the current instruction code to the
								// controller so it knows how to direct the
								// datapath to execute the instruction.

	output [8:0] pc_bus,		// The program memory address. Will result in
								// program instructions to be received on the
								// 'program_bus' bus.

	///////////////////////////////////////////////////////////////
	/////////////////////////// Inputs ////////////////////////////
	///////////////////////////////////////////////////////////////

	input load_status_reg,		// Loads the alu_bus into STATUS register on next
								// posedge(clk).
								
	input alu_mux_sel,			// Selects the 2nd operand to ALU to be 
								// sft_data_bus (0) or ram_data_bus(1).
								
	input load_w_reg,			// Loads alu_bus into W register on next 
								// posedge(clk).
								
	input load_fsr_reg,			// Loads alu_bus into FSR register on next 
								// posedge(clk). 
								
	input load_ram_reg,			// Loads alu_bus into the addressed general purpose
								// register on next posedge(clk). 
								
	input reg_addr_mux_sel,		// Selects the register address to be used when reading or writing a
								// register. Can either be a direct address (0) or an indirect address (1).
								
	input load_tris5_reg,		// Used to load the TRIS registers. A '1' in a
	input load_tris6_reg,		// bit position makes the corresponding bit in 
	input load_tris7_reg,		// the related GPIO register tri-stated (inputs).
								// The TRIS registers are reset as FFh (inputs).
								
	input load_gpio5_reg,		// Used to load the GPIO registers. Output data
	input load_gpio6_reg,		// is only driven out on the I/O pin if the
	input load_gpio7_reg,		// corresponding TRIS bit is '0' (output).

	input inc_stack,			// Increments or decrements the stack pointer on 
	input dec_stack,			// next posedge(clk). 
								
	input load_stack,			// Loads the current stack location with the
								// data on the pc_bus (used by the CALL instruction
								// to store the return address onto the stack).			
								
	input load_pc_reg,			// loads PC register from pc_mux_bus (used by
								// the CALL instruction to load the jump address
								// or used to directly load a new value into PCL
								// when PCL is used as the target register).
								
	input inc_pc,				// Increments the program counter (PC). Used
								// after instruction fetch as well as in 'skip'
								// instructions (DECFSZ, INCFSZ, BTFSC and BTFSS).
								
	input [1:0] pc_mux_sel,		// Chooses whether the load data to PC should come
								// from the stack_bus (0), alu_bus (1) or 
								// ir_reg_bus (2).
								
	input load_ir_reg,			// Loads IR with the contents of the program 
								// memory word currently being addressed by PC.
								
	input [11:0] program_bus,	// Current instruction from the program memory at
								// the address output at pc_bus.

	input skip_next_instruction,// If the previous instruction was DECFSZ, INCFSZ, 
								// BTFSC or BTFSS and the result was zero we should 
								// not execute the next instruction (i.e. we should 
								// treat the next instruction as a NOP).
								
	input reset,				// System reset.
	input clk					// System clock.
								
	);
	
	///////////////////////////////////////////////////////////////
	///////////////////Internal Wires and buses ///////////////////
	///////////////////////////////////////////////////////////////

	wire [7:0] status_reg_bus;		// The output from the STATUS register.
	wire [7:0] alu_bus;				// The output result from the ALU.
	wire [2:0] alu_status_bus;		// The status output from the ALU.
	wire [7:0] sfr_data_bus;		// The output from the SFR Data Mux.
	wire [7:0] fsr_reg_bus;			// The output from the FSR register.
	wire [7:0] triport5_bus;		// The value read from GPIO port 5.
	wire [7:0] triport6_bus;		// The value read from GPIO port 6.
	wire [7:0] triport7_bus;		// The value read from GPIO port 7.
	wire [7:0] ram_data_bus;		// The output from the General Purpose RAM.
	wire [11:0] program_mux_bus;	// The output from the Program Mux.
	
	///////////////////////////////////////////////////////////////
	/////////////////////// Implementation ////////////////////////
	///////////////////////////////////////////////////////////////

	// The 'zero_result' output is tied to the Z bit in the ALU's status_bus.
	assign zero_result = alu_status_bus[`STATUS_Z];

	// This is the status register. It is loaded from alu_bus.
	// The ALU can also individually set/clear the Z, C and DC bits.
		
	pic10_status_reg status_reg(
		status_reg_bus[7:0],	// Output: The current status is always driven.
		carry_bit,				// Output: The carry bit from the STATUS register.
		alu_bus[7:0],			// Input: Data to be loaded when asserting 'load_status_reg'.
		load_status_reg,		// Input: Loads alu_bus into register at next posedge(clk).
		alu_status_bus[2:0],	// Input: Z, C and DC status bits. Driven by ALU.
		load_z,					// Input: Changes Z bit accordingly to alu_status_bus.z.
		load_c,					// Input: Changes C bit accordingly to alu_status_bus.c.
		load_dc,				// Input: Changes DC bit accordingly to alu_status_bus.dc.
		reset,					// System reset.
		clk);					// System clock.
		
	// This is the SFR Data Mux. It is used to select one of the SFR registers as
	// ALUs 2nd operand. Note that there is yet another mux inbetween the SFR Data
	// Mux and the ALU (the ALU Mux) which selects between the SFR Data and General-
	// Purpose RAM data	outputs.
	
	reg [7:0] null_bus = 8'b0;	// Used to drive 00h on the not implemented INDF and TMR0 buses.

	pic10_sfr_data_mux sfr_data_mux(
		sfr_data_bus,			// Output: Operand data from the special function registers 0..7.
		reg_addr_bus[2:0],		// Input: This is the currently selected register address (either
								// a direct address	or an indirect address selected by the Register
								// Address Mux). The SFR Data Mux only uses the three lowest bits.	
		null_bus[7:0],			// Input: INDF returns 00h when read.
		null_bus[7:0],			// Input: TMR0 returns 00h when read (not implemented).
		pc_bus[7:0],			// Input: PCL (the lowest 8 bits of the 9-bit PC).
		status_reg_bus[7:0],	// Input: The STATUS register's output.
		fsr_reg_bus[7:0],		// Input: The FSR register's output.
		triport5_bus[7:0],		// Input: The value read from GPIO port 5. Replaces OSCCAL register.
		triport6_bus[7:0],		// Input: The value read from GPIO port 6.
		triport7_bus[7:0]);		// Input: The value read from GPIO port 7. Replaces CMCON0 register.
		
	// This is the ALU datapath. It contains the ALU, the ALU MUX that selects ALU operand 
	// data from either the sfr_data_bus or ram_data_bus. In addition this module contains
	// the W register. Packaging these modules into the ALU Datapath module makes the
	// ALU Core easy to test separately.
	
	pic10_alu_datapath alu_datapath(
		alu_bus[7:0],			// Output: The output from the ALU. This can either be
								// an unmodified, passed-through operand (W or register),
								// a literal (constant) from the instruction word or
								// the result from an arithmetic or logical operation
								// or some combination of the W reg, registers and a 
								// literal.
								
		alu_status_bus[2:0],	// Output: Z, C and DC status bits. Driven by ALU when
								// an ALU operation affects any of the flags. The load_XXX
								// outputs will determine which of the alu_status_bus signals
								// are valid (and should be loaded into the status register).
								
		load_z,					// Output: The alu_status_bus.z bit is valid.
		load_c,					// Output: The alu_status_bus.c bit is valid.
		load_dc,				// Output: The alu_status_bus.dc bit is valid.
		
		sfr_data_bus[7:0],		// Input: Operand data from the special function registers 0..7.
		ram_data_bus[7:0],		// Input: Operand data from the general-purpose RAM registers.
		ir_reg_bus[11:0],		// Input: This is the current instruction loaded into the 
								// instruction register. The ALU uses the 'literal' operand
								// fields in the instruction word.
								
		alu_mux_sel,			// Input: Selects the ALU's 2nd operand to be either the data on the 
								// sfr_data_bus (0) or the data on the ram_data_bus (1).
		
		load_w_reg,				// Input: Loads the data on the alu_bus into the W register.
		carry_bit,				// Input: The carry bit from the STATUS register.
		
		reset,					// System reset.
		clk);					// System clock.
		
	// This 12-bit multiplexer allows us to replace the next instruction with a NOP.
	// It is used by the controller to skip over the next actual instruction when
	// one of the 'skip'-instructions results in a skip of the next instruction.
	
	reg [11:0] nop_bus = 12'h000;
	
	pic10_program_mux program_mux(
		program_mux_bus,		// Output: This is the bus that drives the ALUs 2nd operand.
		program_bus,			// Input: Operand data from the program_bus (currently addressed instruction).
		nop_bus,				// Input: A NOP instruction Op-code (000h).
		skip_next_instruction);	// Input: When asserted, passes through the NOP Op-code.
		
	// This is the SFR datapath. It contains the Instruction Register (IR), the FSR 
	// register, the Register Address Mux as well as the General-purpose RAM registers.
	// Packaging these modules into the 'SFR Datapath' module allows us to more easily
	// test the functionality of the direct and indirect addressing modes to access RAM. 
	
	pic10_sfr_datapath sfr_datapath(
		fsr_reg_bus[7:0],		// Output: Contains the indirect register address in the FSR register.
								// Used by the controller to determine which load_XXX_reg signal to 
								// assert when writing to a SFR or general-purpose RAM register.
								
		ir_reg_bus[11:0],		// Output: Contains the instruction word in the Instruction Register.
								// Used by the controller to determine which combinational signals
								// to assert to execute the instruction. Also used by the ALU which
								// needs to know the 'literal' constant operands (which some
								// instructions have encoded as part of the instruction).
								
		ram_data_bus[7:0],		// Output: The output from the currently addressed general-purpose
								// RAM register. The address can either be a direct or indirect 
								// address (determined by the Register Address Mux).
								
		reg_addr_bus[4:0],		// Output: This is the currently selected register address (either
								// a direct address	or an indirect address selected by the Register
								// Address Mux). Note that we only pass in a part-select of the bus.

		program_mux_bus[11:0],	// Input: Carries the program word to load into the Instruction Register
								// (IR) when 'load_ir_reg' is asserted.
								
		alu_bus[7:0],			// Input: The output from the ALU. Loaded into a register when one of the
								// load_XXX_reg signals are asserted.					
								
		load_ir_reg,			// Input: Loads alu_bus into IR on the next posedge(clk).
		load_fsr_reg,			// Input: Loads alu_bus into FSR on the next posedge(clk).
		load_ram_reg,			// Input: Loads alu_bus into a General-purpose RAM register with the address 
								// driven on reg_addr_bus. The data is clocked in on the next 
								// posedge(clk).
								
		reg_addr_mux_sel,		// Input: Selects the register address to be used when reading or writing a
								// register. Can either be a direct address (0) or an indirect address (1).
								
		reset,					// System reset.
		clk);					// System clock.
							
	// This is the 'PC Datapath'. It contains the Program Counter (PC), the stack and the PC Mux
	// which allows the PC to be loaded with data from the stack, alu_bus or ir_reg_bus buses. By
	// putting these modules into the 'PC Datapath' module we can test the PC push and pop operations.
	
	pic10_pc_datapath pc_datapath(
		pc_bus[8:0],			// Output: Drives the current value in the Program Counter.
		alu_bus[7:0],			// Input: Used to load register values into PC.
		ir_reg_bus[8:0],		// Input: Used to load literal values (constants in instructions) into PC.
		pc_mux_sel[1:0],		// Input: Allows the PC to be loaded with data from the internal stack bus (0),
								// alu_bus (1) or ir_reg_bus (2).
		load_pc_reg,			// Input: Loads the PC from the input bus currently selected via pc_mux_sel.
		inc_pc,					// Input: Increments the program counter.
		inc_stack,				// Input: Increments the stack pointer.
		dec_stack,				// Input: Decrements the stack pointer.
		load_stack,				// Input: Loads the stack from the current PC value.
		reset,					// System reset.
		clk);					// System clock.
		
	// This is the tri-state register number 5. It combines the GPIO and TRIS register 5.
	pic10_tri_state_port tri_state_port5(
		gpio5_pin_bus[7:0],		// Inout: This is the port bus connected to the outside world.
		triport5_bus[7:0],		// Output: The value read from GPIO port 5. Replaces OSCCAL register.
		alu_bus[7:0],			// Input: Used to load register values into the TRIS or GPIO port (the
								// register to load is determined by asserting either the load_tris5_reg
								// or the load_gpio5_reg signal).
		load_tris5_reg,			// Used to load the TRIS register.
		load_gpio5_reg,			// Used to load the GPIO register.
		reset,					// Input: System reset.
		clk);					// Input: System clock.
		
		
	// This is the tri-state register number 6. It combines the GPIO and TRIS register 6.
	pic10_tri_state_port tri_state_port6(
		gpio6_pin_bus[7:0],		// Inout: This is the port bus connected to the outside world.
		triport6_bus[7:0],		// Output: The value read from GPIO port 6.
		alu_bus[7:0],			// Input: Used to load register values into the TRIS or GPIO port (the
								// register to load is determined by asserting either the load_tris6_reg
								// or the load_gpio6_reg signal).
		load_tris6_reg,			// Used to load the TRIS register.
		load_gpio6_reg,			// Used to load the GPIO register.
		reset,					// Input: System reset.
		clk);					// Input: System clock.
		
		
	// This is the tri-state register number 7. It combines the GPIO and TRIS register 7.
	pic10_tri_state_port tri_state_port7(
		gpio7_pin_bus[7:0],		// Inout: This is the port bus connected to the outside world.
		triport7_bus[7:0],		// Output: The value read from GPIO port 7. Replaces CMCON0 register.
		alu_bus[7:0],			// Input: Used to load register values into the TRIS or GPIO port (the
								// register to load is determined by asserting either the load_tris7_reg
								// or the load_gpio7_reg signal).
		load_tris7_reg,			// Used to load the TRIS register.
		load_gpio7_reg,			// Used to load the GPIO register.
		reset,					// Input: System reset.
		clk);					// Input: System clock.
	
endmodule
			
		
/////////////////////////////////////////////////////////////////
// module pic10_alu_datapath
// Tested OK 2/14/07 with testbench 'test_pic10_alu_datapath'.
/////////////////////////////////////////////////////////////////		
module pic10_alu_datapath(
		output [7:0] alu_bus,	// Output: The output from the ALU. This can either be
								// an unmodified, passed-through operand (W or register),
								// a literal (constant) from the instruction word or
								// the result from an arithmetic or logical operation
								// or some combination of the W reg, registers and a 
								// literal.
								
		output [2:0] alu_status_bus,// Output: Z, C and DC status bits. Driven by ALU when
								// an ALU operation affects any of the flags. The load_XXX
								// outputs will determine which of the alu_status_bus signals
								// are valid (and should be loaded into the status register).
		output load_z,			// Output: The alu_status_bus.z bit is valid.
		output load_c,			// Output: The alu_status_bus.c bit is valid.
		output load_dc,			// Output: The alu_status_bus.dc bit is valid.
		input [7:0]sfr_data_bus,// Input: Operand data from the special function registers 0..7.
		input [7:0]ram_data_bus,// Input: Operand data from the general-purpose RAM registers.
		input [11:0] ir_reg_bus,// Input: This is the current instruction loaded into the 
								// instruction register. The ALU uses the 'literal' operand
								// fields in the instruction word.
		input alu_mux_sel,		// Input: Selects the ALU's 2nd operand to be either the data on the 
								// sfr_data_bus (0) or the data on the ram_data_bus (1).
		input load_w_reg,		// Input: Loads the data on the alu_bus into the W register.
		input carry_bit,		// Input: The carry bit from the STATUS register.
		input reset,			// System reset.
		input clk);				// System clock.
		

	///////////////////////////////////////////////////////////////
	///////////////// Internal Wires and Buses ////////////////////
	///////////////////////////////////////////////////////////////

   wire [7:0] alu_mux_bus;
   wire [7:0] w_reg_bus;

	///////////////////////////////////////////////////////////////
	/////////////////////// Implementation ////////////////////////
	///////////////////////////////////////////////////////////////

	// This is the ALU Mux that selects the 2nd ALU operand to be either 
	// the SFR or the currently addressed general-purpose RAM register.
	
	pic10_alu_mux alu_mux(
		alu_mux_bus[7:0],		// Output: This is the bus that drives the ALUs 2nd operand.
		sfr_data_bus[7:0],		// Input: Operand data from the special function registers 0..7.
		ram_data_bus[7:0],		// Input: Operand data from the general-purpose RAM registers.
		alu_mux_sel);			// Input: Selects the ALU's 2nd operand to be either the data on the 
								// sfr_data_bus (0) or the data on the ram_data_bus (1).
								
	// This is the ALU.
	pic10_alu alu(
		alu_bus[7:0],			// Output: The output from the ALU. This can either be
								// an unmodified, passed-through operand (W or register),
								// a literal (constant) from the instruction word or
								// the result from an arithmetic or logical operation
								// or some combination of the W reg, registers and a 
								// literal.
		load_z,					// Output: The ALU will set this signal when the Z bit is affected 
								// by an ALU operation. The Z bit itself is part of the alu_status_bus.
		load_c,					// Output: The ALU will set this signal when the C bit is affected 
								// by an ALU operation. The C bit itself is part of the alu_status_bus.
		load_dc,				// Output: The ALU will set this signal when the DC bit is affected 
								// by an ALU operation. The DC bit itself is part of the alu_status_bus.
		alu_status_bus[2:0],	// Output: The ALU drives the Z, C and/or DC status bits if affected
								// by the ALU operation input on the ir_reg_bus input.
		w_reg_bus[7:0],			// Input: The output from the W register. Used as 1st ALU operand.
		alu_mux_bus[7:0],		// Input: This is the bus that drives the ALUs 2nd operand.
		ir_reg_bus[11:0],		// Input: The ALU retrieves the opcode and literal operands from this bus.
		carry_bit);				// Input: The carry bit from the STATUS register.
		
	// This is the W register.
	pic10_w_reg w_reg(
		w_reg_bus[7:0],			// Output: The output from the W register. Used as 1st ALU operand.
		alu_bus[7:0],			// Input: The output from the ALU. This can either be
								// an unmodified, passed-through operand (W or register),
								// a literal (constant) from the instruction word or
								// the result from an arithmetic or logical operation
								// or some combination of the W reg, registers and a 
								// literal.
		load_w_reg,				// Input: Loads the data on the alu_bus into the W register.
		reset,					// System reset.
		clk);					// System clock.
									
endmodule


/////////////////////////////////////////////////////////////////
// module pic10_sfr_datapath
// Tested OK 1/22/07 with testbench 'test_pic10_sfr_datapath'.
/////////////////////////////////////////////////////////////////		
module pic10_sfr_datapath(
	output [7:0] fsr_reg_bus,	// Output: Contains the indirect register address in the FSR register.
								// Used by the controller to determine which load_XXX_reg signal to 
								// assert when writing to a SFR or general-purpose RAM register.
							
	output [11:0] ir_reg_bus,	// Output: Contains the instruction word in the Instruction Register.
								// Used by the controller to determine which combinational signals
								// to assert to execute the instruction. Also used by the ALU which
								// needs to know the 'literal' constant operands (which some
								// instructions have encoded as part of the instruction).
								
	output [7:0] ram_data_bus,	// Output: The output from the currently addressed general-purpose
								// RAM register. The address can either be a direct or indirect 
								// address (determined by the Register Address Mux).
				
	output [4:0] reg_addr_bus,	// Output: This is the currently selected register address (either
								// a direct address	or an indirect address selected by the Register
								// Address Mux). Used as select signal for the SFR Data Mux. The SFR Data
								// Mux selects the SFR data to be used as the 2nd ALU operand.
								// NOTE that the 2nd ALU operand is further qualified by the ALU
								// Mux which selects between the output from the SFR Data Mux and 
								// the output from the General Purpose RAM Registers.
							
	input [11:0] program_mux_bus,// Input: Carries the program word to load into the Instruction Register
								// (IR) when 'load_ir_reg' is asserted.
							
	input [7:0] alu_bus,		// Input: The output from the ALU. Loaded into a register when one of the
								// load_XXX_reg signals are asserted.					
							
	input load_ir_reg,			// Input: Loads program_mux_bus into IR on the next posedge(clk).
	input load_fsr_reg,			// Input: Loads alu_bus into FSR on the next posedge(clk).
	input load_ram_reg,			// Input: Loads alu_bus into a General-purpose RAM register with the address 
								// driven on reg_addr_bus. The data is clocked in on  next posedge(clk).
							
	input reg_addr_mux_sel,		// Input: Selects the register address to be used when reading or writing a
								// register. Can either be a direct address (0) or an indirect address (1).
								
	input reset,				// System reset.
	input clk);					// System clock.

	///////////////////////////////////////////////////////////////
	/////////////////////// Implementation ////////////////////////
	///////////////////////////////////////////////////////////////

	// This is the Instruction Register (IR). It holds the 
	// current instruction while it is being executed.
	
	pic10_ir ir(
		ir_reg_bus[11:0],		// Output: Contains the instruction word in the Instruction Register.
		program_mux_bus[11:0],	// Input: Carries the program word to load into the Instruction Register
								// (IR) when 'load_ir_reg' is asserted.					
		load_ir_reg,			// Input: Loads program_mux_bus into IR on the next posedge(clk).
		reset,					// System reset.
		clk);					// System clock.
	
	// This is the FSR register. The value loaded into the FSR register is used as the 
	// indirect register address when an instruction reads a register via the INDF register.
	
	pic10_fsr fsr(
		fsr_reg_bus[7:0],		// Output: Contains the indirect register address in the FSR register.
		alu_bus[7:0],			// Input: The output from the ALU. Loaded into the FSR register when 
								// the load_fsr_reg signals is asserted at the next posedge(clk).
		load_fsr_reg,			// Input: Loads alu_bus into FSR on the next posedge(clk).
		reset,					// System reset.
		clk);					// System clock.
	
	// This is the Register Address Mux. It is used to select between 
	// a direct or indirect address when addressing a register.
	pic10_register_address_mux register_address_mux(
		reg_addr_bus[4:0],		// Output: This is the currently selected register address (either
								// a direct address	or an indirect address selected by the Register
								// Address Mux).
		ir_reg_bus[4:0],		// Input: This part-select of the complete instruction word contains 
								// a 'literal' constant specifying the direct address used to access 
								// a register (note: only for selected data-moving instructions).
		fsr_reg_bus[4:0],		// Input: Contains the indirect register address in the FSR register.
								// The indirect address is only used when accessing a register via
								// the FSR register.
		reg_addr_mux_sel);		// Input: Selects the register address to be used when reading or writing a
								// register. Can either be a direct address (0) or an indirect address (1).
			
	// This is the General Purpose Registers (RAM).
	pic10_ram_registers ram_registers(
		ram_data_bus[7:0],		// Output: Always outputs the data in the addressed register.
		alu_bus[7:0],			// Input: The output from the ALU. Loaded into the register when the
								// load_ram_reg signal is asserted. 
		load_ram_reg,			// Input: Loads alu_bus into a General-purpose RAM register with the address 
								// driven on reg_addr_bus. The data is clocked in on the next 
								// posedge(clk).
		reg_addr_bus[4:0],		// Input: This is the currently selected register address (either
								// a direct address	or an indirect address selected by the Register
								// Address Mux). Note: The first RAM register is at address 8 because
								// the Special Function Registers have addresses 0..7.
		reset,					// System reset.
		clk);					// System clock.
	
endmodule

/////////////////////////////////////////////////////////////////
// module pic10_pc_datapath
// Tested OK 1/27/07 with testbench 'test_pic10_pc_datapath'.
/////////////////////////////////////////////////////////////////		
module pic10_pc_datapath(
	output [8:0] pc_bus,		// Output: Drives the current value in the Program Counter.
	input [7:0] alu_bus,		// Input: Used to load register values into PC.
	input [8:0] ir_reg_bus,		// Input: Used to load literal values (constants in instructions) into PC.
	input [1:0] pc_mux_sel,		// Input: Allows the PC to be loaded with data from the internal stack bus (0),
								// alu_bus (1) or ir_reg_bus (2).
	input load_pc_reg,			// Input: Loads the PC from the input bus currently selected via pc_mux_sel.
	input inc_pc,				// Input: Increments the program counter.
	input inc_stack,			// Input: Increments the stack pointer.
	input dec_stack,			// Input: Decrements the stack pointer.
	input load_stack,			// Input: Loads the stack from the current PC value.
	input reset,				// System reset.
	input clk);					// System clock.
	
	///////////////////////////////////////////////////////////////
	/////////////////////// Implementation ////////////////////////
	///////////////////////////////////////////////////////////////
	
	// This is the n-level stack. The default stack depth is 2 but it
	// can be expended by modifying the 'pic10_stack' module.
	wire [8:0] stack_bus;
	
	pic10_stack stack(
		stack_bus[8:0],			// Output: Current stack location data.
		pc_bus[8:0],			// Input: Data to be loaded when asserting load_stack.
		load_stack,				// Input: Loads the stack from the current PC value.
		inc_stack,				// Input: Increments the current stack location.
		dec_stack,				// Input: Decrements the current stack location.
		reset,					// Input: System reset.
		clk);					// Input: System clock.

	// This is the PC Mux. It allows the PC to be loaded with data from
	// either the stack_bus (0), alu_bus (1) or the ir_reg_bus (2).
	wire [8:0] pc_mux_bus;
	
	pic10_pc_mux pc_mux(
		pc_mux_bus[8:0],		// Output: The PC will be loaded from this bus.
		stack_bus[8:0],			// Input: Used to load stacked values into PC.
		alu_bus[7:0],			// Input: Used to load register values into PC.
		ir_reg_bus[8:0],		// Input: Used to load literal values (constants in instructions) into PC.
		pc_mux_sel[1:0]);		// Input: allows the PC to be loaded with data from the internal stack bus (0),
								// alu_bus (1) or ir_reg_bus (2).
		
	// This is the program counter.
	pic10_pc pc(
		pc_bus[8:0],			// Output: Drives the current value in the Program Counter.
		pc_mux_bus[8:0],		// Input: Data to be loaded into PC when 'load_pc_reg' is asserted. 	
		load_pc_reg,			// Input: Loads the PC from the input bus currently selected via pc_mux_sel.
		inc_pc,					// Input: Increments the program counter.
		reset,					// Input: System reset.
		clk);					// Input: System clock.
		
endmodule

/////////////////////////////////////////////////////////////////
// module pic10_stack
// Tested OK 1/27/07 with testbench 'test_pic10_stack'.
/////////////////////////////////////////////////////////////////	
module pic10_stack(
	output [8:0] stack_bus,		// Output: Current stack location data.
	input [8:0] pc_bus,			// Input: Data to be loaded when asserting load_stack.
	input load_stack,			// Input: Loads the stack from the current PC value.
	input inc_stack,			// Input: Increments the current stack location.
	input dec_stack,			// Input: Decrements the current stack location.
	input reset,				// Input: System reset.
	input clk);					// Input: System clock.

	///////////////////////////////////////////////////////////////
	/////////////////////// Implementation ////////////////////////
	///////////////////////////////////////////////////////////////

	// This is the stack storage.
	reg [8:0] stack_regs[`STACK_DEPTH];
	
	// This is the internal stack pointer. It is updated 
	// by the inc_stack and dec_stack signals.
	reg sp;
		
	always @(posedge clk)
	begin
	
		if(reset)
			sp <= 0;
		else if(load_stack)
			stack_regs[sp] <= pc_bus;	
		else if(inc_stack)
			sp <= sp + 1;
		else if(dec_stack)
			sp <= sp - 1;
	end
	
   assign stack_bus = stack_regs[sp];

endmodule

/////////////////////////////////////////////////////////////////
// module pic10_pc_mux
// Tested OK 1/27/07 with testbench 'test_pic10_pc_mux'.
/////////////////////////////////////////////////////////////////	
module pic10_pc_mux(
	output reg [8:0] pc_mux_bus,// Output: The PC will be loaded from this bus.
	input [8:0] stack_bus,		// Input: Used to load stacked values into PC.
	input [7:0] alu_bus,		// Input: Used to load register values into PC.
	input [8:0] ir_reg_bus,		// Input: Used to load literal values (constants in instructions) into PC.
	input [1:0] pc_mux_sel);	// Input: allows the PC to be loaded with data from the internal stack bus (0),
								// alu_bus (1) or ir_reg_bus (2).

	///////////////////////////////////////////////////////////////
	/////////////////////// Implementation ////////////////////////
	///////////////////////////////////////////////////////////////
	
	reg [8:0] null_bus = 0;
	
	always @(stack_bus, alu_bus, ir_reg_bus, pc_mux_sel) 
	begin
		
		case(pc_mux_sel)
			2'd0:		pc_mux_bus = stack_bus;
			2'd1:		pc_mux_bus = {1'b0, alu_bus};
			2'd2:		pc_mux_bus = ir_reg_bus;
			default:	pc_mux_bus = null_bus;
	endcase
		
	end	
				
endmodule

/////////////////////////////////////////////////////////////////
// module pic10_pc
// Tested OK 1/27/07 with testbench 'test_pic10_pc'.
/////////////////////////////////////////////////////////////////		
module pic10_pc(
	output reg [8:0] pc_bus,	// Output: Drives the current value in the Program Counter.
	input [8:0] pc_mux_bus,		// Input: Data to be loaded into PC when 'load_pc_reg' is asserted. 	
	input load_pc_reg,			// Input: Loads the PC from the input bus currently selected via pc_mux_sel.
	input inc_pc,				// Input: Increments the program counter.
	input reset,				// Input: System reset.
	input clk);					// Input: System clock.

	///////////////////////////////////////////////////////////////
	/////////////////////// Implementation ////////////////////////
	///////////////////////////////////////////////////////////////
	
	always @(posedge clk)
	begin
	
		// 'reset' takes precedence over other inputs.
		if(reset) 
			pc_bus <= 0;
		else if(load_pc_reg)
			pc_bus <= pc_mux_bus;
		else if(inc_pc)
			pc_bus <= pc_bus + 1;
	end	
		
endmodule

/////////////////////////////////////////////////////////////////
// module pic10_ir
// Tested OK 1/27/07 with testbench 'test_pic10_ir'.
/////////////////////////////////////////////////////////////////
module pic10_ir(
	output reg [11:0] ir_reg_bus,	// Output: Contains the instruction word in the Instruction Register.
	input [11:0] program_mux_bus,	// Input: Carries the program word to load into the Instruction Register
									// (IR) when 'load_ir_reg' is asserted.					
	input load_ir_reg,				// Input: Loads program_mux_bus into IR on the next posedge(clk).
	input reset,					// Input: System reset.
	input clk);						// Input: System clock.

	///////////////////////////////////////////////////////////////
	/////////////////////// Implementation ////////////////////////
	///////////////////////////////////////////////////////////////
	
	always @(posedge clk)
	begin
	
		// 'reset' takes precedence over other inputs.
		if(reset) 
			ir_reg_bus <= 0;
		else if(load_ir_reg)
			ir_reg_bus <= program_mux_bus;
	end	
	
endmodule

/////////////////////////////////////////////////////////////////
// module pic10_fsr
// Tested OK 1/27/07 with testbench 'test_pic10_fsr'.
/////////////////////////////////////////////////////////////////
module pic10_fsr(
	output reg [7:0] fsr_reg_bus,// Output: Contains the indirect register address in the FSR register.
	input [7:0] alu_bus,		// Input: The output from the ALU. Loaded into the FSR register when 
								// the load_fsr_reg signals is asserted at the next posedge(clk).
	input load_fsr_reg,			// Input: Loads alu_bus into FSR on the next posedge(clk).
	input reset,				// Input: System reset.
	input clk);					// Input: System clock.

	///////////////////////////////////////////////////////////////
	/////////////////////// Implementation ////////////////////////
	///////////////////////////////////////////////////////////////
	
	always @(posedge clk)
	begin
	
		// 'reset' takes precedence over other inputs.
		if(reset) 
			fsr_reg_bus <= 0;
		else if(load_fsr_reg)
			fsr_reg_bus <= alu_bus;
	end
	
endmodule

/////////////////////////////////////////////////////////////////
// module pic10_status_reg
// Tested OK 1/22/07 with testbench 'test_pic10_status_reg'.
/////////////////////////////////////////////////////////////////
module pic10_status_reg(
	output reg [7:0] status_reg_bus,// Output: The current status is always driven.
	output carry_bit,				// Output: The carry bit from the STATUS register.
	input [7:0] alu_bus,			// Input: Data to be loaded when asserting 'load_status_reg'.
	input load_status_reg,			// Input: Loads alu_bus into register at next posedge(clk).
	input [2:0] alu_status_bus,		// Input: Z, C and DC status bits. Driven by ALU.
	input load_z,					// Input: Changes Z bit accordingly to alu_status_bus.z.
	input load_c,					// Input: Changes C bit accordingly to alu_status_bus.c.
	input load_dc,					// Input: Changes DC bit accordingly to alu_status_bus.dc.
	input reset,					// Input: System reset.
	input clk);						// Input: System clock.
		
	///////////////////////////////////////////////////////////////
	/////////////////////// Implementation ////////////////////////
	///////////////////////////////////////////////////////////////
	
	always @(posedge clk)
	begin
	
		// 'reset' takes precedence over other inputs.
		if(reset) 
			status_reg_bus <= 0;
		else begin
		
			if(load_status_reg) begin
				// If ANY of the status bits are being updated by 'status_bus' we do 
				// not allow a write to these bits by the parallel loaded data.
				if(load_z || load_c || load_dc)
					status_reg_bus[7:3] <= alu_bus[7:3];
				else
					status_reg_bus <= alu_bus;
			end
				
			if(load_z)
				status_reg_bus[`STATUS_Z] = alu_status_bus[`STATUS_Z];
				
			if(load_c)
				status_reg_bus[`STATUS_C] = alu_status_bus[`STATUS_C];
		
			if(load_dc)
				status_reg_bus[`STATUS_DC] = alu_status_bus[`STATUS_DC];
		end
	end

	assign carry_bit = status_reg_bus[`STATUS_C];
		
endmodule

/////////////////////////////////////////////////////////////////
// module pic10_sfr_data_mux
// Tested OK 1/22/07 with testbench 'test_pic10_sfr_data_mux'.
/////////////////////////////////////////////////////////////////
module pic10_sfr_data_mux(
	output reg [7:0] sfr_data_bus,	// Output: Operand data from the special function registers 0..7.
	input [2:0] sfr_data_mux_sel,	// Input: This is the currently selected register address (either
									// a direct address	or an indirect address selected by the Register
									// Address Mux). The SFR Data Mux only uses the three lowest bits.		
	input [7:0] indf_reg_bus,		// Input: The INDF register's output.
	input [7:0] tmr0_reg_bus,		// Input: The TMR0 register's output.
	input [7:0] pcl_reg_bus,		// Input: The PCL register's output (the lowest 8 bits of the 9-bit PC).
	input [7:0] status_reg_bus,		// Input: The STATUS register's output.
	input [7:0] fsr_reg_bus,		// Input: The FSR register's output.
	input [7:0] triport5_bus,		// Input: The value read from GPIO port 5 (replaces OSCCAL register).
	input [7:0] triport6_bus,		// Input: The value read from GPIO port 6.
	input [7:0] triport7_bus);		// Input: The value read from GPIO port 7 (replaces CMCON0 register).

	///////////////////////////////////////////////////////////////
	/////////////////////// Implementation ////////////////////////
	///////////////////////////////////////////////////////////////

	always @(sfr_data_mux_sel, indf_reg_bus, tmr0_reg_bus, pcl_reg_bus,
		status_reg_bus, fsr_reg_bus, triport5_bus, triport6_bus, triport7_bus) 
	begin
		
		case(sfr_data_mux_sel)
			3'd0:	sfr_data_bus = indf_reg_bus;
			3'd1:	sfr_data_bus = tmr0_reg_bus;
			3'd2:	sfr_data_bus = pcl_reg_bus;
			3'd3:	sfr_data_bus = status_reg_bus;
			3'd4:	sfr_data_bus = fsr_reg_bus;
			3'd5:	sfr_data_bus = triport5_bus;
			3'd6:	sfr_data_bus = triport6_bus;
			3'd7:	sfr_data_bus = triport7_bus;
		endcase
		
	end
	
endmodule

/////////////////////////////////////////////////////////////////
// module pic10_register_address_mux
// Tested OK 1/22/07 with testbench 'test_pic10_register_address_mux'.
/////////////////////////////////////////////////////////////////
module pic10_register_address_mux(
	output [4:0] reg_addr_bus,		// Output: This is the currently selected register address (either
									// a direct address	or an indirect address selected by the Register
									// Address Mux).
	input [4:0] ir_reg_bus,			// Input: This part-select of the complete instruction word contains 
									// a 'literal' constant specifying the direct address used to access 
									// a register (note: only for selected data-moving instructions).
	input [4:0] fsr_reg_bus,		// Input: Contains the indirect register address in the FSR register.
									// The indirect address is only used when accessing a register via
									// the FSR register.
	input reg_addr_mux_sel);		// Input: Selects the register address to be used when reading or writing a
									// register. Can either be a direct address (0) or an indirect address (1).

	///////////////////////////////////////////////////////////////
	/////////////////////// Implementation ////////////////////////
	///////////////////////////////////////////////////////////////

	assign reg_addr_bus = reg_addr_mux_sel ? fsr_reg_bus : ir_reg_bus;
	
endmodule

/////////////////////////////////////////////////////////////////
// module pic10_ram_registers
// Tested OK 1/21/07 with testbench 'test_pic10_ram_registers'.
/////////////////////////////////////////////////////////////////
module pic10_ram_registers(
	output [7:0] ram_data_bus,	// Output: Always outputs the data in the addressed register.
	input [7:0] alu_bus,		// Input: The output from the ALU. Loaded into the register when the
								// load_ram_reg signal is asserted. 
	input load_ram_reg,			// Input: Loads alu_bus into a General-purpose RAM register with the address 
								// driven on reg_addr_bus. The data is clocked in on the next 
								// posedge(clk). 
	input [4:0] reg_addr_bus,	// Input: This is the currently selected register address (either
								// a direct address	or an indirect address selected by the Register
								// Address Mux). Note: The first RAM register is at address 8 because
								// the Special Function Registers have addresses 0..7.
	input reset,				// Input: System reset.
	input clk);					// Input: System clock.

	///////////////////////////////////////////////////////////////
	/////////////////////// Implementation ////////////////////////
	///////////////////////////////////////////////////////////////

	// This is the RAM register storage (24 8-bit registers).
	reg [7:0] ram_regs[24];
	
	// This is a loop variable used during reset to initialize the RAM.
	integer i;
	
	always @(posedge clk)
	begin
	
		if(reset) begin
			// Clear all RAM registers.
			for(i=0; i<24; i=i+1)
				ram_regs[i] <= 0;
		end
		else if(load_ram_reg) begin
			ram_regs[reg_addr_bus - 8] <= alu_bus;
		end	

	end

	// The RAM registers always output the addressed register.
	// NOTE: Output 00h for addresses we don't decode (our first
	// RAM register starts at address 8).
	// NOTE: Subtract 8 because address 0..7 addresses the SFR registers.
	assign ram_data_bus = reg_addr_bus >= 5'd8 ? ram_regs[reg_addr_bus - 8] : 0;

endmodule

/////////////////////////////////////////////////////////////////
// module pic10_alu_mux
// Tested OK 1/20/07 with testbench 'test_pic10_alu_mux'.
/////////////////////////////////////////////////////////////////
module pic10_alu_mux(
	output [7:0] alu_mux_bus,	// Output: This is the bus that drives the ALUs 2nd operand.
	input [7:0] sfr_data_bus,	// Input: Operand data from the special function registers 0..7.
	input [7:0] ram_data_bus,	// Input: Operand data from the general-purpose RAM registers.
	input alu_mux_sel);			// Input: Selects the ALU's 2nd operand to be either the data on the 
								// sfr_data_bus (0) or the data on the ram_data_bus (1).

	///////////////////////////////////////////////////////////////
	/////////////////////// Implementation ////////////////////////
	///////////////////////////////////////////////////////////////

	assign alu_mux_bus = alu_mux_sel ? ram_data_bus : sfr_data_bus;
	
endmodule

module pic10_alu(
	output reg [7:0] alu_bus,	// Output: The output from the ALU. This can either be
								// an unmodified, passed-through operand (W or register),
								// a literal (constant) from the instruction word or
								// the result from an arithmetic or logical operation
								// or some combination of the W reg, registers and a 
								// literal.
	output reg load_z,			// Output: The ALU will set this signal when the Z bit is affected 
								// by an ALU operation. The Z bit itself is part of the alu_status_bus.
	output reg load_c,			// Output: The ALU will set this signal when the C bit is affected 
								// by an ALU operation. The C bit itself is part of the alu_status_bus.
	output reg load_dc,			// Output: The ALU will set this signal when the DC bit is affected 
								// by an ALU operation. The DC bit itself is part of the alu_status_bus.
	output reg [2:0] alu_status_bus,// Output: The ALU drives the Z, C and/or DC status bits if affected
								// by the ALU operation input on the ir_reg_bus input.
	input [7:0] w_reg_bus,		// Input: The output from the W register. Used as 1st ALU operand.
	input [7:0] alu_mux_bus,	// Input: This is the bus that drives the ALUs 2nd operand.
	input [11:0] ir_reg_bus,	// Input: The ALU retrieves the opcode and literal operands from this bus.
	input carry_bit);			// Input: The carry bit from the STATUS register.


	///////////////////////////////////////////////////////////////
	/////////////////////// Implementation ////////////////////////
	///////////////////////////////////////////////////////////////

	always @(w_reg_bus, alu_mux_bus, ir_reg_bus, carry_bit)
	begin

		// Clear the old status. It will be updated by the below ALU operations.
		alu_status_bus = 0;
		load_z = 0;
		load_c = 0;
		load_dc = 0;
      
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
			// NOTE: No ALU operation needed.
			
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
			12'b0110_xxxx_xxxx: bittest();	// NOTE: Common implementation for BTFSC and BTFSS.
	
			// BTFSS	f, b		Bit Test f,				1(2)	0111 bbbf ffff		None	
			//						Skip if Set	
			12'b0111_xxxx_xxxx: bittest();	// NOTE: Common implementation for BTFSC and BTFSS.

			// ANDLW	k			AND literal with W		1		1110 kkkk kkkk		Z
			12'b1110_xxxx_xxxx: andlw();
				
			// CALL		k			Call Subroutine			2		1001 kkkk kkkk		None			1
			12'b1001_xxxx_xxxx: call();
			
			// CLRWDT	-			Clear Watchdog Timer	1		0000 0000 0100		TO, PD
			// NOTE: Not implemented instruction.
				
			// GOTO		k			Unconditional branch	2		101k kkkk kkkk		None
			// NOTE: No ALU operation needed. The controller will 
			// load the 9-bit literal directly from the ir_reg_bus.
				
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

	end

	task addwf();
	reg [3:0] dummy_result_nybble;    
	begin
	    
	    $display("pic10_alu: addwf");
	    
		// The controller has already set up the datapath to the operands.
		// Add the two operands, set the ALU output and update the status bus.
		
		// The result and carry flag are calculated in one operation.
		{alu_status_bus[`STATUS_C], alu_bus} = w_reg_bus + alu_mux_bus;
		
		// The Digit Carry (DC) flag reflects a carry from the lower nybble
		// just like only the lower nybble had been added.
		{alu_status_bus[`STATUS_DC], dummy_result_nybble} = 
		   w_reg_bus[3:0] + alu_mux_bus[3:0];
		
		// The Z flag is set if the result is zero.
		alu_status_bus[`STATUS_Z] = (alu_bus == 8'b0);
		
		// The ADDWF affects all three STATUS register flags.
		load_z = 1;
		load_c = 1;
		load_dc = 1;
	end
	endtask
	
	task andwf();
	begin

       $display("pic10_alu: andwf");
       
		// The controller has already set up the datapath to the operands.
		// AND the two operands, set the ALU output and update the status bus.
		
		// AND the two operands and set the ALU output.
		alu_bus = w_reg_bus & alu_mux_bus;
		
		// Update the status bus. The Z flag is set if the result is zero.
		alu_status_bus[`STATUS_Z] = (alu_bus == 8'b0);
		
		// The ANDWF instruction affects the Z STATUS register flag.
		load_z = 1;

	end
	endtask
	
	task crlf();
	begin
   
      $display("pic10_alu: crlf");
      
		// There are no ALU operands, we simply output 00h.
		// The controller will load the ALU output into the 
		// 'f' register indicated in the instruction word.
		alu_bus = 8'b0;
		
		// Update the status bus. The Z flag is always set since the result is zero.
		alu_status_bus[`STATUS_Z] = 1;
		
		// The CLRF instruction affects the Z STATUS register flag.
		load_z = 1;

	end
	endtask	
	
	task clrw();
	begin
      
      $display("pic10_alu: clrw");
      
		// There are no ALU operands, we simply output 00h.
		// The controller will load the ALU output into the 
		// 'W' register.
		alu_bus = 8'b0;
		
		// Update the status bus. The Z flag is always set since the result is zero.
		alu_status_bus[`STATUS_Z] = 1;
		
		// The CLRW instruction affects the Z STATUS register flag.
		load_z = 1;

	end
	endtask	
	
	task comf();
	begin
      
      $display("pic10_alu: comf");
      
		// The controller has already set up the datapath to the operands.
		// Complement the 2nd operand, set the ALU output and update the status bus.
		
		// Complement the 2nd operand and set the ALU output.
		alu_bus = ~alu_mux_bus;
		
		// Update the status bus. The Z flag is set if the result is zero.
		alu_status_bus[`STATUS_Z] = (alu_bus == 8'b0);
		
		// The COMF instruction affects the Z STATUS register flag.
		load_z = 1;

	end
	endtask
	
	task decf();
	begin

      $display("pic10_alu: decf");
      
		// The controller has already set up the datapath to the operands.
		// Decrement the 2nd operand, set the ALU output and update the status bus.
		
		// Decrement the 2nd operand and set the ALU output.
		alu_bus = alu_mux_bus - 1;
		
		// Update the status bus. The Z flag is set if the result is zero.
		alu_status_bus[`STATUS_Z] = (alu_bus == 8'b0);
		
		// The DECF instruction affects the Z STATUS register flag.
		load_z = 1;

	end
	endtask	

	task decfsz();
	begin

      $display("pic10_alu: decfsz");
      
		// The controller has already set up the datapath to the operands.
		// Decrement the 2nd operand. NOTE that no status flags are updated.
		
		// Decrement the 2nd operand and set the ALU output.
		alu_bus = alu_mux_bus - 1;

		// Update the status bus. The Z flag is set if the result is zero.
		// NOTE that the 'load_z' signal is never asserted. The controller
		// needs to know if we have a zero result in order to skip over the
		// next instruction.
		alu_status_bus[`STATUS_Z] = (alu_bus == 8'b0);

	end
	endtask	
	
	task incf();
	begin
      
      $display("pic10_alu: incf");
      
		// The controller has already set up the datapath to the operands.
		// Increment the 2nd operand, set the ALU output and update the status bus.
		
		// Increment the 2nd operand and set the ALU output.
		alu_bus = alu_mux_bus + 1;
		
		// Update the status bus. The Z flag is set if the result is zero.
		alu_status_bus[`STATUS_Z] = (alu_bus == 8'b0);
		
		// The INCF instruction affects the Z STATUS register flag.
		load_z = 1;

	end
	endtask	

	task incfsz();
	begin
      
      $display("pic10_alu: incfsz");
      
		// The controller has already set up the datapath to the operands.
		// Increment the 2nd operand. NOTE that no status flags are updated.
		
		// Increment the 2nd operand and set the ALU output.
		alu_bus = alu_mux_bus + 1;

		// Update the status bus. The Z flag is set if the result is zero.
		// NOTE that the 'load_z' signal is never asserted. The controller
		// needs to know if we have a zero result in order to skip over the
		// next instruction.
		alu_status_bus[`STATUS_Z] = (alu_bus == 8'b0);

	end
	endtask	
	
	task iorwf();
	begin
       
       $display("pic10_alu: iorwf");
       
		// The controller has already set up the datapath to the operands.
		// OR the two operands, set the ALU output and update the status bus.
		
		// OR the two operands and set the ALU output.
		alu_bus = w_reg_bus | alu_mux_bus;
		
		// Update the status bus. The Z flag is set if the result is zero.
		alu_status_bus[`STATUS_Z] = (alu_bus == 8'b0);
		
		// The IORWF instruction affects the Z STATUS register flag.
		load_z = 1;

	end
	endtask

	task movf();
	begin
      
      $display("pic10_alu: movf");
      
		// The controller has already set up the datapath to the operands.
		// Simply pass through the 2nd 'f' operand. 
		alu_bus = alu_mux_bus;
		
		// Update the status bus. The Z flag is set if the result is zero.
		alu_status_bus[`STATUS_Z] = (alu_bus == 8'b0);
		
		// The instruction affects the Z STATUS register flag.
		load_z = 1;
		
	end
	endtask	
	
	task movwf();
	begin
      
      $display("pic10_alu: movwf");
      
		// The controller has already set up the datapath to the operands.
		// Simply pass through the 1st 'W' operand. No flags are affected.
		alu_bus = w_reg_bus;
		
	end
	endtask
	
	task rlf();
	begin
      
      $display("pic10_alu: rlf");
      
		// The controller has already set up the datapath to the operands.

		// Rotate the 2nd ALU operand left one bit through the Carry bit.
		{alu_status_bus[`STATUS_C], alu_bus} = {alu_mux_bus, carry_bit};
		
		// The RLF instruction affects the C STATUS register flag.
		load_c = 1;

	end
	endtask

	task rrf();
	begin
      
      $display("pic10_alu: rrf");
      
		// The controller has already set up the datapath to the operands.

		// Rotate the 2nd ALU operand right one bit through the Carry bit.
		{alu_bus, alu_status_bus[`STATUS_C]} = {carry_bit, alu_mux_bus};
				
		// The RRF instruction affects the C STATUS register flag.
		load_c = 1;

	end
	endtask
	
	task subwf();
	begin
	    
	    $display("pic10_alu: subwf");
	
		// The controller has already set up the datapath to the operands.
		// Subtract W from f, set the ALU output and update the status bus.
		
		// Subtract W from f.
		alu_bus = alu_mux_bus - w_reg_bus;
		
		// The Carry flag (C) is cleared if borrow occured.
		// Borrow occurs (C=0) if the 2nd operand is larger than the 1st operand.
		// More conveniently, Borrow does not occur (C=1) if the 1st op >= 2nd op.
		alu_status_bus[`STATUS_C] = alu_mux_bus >= w_reg_bus;
		
		// The Digit Carry (DC) flag works the same way as the C flag but it
		// only deals with the 4 lower bits of the operands and result.
		alu_status_bus[`STATUS_DC] = alu_mux_bus[3:0] >= w_reg_bus[3:0];
		
		// The Z flag is set if the result is zero.
		alu_status_bus[`STATUS_Z] = (alu_bus == 8'b0);
		
		// The SUBWF instruction affects all three STATUS register flags.
		load_z = 1;
		load_c = 1;
		load_dc = 1;
	end
	endtask

	task swapf();
	begin
      
      $display("pic10_alu: swapf");
      
		// The controller has already set up the datapath to the operands.
		// Simply swap the nybbles of 'f', no flags are updated.
		alu_bus = {alu_mux_bus[3:0], alu_mux_bus[7:4]};

	end
	endtask
	
	task xorwf();
	begin
      
      $display("pic10_alu: xorwf");
      
		// The controller has already set up the datapath to the operands.
		// XOR the two operands, set the ALU output and update the status bus.
		
		// XOR the two operands and set the ALU output.
		alu_bus = w_reg_bus ^ alu_mux_bus;
		
		// Update the status bus. The Z flag is set if the result is zero.
		alu_status_bus[`STATUS_Z] = (alu_bus == 8'b0);
		
		// The XORWF instruction affects the Z STATUS register flag.
		load_z = 1;

	end
	endtask
	
	task bcf();
		reg [2:0] b;
		reg [7:0] mask;
	begin
      
      $display("pic10_alu: bcf");
      
		// The controller has already set up the datapath to the operands.
		// Clear bit 'b' of register 'f', no flags are updated.
		
		// The bit index resides in bits [7:5] of the instruction word.
		b = ir_reg_bus[7:5];
		
		// Create a bit mask with bit 'b' cleared.
		mask = ~(1<<b);
		
		// Clear bit 'b' by AND'ing 'f' with the mask.
		alu_bus = alu_mux_bus & mask;

	end
	endtask	
		
	task bsf();
		reg [2:0] b;
		reg [7:0] mask;
	begin
	    
	    $display("pic10_alu: bsf");

		// The controller has already set up the datapath to the operands.
		// Set bit 'b' of register 'f', no flags are updated.
		
		// The bit index resides in bits [7:5] of the instruction word.
		b = ir_reg_bus[7:5];
		
		// Create a bit mask with bit 'b' set.
		mask = (1<<b);
		
		// Set bit 'b' by OR'ing 'f' with the mask.
		alu_bus = alu_mux_bus | mask;

	end
	endtask	
	
	task bittest();
		reg [2:0] b;
		reg bitset;
		reg [7:0] mask;
	begin
      
      $display("pic10_alu: bittest");
      
		// The controller has already set up the datapath to the operands.
		// If bit 'b' of register 'f' is clear we set the Z flag. This
		// will notify the controller whether the next instruction should be
		// skipped (i.e. the PC is incremented). 
		
		// The bit index resides in bits [7:5] of the instruction word.
		b = ir_reg_bus[7:5];
		
		// Prepare a mask that has the indicated bit set.
		mask = (1<<b);
		
		// Find out if bit 'b' in the 'f' operand is set.
		bitset = ((alu_mux_bus & mask) == mask);
		
		// Update the status bus. NOTE: The Z flag on the bus will NOT be 
		// loaded into the status register (load_z will not be asserted)
		// but the controller will look at the Z bit to determine whether 
		// the next instruction should be skipped.
		alu_status_bus[`STATUS_Z] = !bitset;
			
		// Drive the ALU bus with arbitrary data (not used).
		alu_bus = 8'b0;

	end
	endtask	
	
	task andlw();
	begin
      
      $display("pic10_alu: andlw");
      
		// The controller has already set up the datapath to the operands.
		// AND 'W' with the literal (constant) in the instruction word, 
		// set the ALU output and update the status bus Z flag.
		
		// AND 'W' with the literal (bits [7:0] of the instruction word.
		alu_bus = w_reg_bus & ir_reg_bus[7:0];
		
		// Update the status bus. The Z flag is set if the result is zero.
		alu_status_bus[`STATUS_Z] = (alu_bus == 8'b0);
		
		// This instruction only affects the Z STATUS register flag.
		load_z = 1;

	end
	endtask	
	
	task call();
	begin
      
      $display("pic10_alu: call");
      
		// The controller has already set up the datapath to the operands.

		// Pass through the 8-bit literal to the output bus. The controller
		// will load it into the PCL register.
		alu_bus = ir_reg_bus[7:0];

	end
	endtask		
	
	task iorlw();
	begin
      
      $display("pic10_alu: iorlw");
      
		// The controller has already set up the datapath to the operands.
		// OR 'W' with the literal (constant) in the instruction word, 
		// set the ALU output and update the status bus Z flag.
		
		// OR 'W' with the literal (bits [7:0] of the instruction word.
		alu_bus = w_reg_bus | ir_reg_bus[7:0];
		
		// Update the status bus. The Z flag is set if the result is zero.
		alu_status_bus[`STATUS_Z] = (alu_bus == 8'b0);
		
		// This instruction only affects the Z STATUS register flag.
		load_z = 1;

	end
	endtask		
	
	task movlw();
	begin
      
      $display("pic10_alu: movlw");
      
		// The controller has already set up the datapath to the operands.
		
		// Pass through the literal to the output. The controller will load 
		// it into the W register. No flags are updated by this instruction.
		alu_bus = ir_reg_bus[7:0];

	end
	endtask	
	
	task option();
	begin
      
      $display("pic10_alu: option");
      
		// The controller has already set up the datapath to the operands.
		
		// Pass through the W register to the output. The controller will load 
		// it into the OPTION register. No flags are updated by this instruction.
		alu_bus = w_reg_bus;

	end
	endtask			
	
	task retlw();
	begin
      
      $display("pic10_alu: retlw");
      
		// The controller has already set up the datapath to the operands.
		
		// Pass through the 8-bit literal to our output bus. The controller 
		// will load the data into the W register.  No flags are updated.
		alu_bus = ir_reg_bus[7:0];

	end
	endtask		
	
	task tris();
	begin
      
      $display("pic10_alu: tris");
      
		// The controller has already set up the datapath to the operands.
		
		// Pass through the W register to the output. The controller will load 
		// it into the TRIS register. No flags are updated by this instruction.
		alu_bus = w_reg_bus;

	end
	endtask	

	task xorlw();
	begin
      
      $display("pic10_alu: xorlw");
      
 	   // The controller has already set up the datapath to the operands.
		// XOR 'W' with the literal (constant) in the instruction word, 
		// set the ALU output and update the status bus Z flag.
		
		// XOR 'W' with the literal (bits [7:0] of the instruction word.
		alu_bus = w_reg_bus ^ ir_reg_bus[7:0];
		
		// Update the status bus. The Z flag is set if the result is zero.
		alu_status_bus[`STATUS_Z] = (alu_bus == 8'b0);
		
		// This instruction only affects the Z STATUS register flag.
		load_z = 1;

	end
	endtask	
	
endmodule

/////////////////////////////////////////////////////////////////
// module pic10_w_reg
// Tested OK 1/20/07 with testbench 'test_pic10_w_reg'.
/////////////////////////////////////////////////////////////////
module pic10_w_reg(
	output reg [7:0] w_reg_bus,	// Output: The output from the W register. Used as 1st ALU operand.
	input [7:0] alu_bus,		// Input: The output from the ALU. This can either be
								// an unmodified, passed-through operand (W or register),
								// a literal (constant) from the instruction word or
								// the result from an arithmetic or logical operation
								// or some combination of the W reg, registers and a 
								// literal.
	input load_w_reg,			// Input: Loads the data on the alu_bus into the W register.
	input reset,				// Input: System reset.
	input clk);					// Input: System clock.

	/////////////////////// Implementation ////////////////////////

	always @(posedge clk)
	begin
		if(reset)
			w_reg_bus <= 0;
		else if (load_w_reg)
			w_reg_bus <= alu_bus;
	end

endmodule

/////////////////////////////////////////////////////////////////
// module pic10_tri_state_port
// Tested OK 1/20/07 with testbench 'test_pic10_tri_state_port'.
/////////////////////////////////////////////////////////////////
module pic10_tri_state_port(
	inout [7:0] gpio_pin_bus,	// Inout: This is the port bus connected to the outside world.
	output [7:0] triport_bus,	// Output: The value read from GPIO port. Replaces OSCCAL register.
	input [7:0] alu_bus,		// Input: Used to load register values into the TRIS or GPIO port (the
								// register to load is determined by asserting either the load_tris_reg
								// or the load_gpio_reg signal).
	input load_tris_reg,		// Input: Used to load the TRIS register.
	input load_gpio_reg,		// Input: Used to load the GPIO register.
	input reset,				// Input: System reset.
	input clk);					// Input: System clock.

	/////////////////////// Implementation ////////////////////////

	// This is the GPIO and TRIS register storage for this port.
	reg [7:0] gpio_reg;
	reg [7:0] tris_reg;

	always @(posedge clk)
	begin
		// We initialize the port as input (FFh) when reset.
		if(reset) begin
			gpio_reg <= 8'b0;
			tris_reg <= 8'hFF;
		end
		else if(load_tris_reg)
			tris_reg <= alu_bus;
		else if(load_gpio_reg)
			gpio_reg <= alu_bus;
	end

	// These combinational expressions will implement the inout bus. If a TRIS register bit is '1' then the
	// corresponding port bit will be an input bit. If a TRIS register bit is '0' then the corresponding
	// GPIO register bit will be driven out on the gpio port pin. The triport_bus will always reflect the signals
	// on the inout gpio_pin_bus (either the driven data for output pins or the external data for input pins).
	
	assign gpio_pin_bus[7] = tris_reg[7] ? 1'bz : gpio_reg[7];
	assign triport_bus[7] = tris_reg[7] ? gpio_pin_bus[7] : gpio_reg[7]; 

	assign gpio_pin_bus[6] = tris_reg[6] ? 1'bz : gpio_reg[6];
	assign triport_bus[6] = tris_reg[6] ? gpio_pin_bus[6] : gpio_reg[6]; 

	assign gpio_pin_bus[5] = tris_reg[5] ? 1'bz : gpio_reg[5];
	assign triport_bus[5] = tris_reg[5] ? gpio_pin_bus[5] : gpio_reg[5]; 

	assign gpio_pin_bus[4] = tris_reg[4] ? 1'bz : gpio_reg[4];
	assign triport_bus[4] = tris_reg[4] ? gpio_pin_bus[4] : gpio_reg[4]; 

	assign gpio_pin_bus[3] = tris_reg[3] ? 1'bz : gpio_reg[3];
	assign triport_bus[3] = tris_reg[3] ? gpio_pin_bus[3] : gpio_reg[3]; 

	assign gpio_pin_bus[2] = tris_reg[2] ? 1'bz : gpio_reg[2];
	assign triport_bus[2] = tris_reg[2] ? gpio_pin_bus[2] : gpio_reg[2]; 

	assign gpio_pin_bus[1] = tris_reg[1] ? 1'bz : gpio_reg[1];
	assign triport_bus[1] = tris_reg[1] ? gpio_pin_bus[1] : gpio_reg[1]; 

	assign gpio_pin_bus[0] = tris_reg[0] ? 1'bz : gpio_reg[0];
	assign triport_bus[0] = tris_reg[0] ? gpio_pin_bus[0] : gpio_reg[0]; 

endmodule

/////////////////////////////////////////////////////////////////
// module pic10_program_mux
/////////////////////////////////////////////////////////////////
module pic10_program_mux(
	output [11:0] program_mux_bus,	// Output: This is the bus that drives the ALUs 2nd operand.
	input [11:0] program_bus,		// Input: Operand data from the program_bus (currently addressed instruction).
	input [11:0] nop_bus,			// Input: A NOP instruction Op-code (000h).
	input program_mux_sel);			// Input: When asserted will select the NOP op-code instead of the real op-code.
									//        This is used by the 'skip' instructions to have the datapath execute
									//        a NOP instead of the actual instruction on the program bus.

	///////////////////////////////////////////////////////////////
	/////////////////////// Implementation ////////////////////////
	///////////////////////////////////////////////////////////////

	assign program_mux_bus = program_mux_sel ? nop_bus : program_bus;
	
endmodule

