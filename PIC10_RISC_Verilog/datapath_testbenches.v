/test_pic10_datapath****************************************************************************
*
* By John.Gulbrandsen@SummitSoftConsulting.com, 1/17/2007.
*
* This file contains testbenches for the modules used in the PIC10 datapath.
*
****************************************************************************/

// These constants determine the signals on the status_bus (output from ALU)
// as well as on the status_reg_bus (output from STATUS register).
`define STATUS_C	0
`define STATUS_DC	1
`define STATUS_Z	2	

// NOTE: VERIFIED OK.
module test_pic10_tri_state_port;

	// These are the output signals from the Device Under Test:
	tri1 [7:0] gpio_pin_bus;		// This is the port bus connected to the outside world.
	                           // NOTE: We have pull-ups on tall port pins so all inputs
	                           // will read '1'.
	wire [7:0] triport_bus;		// The value read from GPIO port. 
	
	// These are the input signals to the Device Under Test:
	reg [7:0] alu_bus;			// Input: Used to load register values into the TRIS or GPIO port (the
								// register to load is determined by asserting either the load_tris_reg
								// or the load_gpio_reg signal).
	reg load_tris_reg;			// Input: Used to load the TRIS register.
	reg load_gpio_reg;			// Input: Used to load the GPIO register.
	reg reset;					// Input: System reset.
	reg clk;					// Input: System clock.

	// This is the Device Under Test.
	pic10_tri_state_port m1(
		gpio_pin_bus,			// Inout: This is the port bus connected to the outside world.
		triport_bus,			// Output: The value read from GPIO port. Replaces OSCCAL register.
		alu_bus,				// Input: Used to load register values into the TRIS or GPIO port (the
								// register to load is determined by asserting either the load_tris_reg
								// or the load_gpio_reg signal).
		load_tris_reg,			// Input: Used to load the TRIS register.
		load_gpio_reg,			// Input: Used to load the GPIO register.
		reset,					// Input: System reset.
		clk);	

	// Generate the clock signal.
	initial clk = 0;
	always #1 clk = ~clk;
	
	// Assert reset from t=5 to t=10. After reset all input pins
	// should be inputs and the triport_bus should read FFh.
	initial 
	begin
		reset <= #5 1;
		reset <= #15 0;
	end

	// Configure every other port bit as input at t=20.
	// Since the gpio registers are reset as 00h the 
	// triport_bus should now read 55h.
	initial 
	begin
		alu_bus <= #20 8'h55;
		load_tris_reg <= #20 1;
		load_tris_reg <= #25 0;
	end
	
	// Write F0h to the GPIO register at t=30.
	// The triport_bus should now read F5h.
	initial 
	begin
		alu_bus <= #30 8'hF0;
		load_gpio_reg <= #30 1;
		load_gpio_reg <= #35 0;
	end	

endmodule

// NOTE: VERIFIED OK.
module test_pic10_w_reg;

	// These are the output signals from the Device Under Test:
	wire [7:0] w_reg_bus;	// Output: The output from the W register. Used as 1st ALU operand.

	// These are the input signals to the Device Under Test:
	reg [7:0] alu_bus;		// Input: The output from the ALU. This can either be
							// an unmodified, passed-through operand (W or register),
							// a literal (constant) from the instruction word or
							// the result from an arithmetic or logical operation
							// or some combination of the W reg, registers and a 
							// literal.
	reg load_w_reg;			// Input: Loads the data on the alu_bus into the W register.
	reg reset;				// Input: System reset.
	reg clk;				// Input: System clock.

	// This is the Device Under Test.
	pic10_w_reg m1(
		w_reg_bus,			// Output: The output from the W register. Used as 1st ALU operand.
		alu_bus,			// Input: The output from the ALU. This can either be
							// an unmodified, passed-through operand (W or register),
							// a literal (constant) from the instruction word or
							// the result from an arithmetic or logical operation
							// or some combination of the W reg, registers and a 
							// literal.
		load_w_reg,			// Input: Loads the data on the alu_bus into the W register.
		reset,				// Input: System reset.
		clk);				// Input: System clock.

	// Generate the clock signal.
	initial clk = 0;
	always #1 clk = ~clk;
	
	// Assert reset from t=5 to t=10. After reset the register
	// should be cleared and the w_reg_bus should read 00h.
	initial 
	begin
		reset <= #5 1;
		reset <= #15 0;
	end

	// Load 55h into the register at t=20;
	initial 
	begin
		alu_bus <= #20 8'h55;
		load_w_reg <= #20 1;
		load_w_reg <= #25 0;
	end
	
	// Load AAh into the register at t=30;
	initial 
	begin
		alu_bus <= #30 8'hAA;
		load_w_reg <= #30 1;
		load_w_reg <= #35 0;
	end

endmodule

// NOTE: VERIFIED OK.
module test_pic10_alu_mux;

	// These are the output signals from the Device Under Test:
	wire [7:0] alu_mux_bus;			// Output: This is the bus that drives the ALUs 2nd operand.

	// These are the input signals to the Device Under Test:
	reg [7:0] sfr_data_bus = 8'h55;	// Input: Operand data from the special function registers 0..7.
	reg [7:0] ram_data_bus = 8'hAA;	// Input: Operand data from the general-purpose RAM registers.
	reg alu_mux_sel = 0;			// Input: Selects the ALU's 2nd operand to be either the data on the 
	                        // sfr_data_bus (0) or the data on the ram_data_bus (1).

	// This is the Device Under Test.
	pic10_alu_mux m1(
		alu_mux_bus,				// Output: This is the bus that drives the ALUs 2nd operand.
		sfr_data_bus,				// Input: Operand data from the special function registers 0..7.
		ram_data_bus,				// Input: Operand data from the general-purpose RAM registers.
		alu_mux_sel);				// Input: Selects the ALU's 2nd operand to be either the data on the 
									// sfr_data_bus (0) or the data on the ram_data_bus (1).

	// Toggle the mux select input every 10 time units.
	always #10 alu_mux_sel = ~alu_mux_sel;

endmodule

// NOTE: VERIFIED OK.
module test_pic10_ram_registers;

	// These are the output signals from the Device Under Test:
	wire [7:0] ram_data_bus;// Output: Always outputs the data in the addressed register.

	// These are the input signals to the Device Under Test:
	reg [7:0] alu_bus=0;		// Input: The output from the ALU. Loaded into the register when the
							// load_ram_reg signal is asserted. 
	reg load_ram_reg=0;		// Input: Loads alu_bus into a General-purpose RAM register with the address 
							// driven on reg_addr_bus. The data is clocked in on the next 
							// posedge(clk). 
	reg [4:0] reg_addr_bus=0;	// Input: This is the currently selected register address (either
							// a direct address	or an indirect address selected by the Register
							// Address Mux). Note: The first RAM register is at address 8 because
							// the Special Function Registers have addresses 0..7.
	reg reset=0;				// Input: System reset.
	reg clk=0;					// Input: System clock.

	// This is the Device Under Test.
	pic10_ram_registers m1(
		ram_data_bus,		// Output: Always outputs the data in the addressed register.
		alu_bus,			// Input: The output from the ALU. Loaded into the register when the
							// load_ram_reg signal is asserted. 
		load_ram_reg,		// Input: Loads alu_bus into a General-purpose RAM register with the address 
							// driven on reg_addr_bus. The data is clocked in on the next 
							// posedge(clk). 
		reg_addr_bus,		// Input: This is the currently selected register address (either
							// a direct address	or an indirect address selected by the Register
							// Address Mux). Note: The first RAM register is at address 8 because
							// the Special Function Registers have addresses 0..7.
		reset,				// Input: System reset.
		clk);	
		
	// Generate the clock signal.
	always #1 clk = ~clk;
	
	// Assert reset from t=5 to t=8. After reset the registers
	// should be cleared and the ram_data_bus should read 00h
	// for all addresses (8..31).
	initial 
	begin
		reset <= #5 1;
		reset <= #8 0;
	end

	// Generate known data that we'll load into the RAM registers.
	always #1 alu_bus = alu_bus + 1;

	initial 
	begin
		// Start loading the RAM registers at t=12.
		load_ram_reg <= #12 1;	
	
		// Stop loading the RAM registers at t=64.
		load_ram_reg <= #64 0;	
	end

	// Increment the address every clock cycle so we 
	// clock in unique data in each register RAM location.
	always
	begin:load_block
		#2
		reg_addr_bus <= reg_addr_bus + 1;
		if(reg_addr_bus == 5'd31)
			disable load_block;
	end
	

endmodule

// NOTE: VERIFIED OK.
module test_pic10_register_address_mux;

	// These are the output signals from the Device Under Test:
	wire [4:0] reg_addr_bus;	// Output: This is the currently selected register address (either
								// a direct address	or an indirect address selected by the Register
								// Address Mux).

	// These are the input signals to the Device Under Test:
	reg	[4:0] ir_reg_bus=5'b01010;	// Input: This part-select of the complete instruction word contains 
								// a 'literal' constant specifying the direct address used to access 
								// a register (note: only for selected data-moving instructions).
	reg	[4:0] fsr_reg_bus=5'b10101;	// Input: Contains the indirect register address in the FSR register.
								// The indirect address is only used when accessing a register via
								// the FSR register.
	reg	reg_addr_mux_sel=0;		// Input: Selects the register address to be used when reading or writing a
								// register. Can either be a direct address (0) or an indirect address (1).

	// This is the Device Under Test.
	pic10_register_address_mux m1(
		reg_addr_bus,			// Output: This is the currently selected register address (either
								// a direct address	or an indirect address selected by the Register
								// Address Mux).
		ir_reg_bus,				// Input: This part-select of the complete instruction word contains 
								// a 'literal' constant specifying the direct address used to access 
								// a register (note: only for selected data-moving instructions).
		fsr_reg_bus,			// Input: Contains the indirect register address in the FSR register.
								// The indirect address is only used when accessing a register via
								// the FSR register.
		reg_addr_mux_sel);		// Input: Selects the register address to be used when reading or writing a
								// register. Can either be a direct address (0) or an indirect address (1).

	// Toggle the mux select input every time unit.
	always #1 reg_addr_mux_sel = ~reg_addr_mux_sel;

endmodule

// NOTE: VERIFIED OK:
module test_pic10_sfr_data_mux;

	// These are the output signals from the Device Under Test:
	wire [7:0] sfr_data_bus;			// Output: Operand data from the special function registers 0..7.

	// These are the input signals to the Device Under Test:
	reg [2:0] sfr_data_mux_sel = 0;			// Input: This is the currently selected register address (either
										// a direct address	or an indirect address selected by the Register
										// Address Mux). The SFR Data Mux only uses the three lowest bits.		
	reg [7:0] indf_reg_bus = 8'h00;		// Input: The INDF register's output.
	reg [7:0] tmr0_reg_bus = 8'h11;		// Input: The TMR0 register's output.
	reg [7:0] pcl_reg_bus = 8'h22;		// Input: The PCL register's output (the lowest 8 bits of the 9-bit PC).
	reg [7:0] status_reg_bus = 8'h33;	// Input: The STATUS register's output.
	reg [7:0] fsr_reg_bus = 8'h44;		// Input: The FSR register's output.
	reg [7:0] triport5_bus = 8'h55;		// Input: The value read from GPIO port 5 (replaces OSCCAL register).
	reg [7:0] triport6_bus = 8'h66;		// Input: The value read from GPIO port 6.
	reg [7:0] triport7_bus = 8'h77;		// Input: The value read from GPIO port 7 (replaces CMCON0 register).

	// This is the Device Under Test.
	pic10_sfr_data_mux m1(
		sfr_data_bus,			// Output: Operand data from the special function registers 0..7.
		sfr_data_mux_sel,		// Input: This is the currently selected register address (either
								// a direct address	or an indirect address selected by the Register
								// Address Mux). The SFR Data Mux only uses the three lowest bits.	
		indf_reg_bus,				// Input: INDF returns 00h when read.
		tmr0_reg_bus,				// Input: TMR0 returns 00h when read (not implemented).
		pcl_reg_bus,					// Input: PCL (the lowest 8 bits of the 9-bit PC).
		status_reg_bus,			// Input: The STATUS register's output.
		fsr_reg_bus,			// Input: The FSR register's output.
		triport5_bus,			// Input: The value read from GPIO port 5. Replaces OSCCAL register.
		triport6_bus,			// Input: The value read from GPIO port 6.
		triport7_bus);			// Input: The value read from GPIO port 7. Replaces CMCON0 register.

	// Increment the mux select every 10 clock cycles.
	always #10 sfr_data_mux_sel = sfr_data_mux_sel + 1;

endmodule

// NOTE: VERIFIED OK:
module test_pic10_status_reg;

	// These are the output signals from the Device Under Test:
	wire [7:0] status_reg_bus;	// Output: The current status is always driven.
	wire carry_bit;				// Output: The carry bit from the STATUS register.

	// These are the input signals to the Device Under Test:
	reg [7:0] alu_bus;			// Input: Data to be loaded when asserting 'load_status_reg'.
	reg load_status_reg;		// Input: Loads alu_bus into register at next posedge(clk).
	reg [2:0] alu_status_bus;	// Input: Z, C and DC status bits. Driven by ALU.
	reg load_z = 0;				// Input: Changes Z bit accordingly to alu_status_bus.z.
	reg load_c = 0;				// Input: Changes C bit accordingly to alu_status_bus.c.
	reg load_dc = 0;			// Input: Changes DC bit accordingly to alu_status_bus.dc.
	reg reset = 0;				// Input: System reset.
	reg clk = 0;				// Input: System clock.

	// This is the Device Under Test.
	pic10_status_reg m1(
		status_reg_bus,			// Output: The current status is always driven.
		carry_bit,				// Output: The carry bit from the STATUS register.
		alu_bus,				// Input: Data to be loaded when asserting 'load_status_reg'.
		load_status_reg,		// Input: Loads alu_bus into register at next posedge(clk).
		alu_status_bus,			// Input: Z, C and DC status bits. Driven by ALU.
		load_z,					// Input: Changes Z bit accordingly to alu_status_bus.z.
		load_c,					// Input: Changes C bit accordingly to alu_status_bus.c.
		load_dc,				// Input: Changes DC bit accordingly to alu_status_bus.dc.
		reset,					// System reset.
		clk);					// System clock.

	// Generate the clock signal.
	always #1 clk = ~clk;

	// Assert reset from t=3 to t=5.
	// status_reg_bus should now be 00h.
	initial begin
		reset <= #3 1;
		reset <= #5 0;
	end

	// Set the Z flag at t=10.
	initial begin
		alu_status_bus[`STATUS_Z] <= #10 1;
		load_z <= #10 1;
		load_z <= #12 0;
	end
	
	// Set the C flag at t=15.
	initial begin
		alu_status_bus[`STATUS_C] <= #15 1;
		load_c <= #15 1;
		load_c <= #17 0;
	end
	
	// Set the DC flag at t=20.
	initial begin
		alu_status_bus[`STATUS_DC] <= #20 1;
		load_dc <= #20 1;
		load_dc <= #22 0;
	end

	// Parallel load the register at t=30.
	initial begin
		alu_bus <= #30 8'h55;
		load_status_reg <= #30 1;
		load_status_reg <= #32 0;
	end
	

endmodule

// NOTE: VERIFIED OK:
module test_pic10_alu;

	// These are the output signals from the Device Under Test:
	wire [7:0] alu_bus;			// Output: The output from the ALU. This can either be
								// an unmodified, passed-through operand (W or register),
								// a literal (constant) from the instruction word or
								// the result from an arithmetic or logical operation
								// or some combination of the W reg, registers and a 
								// literal.
	wire load_z;				// Output: The ALU will set this signal when the Z bit is affected 
								// by an ALU operation. The Z bit itself is part of the alu_status_bus.
	wire load_c;				// Output: The ALU will set this signal when the C bit is affected 
								// by an ALU operation. The C bit itself is part of the alu_status_bus.
	wire load_dc;				// Output: The ALU will set this signal when the DC bit is affected 
								// by an ALU operation. The DC bit itself is part of the alu_status_bus.
	wire [2:0] alu_status_bus;	// Output: The ALU drives the Z, C and/or DC status bits if affected
								// by the ALU operation input on the ir_reg_bus input.

	// These are the input signals to the Device Under Test:
	reg [7:0] w_reg_bus;		// Input: The output from the W register. Used as 1st ALU operand.
	reg [7:0] alu_mux_bus;		// Input: This is the bus that drives the ALUs 2nd operand.
	reg [11:0] ir_reg_bus;		// Input: The ALU retrieves the opcode and literal operands from this bus.
	reg carry_bit=0;				// Input: The carry bit from the STATUS register.

	// This is the Device Under Test.
	pic10_alu m1(
		alu_bus,				// Output: The output from the ALU. This can either be
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
		alu_status_bus,			// Output: The ALU drives the Z, C and/or DC status bits if affected
								// by the ALU operation input on the ir_reg_bus input.
		w_reg_bus,				// Input: The output from the W register. Used as 1st ALU operand.
		alu_mux_bus,			// Input: This is the bus that drives the ALUs 2nd operand.
		ir_reg_bus,				// Input: The ALU retrieves the opcode and literal operands from this bus.
		carry_bit);				// Input: The carry bit from the STATUS register.

	// Test the instructions.
	initial begin	
		TestAddwf();
		TestAndwf();
		TestCrlf();
		TestClrw();
		TestComf();
		TestDecf();
		TestIncf();
		TestIorwf();
		TestMovf();
		TestMovwf();
		TestRlf();
		TestRrf();
		TestSubwf();
		TestSwapf();
		TestXorwf();
		TestBcf();
		TestBsf();
		TestBittest();
		TestAndlw();
		TestCall();
		TestIorlw();
		TestMovlw();
		TestOption();
		TestRetlw();
		TestTris();
		TestXorlw();
	end	
	
	task TestAddwf(); // Tested OK 1/22/07.
	begin
		
		$display("TestAddwf");
		
		// Set up the instruction word. Note that we clear out the source and destination 
		// register fields in the instruction word since that information is used by the 
		// controller to set up the datapath. The ALU doesn't care where the data is 
		// coming from or where it is going.
		ir_reg_bus = 12'b0001_1100_0000;
	
		// Let's add F0h and 16h. This should result in the result 06h with a 1 in the carry.
		w_reg_bus = 8'hF0;
		alu_mux_bus = 8'h16;
		
		#5;
		
		// Then let's add 0Fh and 01h. This should result in the DC flag being set.
		w_reg_bus = 8'h0F;
		alu_mux_bus = 8'h01;
	
	 	#5;
		
		// Then let's add FFh and 01h. This should result in the C, DC and Z flags being set.
		w_reg_bus = 8'hFF;
		alu_mux_bus = 8'h01;

		#5;
	  
	end
	endtask
	
	task TestAndwf(); // Tested OK 1/23/07.
	begin
		
		$display("TestAndwf");
		
		// Set up the instruction word. Note that we clear out the source and destination 
		// register fields in the instruction word since that information is used by the 
		// controller to set up the datapath. The ALU doesn't care where the data is 
		// coming from or where it is going.
		ir_reg_bus = 12'b0001_0100_0000;
	
		// Let's AND FFh and 55h. This should result in the result 55h and a clear Z flag.
		w_reg_bus = 8'hFF;
		alu_mux_bus = 8'h55;
		
		#5;
		
		// Then let's AND AAh and 55h. This should result in the result 00h and a set Z flag.
		w_reg_bus = 8'hAA;
		alu_mux_bus = 8'h55;

		#5;
	
	end
	endtask
	
	task TestCrlf(); // Tested OK 1/23/07.
	begin
		
		$display("TestCrlf");
		
		// Set up the instruction word. Note that we clear out the source and destination 
		// register fields in the instruction word since that information is used by the 
		// controller to set up the datapath. The ALU doesn't care where the data is 
		// coming from or where it is going.
		ir_reg_bus = 12'b0000_0110_0000;
	
		// This ALU op does not take any operands. It simply drives 00h on alu_bus and sets the Z flag.

		#5;
	
	end
	endtask	
	
	task TestClrw(); // Tested OK 1/23/07.
	begin
		
		$display("TestClrw");
		
		// Set up the instruction word. Note that we clear out the source and destination 
		// register fields in the instruction word since that information is used by the 
		// controller to set up the datapath. The ALU doesn't care where the data is 
		// coming from or where it is going.
		ir_reg_bus = 12'b0000_0100_0000;
	
		// This ALU op does not take any operands. It simply drives 00h on alu_bus and sets the Z flag.

		#5;
	
	end
	endtask		

	task TestComf(); // Tested OK 1/23/07.
	begin
		
		$display("TestComf");
		
		// Set up the instruction word. Note that we clear out the source and destination 
		// register fields in the instruction word since that information is used by the 
		// controller to set up the datapath. The ALU doesn't care where the data is 
		// coming from or where it is going.
		ir_reg_bus = 12'b0010_0100_0000;
	
		// The ALU should output the 1's complement of the alu_mux_bus input.
		// The 'W' input should not affect the output. The Z flag should not be set.
		w_reg_bus = 8'hFF;
		alu_mux_bus = 8'h55;
		
		#5;
		
		// Let's cause the ALU output to be 00h. This should set the Z flag.
		w_reg_bus = 8'hAA;
		alu_mux_bus = 8'hFF;

		#5;
	
	end
	endtask

	task TestDecf(); // Tested OK 1/23/07.
	begin
		
		$display("TestDecf");
		
		// Set up the instruction word. Note that we clear out the source and destination 
		// register fields in the instruction word since that information is used by the 
		// controller to set up the datapath. The ALU doesn't care where the data is 
		// coming from or where it is going.
		ir_reg_bus = 12'b0000_1100_0000;
	
		// The ALU should output alu_mux_bus - 1 = 54.
		// The 'W' input should not affect the output. The Z flag should not be set.
		w_reg_bus = 8'hAA;
		alu_mux_bus = 8'h55;
		
		#5;
		
		// Let's cause the ALU output to be 00h. This should set the Z flag.
		w_reg_bus = 8'hDD;
		alu_mux_bus = 8'h01;

		#5;
	
	end
	endtask

	task TestIncf(); // Tested OK 1/23/07.
	begin
		
		$display("TestIncf");
		
		// Set up the instruction word. Note that we clear out the source and destination 
		// register fields in the instruction word since that information is used by the 
		// controller to set up the datapath. The ALU doesn't care where the data is 
		// coming from or where it is going.
		ir_reg_bus = 12'b0010_1000_0000;
	
		// The ALU should output alu_mux_bus + 1 = 56.
		// The 'W' input should not affect the output. The Z flag should not be set.
		w_reg_bus = 8'hAA;
		alu_mux_bus = 8'h55;
		
		#5;
		
		// Let's cause the ALU output to be 00h. This should set the Z flag.
		w_reg_bus = 8'hDD;
		alu_mux_bus = 8'hFF;

		#5;
	
	end
	endtask
	
	task TestIorwf(); // Tested OK 1/23/07.
	begin
		
		$display("TestIorwf");
		
		// Set up the instruction word. Note that we clear out the source and destination 
		// register fields in the instruction word since that information is used by the 
		// controller to set up the datapath. The ALU doesn't care where the data is 
		// coming from or where it is going.
		ir_reg_bus = 12'b0001_0000_0000;
	
		// The ALU should output w_reg_bus OR alu_mux_bus = FFh.
		// The Z flag should not be set.
		w_reg_bus = 8'hAA;
		alu_mux_bus = 8'h55;
		
		#5;
		
		// Let's cause the ALU output to be 00h. This should set the Z flag.
		w_reg_bus = 8'h00;
		alu_mux_bus = 8'h00;

		#5;
	
	end
	endtask	

	task TestMovf(); // Tested OK 1/23/07.
	begin
		
		$display("TestMovf");
		
		// Set up the instruction word. Note that we clear out the source and destination 
		// register fields in the instruction word since that information is used by the 
		// controller to set up the datapath. The ALU doesn't care where the data is 
		// coming from or where it is going.
		ir_reg_bus = 12'b0010_0000_0000;
	
		// This ALU operation simply passes through the 'f' operand 
		// and sets the Z flag if the 'f' operand is 00h. The 'W'
		// operand should not affect the output.
		w_reg_bus = 8'hAA;
		alu_mux_bus = 8'h11;
		
		#5;

		// Let's cause the ALU output to be 00h. This should set the Z flag.
		w_reg_bus = 8'hBB;
		alu_mux_bus = 8'h00;

		#5;
	
	end
	endtask		

	task TestMovwf(); // Tested OK 1/24/07.
	begin
	    
	    $display("TestMovwf");
	    
		// Set up the instruction word. Note that we clear out the source and destination 
		// register fields in the instruction word since that information is used by the 
		// controller to set up the datapath. The ALU doesn't care where the data is 
		// coming from or where it is going.
		ir_reg_bus = 12'b0000_0010_0000;
	
		// This ALU operation simply passes through the 'W' operand.
		// The 'f' operand should not affect the output.
		// No flags are updated by this ALU Operation.
		w_reg_bus = 8'h55;
		alu_mux_bus = 8'h33;

		#5;
	
	end
	endtask		

	task TestRlf(); // Tested OK 1/24/07.
	begin
		
		$display("TestRlf");
		
		// Set up the instruction word. Note that we clear out the source and destination 
		// register fields in the instruction word since that information is used by the 
		// controller to set up the datapath. The ALU doesn't care where the data is 
		// coming from or where it is going.
		ir_reg_bus = 12'b0011_0100_0000;
		
		// This ALU operation rotates left 'f' through the carry flag.
		// No other flags than C are affected by this instruction.
		alu_mux_bus = 8'hAA; // This should set C
	
		#5;
		
		alu_mux_bus = 8'h55; // This should clear C
					
		#5;	
		
		// This should shift in a '1' in the least significant bit of the output.
		carry_bit = 1;
	
		#5;	
	end
	endtask

	task TestRrf(); // Tested OK 1/24/07.
	begin
		
		$display("TestRrf");
		
		// Set up the instruction word. Note that we clear out the source and destination 
		// register fields in the instruction word since that information is used by the 
		// controller to set up the datapath. The ALU doesn't care where the data is 
		// coming from or where it is going.
		ir_reg_bus = 12'b0011_0000_0000;
		
		// This ALU operation rotates right 'f' through the carry flag.
		// No other flags than C are affected by this instruction.
		alu_mux_bus = 8'hAA;
	
		#5;
		
		alu_mux_bus = 8'h55;
					
		#5;	
		
		// This should shift in a '1' in the least significant bit of the output.
		carry_bit = 0;
	
		#5;	
	end
	endtask

	task TestSubwf(); // Tested OK 1/24/07.
	begin
		
		$display("TestSubwf");
		
		// Set up the instruction word. Note that we clear out the source and destination 
		// register fields in the instruction word since that information is used by the 
		// controller to set up the datapath. The ALU doesn't care where the data is 
		// coming from or where it is going.
		ir_reg_bus = 12'b0000_1000_0000;
		
		// This ALU operation Subtracts W from f.
		// All flags are affected by this instruction.
		
		// The Carry flag (C) is cleared if borrow occured.
		// Borrow occurs (C=0) if the 2nd operand is larger than the 1st operand.
		// More conveniently, Borrow does not occur (C=1) if the 1st op >= 2nd op.
			
		// These operands will set both the C and DC flags
		// because both f > W and f[3:0] > W[3:0].
		alu_mux_bus = 8'hff; // 'f'
		w_reg_bus = 8'h11;   // 'W'
			
		#5;

		// These operands will clear the DC flag
		// because borrow occured in the lower nybble
		// (W[3:0] > f[3:0]).
		alu_mux_bus = 8'hf0; // 'f'
		w_reg_bus = 8'h15;   // 'W'
			
		#5;

		// These operands will clear both the C and DC flags
		// because both W > f and W[3:0] > f[3:0]. I.e.
		// borrow occured both in the overall byte as well as 
		// in the lower nybble.
		alu_mux_bus = 8'h11; // 'f'
		w_reg_bus = 8'hff;   // 'W'
			
		#5;

		// These operands will set the Z flag because the result is 0.
		alu_mux_bus = 8'h33; // 'f'
		w_reg_bus = 8'h33;   // 'W'
			
		#5;
	end
	endtask

	task TestSwapf(); // Tested OK 1/24/07.
	begin
		
		$display("TestSwapf");
		
		// Set up the instruction word. Note that we clear out the source and destination 
		// register fields in the instruction word since that information is used by the 
		// controller to set up the datapath. The ALU doesn't care where the data is 
		// coming from or where it is going.
		ir_reg_bus = 12'b0011_1000_0000;
	
		// The ALU should swap the two nybbles and output 5Ah.
		// The 'W' input should not affect the output. 
		// No flags should be affected.
		w_reg_bus = 8'hAC;
		alu_mux_bus = 8'hA5;
		
		#5;
	
	end
	endtask
	
	task TestXorwf(); // Tested OK 1/24/07.
	begin
		
		$display("TestXorwf");
		
		// Set up the instruction word. Note that we clear out the source and destination 
		// register fields in the instruction word since that information is used by the 
		// controller to set up the datapath. The ALU doesn't care where the data is 
		// coming from or where it is going.
		ir_reg_bus = 12'b0001_1000_0000;
	
		// The ALU should XOR 'W' and 'f'.
		// Only the Z flag is affected.
		
		// This should result in 00h, Z flag set.
		w_reg_bus = 8'h55;
		alu_mux_bus = 8'h55;
		
		#5;

		// This should result in AAh, Z flag cleared.
		w_reg_bus = 8'hFF;
		alu_mux_bus = 8'h55;
		
		#5;
	
	end
	endtask	
		
	task TestBcf(); // Tested OK 1/24/07.
	begin
		
		$display("TestBcf");
		
		// Set up the instruction word. Note that we clear out the source and destination 
		// register fields in the instruction word since that information is used by the 
		// controller to set up the datapath. The ALU doesn't care where the data is 
		// coming from or where it is going. We DO specify which bit to be cleared in the
		// ALU output (Instruction format: 0100 bbbf ffff, 'bbb' indicates the bit to clear).
		ir_reg_bus = 12'b0100_0100_0000; // This clears bit 2.
	
		// This should result in FDh, no flags are affected by this ALU op.
		w_reg_bus = 8'h55;
		alu_mux_bus = 8'hFF;
		
		#5;

		// This should clear bit 7.
      ir_reg_bus = 12'b0100_1110_0000;
		
		#5;
	
	end
	endtask			
		
	task TestBsf(); // Tested OK 1/24/07.
	begin
		
		$display("TestBsf");
		
		// Set up the instruction word. Note that we clear out the source and destination 
		// register fields in the instruction word since that information is used by the 
		// controller to set up the datapath. The ALU doesn't care where the data is 
		// coming from or where it is going. We DO specify which bit to be set in the
		// ALU output (Instruction format: 0101 bbbf ffff, 'bbb' indicates the bit to set).
		ir_reg_bus = 12'b0101_0100_0000; // This sets bit 2.
	
		// This should result in 04h, no flags are affected by this ALU op.
		w_reg_bus = 8'h55;
		alu_mux_bus = 8'h00;
		
		#5;

		// This should set bit 7.
      ir_reg_bus = 12'b0101_1110_0000;
		
		#5;
	
	end
	endtask					
		
	task TestBittest(); // Tested OK 1/24/07.
	begin
		
		$display("TestBittest");
		   
		// Set up the instruction word. Note that we clear out the source and destination 
		// register fields in the instruction word since that information is used by the 
		// controller to set up the datapath. The ALU doesn't care where the data is 
		// coming from or where it is going. We DO specify which bit to be tested in the
		// 'f' input (Instruction format: 0110 bbbf ffff, 'bbb' indicates the bit to test).
		ir_reg_bus = 12'b0110_0100_0000; // This tests bit 2.
	
	   // NOTE: This ALU operation does not update any status register flags. The ALU
	   // however DOES use the Z-bit in the status_bus to notify the controller if
	   // the result is zero. Since the ALU never sets the load_z signal the
	   // Z status_bus flag will never be loaded into the STATUS register.
	
		// This should result in Z=0 (bit 2 is non-zero).
		// The 'W' operand should not be used.
		w_reg_bus = 8'hAA;
		alu_mux_bus = 8'b00000100;
		
		#5;

		// This should result in Z=1 (bit 2 is zero).
      alu_mux_bus = 8'b11111011;
		
		#5;
	
	end
	endtask				
		
	task TestAndlw(); // Tested OK 1/25/07.
	begin

      $display("TestAndlw");
      
		// Set up the instruction word. Note that we specify the literal to be AND'ed 
		// with the 'W' register (Instruction format: 1110 kkkk kkkk, where 'kkkk kkkk' 
		// indicates the binary constant to AND the 'W' register with).
		ir_reg_bus = 12'b1110_0101_0101; // AND 'W' with 55h.

		// This should result in Z=0 (result is non-zero).
		w_reg_bus = 8'h0F; // The result should be 05h
		alu_mux_bus = 8'h00; // The 'f' operand should not be used.
		
		#5;

		// This should result in Z=0 (result is non-zero).
		w_reg_bus = 8'hF0; // The result should be 50h
		
		#5;

		// This should result in Z=1 (result is zero).
		w_reg_bus = 8'hAA; // The result should be 00h
		
		#5;
	
	end
	endtask			
		
	task TestCall(); // Tested OK 1/25/07.
	begin

      $display("TestCall");
      
		// Set up the instruction word. Note that we specify the 8-bit call address in 
		// the instruction word (Instruction format: 1001 kkkk kkkk, where 'kkkk kkkk' 
		// indicates the binary call address). The ALU simply passes through the literal.
		ir_reg_bus = 12'b1001_0110_0110; // Call address 66h. No flags are affected.

		#5;
	
	end
	endtask		

	task TestIorlw(); // Tested OK 1/25/07.
	begin

      $display("TestIorlw");
      
		// Set up the instruction word. Note that we specify the literal to be OR'ed 
		// with the 'W' register (Instruction format: 1101 kkkk kkkk, where 'kkkk kkkk' 
		// indicates the binary constant to OR the 'W' register with).
		ir_reg_bus = 12'b1101_0101_0101; // OR 'W' with 55h.

		// This should result in Z=0 (result is non-zero).
		w_reg_bus = 8'hAA; // The result should be FFh
		alu_mux_bus = 8'h44; // The 'f' operand should not be used.
		
		#5;

		// This should result in Z=0 (result is zero).
		w_reg_bus = 8'h00; // The result should be 55h
		
		#5;
	
	end
	endtask		
		
	task TestMovlw(); // Tested OK 1/25/07.
	begin
      
      $display("TestMovlw");
      
		// Set up the instruction word. Note that we specify the literal to be passed through
		// by the ALU in the instruction word (Instruction format: 1100 kkkk kkkk, where 'kkkk kkkk' 
		// indicates the binary constant the ALU will pass through to its output).
		ir_reg_bus = 12'b1100_1001_1001; // Pass through 99h.

		// No flags should be affected by this ALU operation.
		w_reg_bus = 8'hBB;		// The 'W' operand should not be used.
		alu_mux_bus = 8'hCC;	// The 'f' operand should not be used.
		
		#5;
	
		ir_reg_bus = 12'b1100_0110_0110; // Pass through 66h.

	   #5;
	   
	end
	endtask			
		
	task TestOption(); // Tested OK 1/25/07.
	begin
      
      $display("TestOption");
      
		// Set up the instruction word. 
		ir_reg_bus = 12'b0000_0000_0010;

		// The ALU should output the 'W' operand. No flags should be affected.
		w_reg_bus = 8'h55;
		alu_mux_bus = 8'hAA;	// The 'f' operand should not be used.
		
		#5;
	   
	end
	endtask	
		
	task TestRetlw(); // Tested OK 1/25/07.
	begin

      $display("TestRetlw");
      
		// Set up the instruction word. Note that we specify the literal to be passed through
		// by the ALU in the instruction word (Instruction format: 1000 kkkk kkkk, where 'kkkk kkkk' 
		// indicates the binary constant the ALU will pass through to its output).
		ir_reg_bus = 12'b1000_1110_1110; // Pass through EEh.

		// No flags should be affected by this ALU operation.
		w_reg_bus = 8'h33;		// The 'W' operand should not be used.
		alu_mux_bus = 8'h77;	// The 'f' operand should not be used.
		
		#5;
	
		ir_reg_bus = 12'b1000_1100_1100; // Pass through CCh.

	   #5;
	   
	end
	endtask	
	
	task TestTris(); // Tested OK 1/25/07.
	begin

      $display("TestTris");
      
	   // Set up the instruction word. Note that the literal in the instruction word specifies
		// by the ALU in the instruction word (Instruction format: 0000 0000 0ffff, where 'fff' 
		// indicates the index of the TRIS register to load: 5, 6 or 7). NOTE that the op-code
		// is the same as for the OPTION instruction but since fff is always non-zero the
		// instruction decoder will always be able to tell the difference between TRIS and OPTION.
		ir_reg_bus = 12'b0000_0000_0111; // Load TRIS register 7.

		// This ALU op simply passes through the 'W' register.
		// No flags should be affected by this ALU operation.
		w_reg_bus = 8'h77;
		
		#5;
	
		w_reg_bus = 8'h88; // Pass through 88h.

	   #5;
	   
	end
	endtask

	task TestXorlw(); // Tested OK 1/25/07.
	begin
      
      $display("TestXorlw");
      
		// Set up the instruction word. Note that we specify the literal to be XOR'ed 
		// with the 'W' register (Instruction format: 1111 kkkk kkkk, where 'kkkk kkkk' 
		// indicates the binary constant to XOR the 'W' register with).
		ir_reg_bus = 12'b1111_0101_0101; // XOR 'W' with 55h.

		// This should result in Z=0 (result is non-zero).
		w_reg_bus = 8'hAA; // The result should be FFh
		alu_mux_bus = 8'h44; // The 'f' operand should not be used.
		
		#5;

		// This should result in Z=0 (result is zero).
		w_reg_bus = 8'h55; // The result should be 00h
		
		#5;
	
	end
	endtask		
							
endmodule

// NOTE: VERIFIED OK.
module test_pic10_fsr;

	// These are the output signals from the Device Under Test:
	wire [7:0] fsr_reg_bus;	// Output: Contains the indirect register address in the FSR register.
	
	// These are the input signals to the Device Under Test:
	reg [7:0] alu_bus;		// Input: The output from the ALU. Loaded into the FSR register when 
							// the load_fsr_reg signals is asserted at the next posedge(clk).
	reg load_fsr_reg;		// Input: Loads alu_bus into FSR on the next posedge(clk).
	reg reset;				// Input: System reset.
	reg clk;				// Input: System clock.

	// This is the Device Under Test.
	pic10_fsr fsr(
		fsr_reg_bus,		// Output: Contains the indirect register address in the FSR register.
		alu_bus,			// Input: The output from the ALU. Loaded into the FSR register when 
							// the load_fsr_reg signals is asserted at the next posedge(clk).
		load_fsr_reg,		// Input: Loads alu_bus into FSR on the next posedge(clk).
		reset,				// Input: System reset.
		clk);				// Input: System clock.

	// Generate the clock signal.
	initial clk = 0;
	always #1 clk = ~clk;
	
	// Assert reset from t=5 to t=10. The 'fsr_reg_bus' should 
	// output 00h from the next positive edge of clk.
	initial 
	begin
		reset <= #5 1;
		reset <= #10 0;
	end	

	// Load a known value into the register at t=20.
	// The known value should be output on 'fsr_reg_bus'
	// at the next posedge(clk).
	initial 
	begin
		alu_bus <= #20 8'hAA;
		load_fsr_reg <= #20 1;
		load_fsr_reg <= #22 0;
	end	
	
endmodule

// NOTE: VERIFIED OK.
module test_pic10_ir;

	// These are the output signals from the Device Under Test:
	wire [11:0] ir_reg_bus;	// Output: Contains the instruction word in the Instruction Register.
	
	// These are the input signals to the Device Under Test:
	reg [11:0] program_mux_bus;	// Input: Carries the program word to load into the Instruction Register
							// (IR) when 'load_ir_reg' is asserted.					
	reg load_ir_reg;		// Input: Loads program_mux_bus into IR on the next posedge(clk).
	reg reset;				// Input: System reset.
	reg clk;				// Input: System clock.

	// This is the Device Under Test.
	pic10_ir ir(
		ir_reg_bus,			// Output: Contains the instruction word in the Instruction Register.
		program_mux_bus,	// Input: Carries the program word to load into the Instruction Register
							// (IR) when 'load_ir_reg' is asserted.					
		load_ir_reg,		// Input: Loads program_mux_bus into IR on the next posedge(clk).
		reset,				// Input: System reset.
		clk);				// Input: System clock.

	// Generate the clock signal.
	initial clk = 0;
	always #1 clk = ~clk;
	
	// Assert reset from t=5 to t=10. The 'ir_reg_bus' should 
	// output 000h from the next positive edge of clk.
	initial 
	begin
		reset <= #5 1;
		reset <= #10 0;
	end	

	// Load a known value into the register at t=20.
	// The known value should be output on 'ir_reg_bus'
	// at the next posedge(clk).
	initial 
	begin
		program_mux_bus <= #20 12'hABC;
		load_ir_reg <= #20 1;
		load_ir_reg <= #22 0;
	end	
	
endmodule

// NOTE: VERIFIED OK.
module test_pic10_pc;

	// These are the output signals from the Device Under Test:
	wire [8:0] pc_bus;		// Output: Drives the current value in the Program Counter.
	
	// These are the input signals to the Device Under Test:
	reg [8:0] pc_mux_bus=0;	// Input: Data to be loaded into PC when 'load_pc_reg' is asserted. 	
	reg load_pc_reg = 0;	// Input: Loads the PC from the input bus currently selected via pc_mux_sel.
	reg inc_pc = 0;			// Input: Increments the program counter.
	reg reset = 0;			// Input: System reset.
	reg clk = 0;			// Input: System clock.

	// This is the Device Under Test.
	pic10_pc pc(
		pc_bus,				// Output: Drives the current value in the Program Counter.
		pc_mux_bus,			// Input: Data to be loaded into PC when 'load_pc_reg' is asserted. 	
		load_pc_reg,		// Input: Loads the PC from the input bus currently selected via pc_mux_sel.
		inc_pc,				// Input: Increments the program counter.
		reset,				// Input: System reset.
		clk);				// Input: System clock.

	// Generate the clock signal.
	always #1 clk = ~clk;
	
	// Assert reset from t=5 to t=10. The 'pc_bus' should 
	// output 9'b0 from the next positive edge of clk.
	initial 
	begin
		reset <= #5 1;
		reset <= #10 0;
	end	

	// Load a known value into the register at t=20.
	// The known value should be output on 'pc_bus'
	// at the next posedge(clk).
	initial 
	begin
		pc_mux_bus <= #20 9'h123;
		load_pc_reg <= #20 1;
		load_pc_reg <= #22 0;
	end	
	
	// Increment the PC at t=30.
	initial 
	begin
		inc_pc <= #30 1;
		inc_pc <= #32 0;
	end	
		
endmodule

// NOTE: VERIFIED OK.
module test_pic10_pc_mux;

	// These are the output signals from the Device Under Test:
	wire [8:0] pc_mux_bus;	// Output: The PC will be loaded from this bus.

	// These are the input signals to the Device Under Test:
	reg [8:0] stack_bus = 9'h111;	// Input: Used to load stacked values into PC.
	reg [7:0] alu_bus = 8'h22;		// Input: Used to load register values into PC.
	reg [8:0] ir_reg_bus = 9'h133;	// Input: Used to load literal values (constants in instructions) into PC.
	reg [1:0] pc_mux_sel;			// Input: allows the PC to be loaded with data from the alu_bus, 

	// This is the Device Under Test.
	pic10_pc_mux pc_mux(
		pc_mux_bus,			// Output: The PC will be loaded from this bus.
		stack_bus,			// Input: Used to load stacked values into PC.
		alu_bus,			// Input: Used to load register values into PC.
		ir_reg_bus,			// Input: Used to load literal values (constants in instructions) into PC.
		pc_mux_sel);		// Input: allows the PC to be loaded with data from the alu_bus, 
								// ir_reg_bus or internal stack buses.

	// Increment the mux select input every 10 time units.
	initial pc_mux_sel = 0;
	always #10 pc_mux_sel = pc_mux_sel + 1;

endmodule

// NOTE: VERIFIED OK.
module test_pic10_stack;

	// These are the output signals from the Device Under Test:
	wire [8:0] stack_bus;	// Output: Current stack location data.

	// These are the input signals to the Device Under Test:
	reg [8:0] pc_bus = 0;	// Input: Data to be loaded when asserting load_stack.
	reg load_stack = 0;		// Input: Loads the stack from the current PC value.
	reg inc_stack = 0;		// Input: Increments the current stack location.
	reg dec_stack = 0;		// Input: Decrements the current stack location.
	reg reset = 0;			// Input: System reset.
	reg clk = 0;			// Input: System clock.

	// This is the Device Under Test.
	pic10_stack stack(
		stack_bus,			// Output: Current stack location data.
		pc_bus,				// Input: Data to be loaded when asserting load_stack.
		load_stack,			// Input: Loads the stack from the current PC value.
		inc_stack,			// Input: Increments the current stack location.
		dec_stack,			// Input: Decrements the current stack location.
		reset,				// Input: System reset.
		clk);				// Input: System clock.
							// ir_reg_bus or internal stack buses.

	// Generate the clock signal.
	always #1 clk = ~clk;

	// Assert reset from t=5 to t=10. The 'stack_bus' should 
	// have an UNDEFINED value because only the stack location
	// is reset, the stack content is NOT reset.
	initial 
	begin
		reset <= #5 1;
		reset <= #10 0;
	end	

	// Load the stack location (0) with data at t=20. The 'stack_bus' should 
	// output the same data on the next rising edge of clk.
	initial 
	begin
		pc_bus <= #20 9'h123;
		load_stack <= #20 1;
		load_stack <= #22 0;
	end	
	
	// Increment the stack location at t=30. The 'stack_bus' output should once 
	// again be undefined since this stack location (1) has not yet been loaded.
	initial 
	begin
		inc_stack <= #30 1;
		inc_stack <= #32 0;
	end	
	
	// Load the stack location (1) with data at t=40. The 'stack_bus' should 
	// output the same data on the next rising edge of clk.
	initial 
	begin
		pc_bus <= #40 9'h145;
		load_stack <= #40 1;
		load_stack <= #42 0;
	end	

	// Decrement the stack at t=50. The data loaded into this stack 
	// location (0) should become available on the 'stack_bus' output.
	initial 
	begin
		dec_stack <= #50 1;
		dec_stack <= #52 0;
	end	

endmodule

// NOTE: VERIFIED OK.
module test_pic10_pc_datapath;

	// These are the output signals from the Device Under Test:
	wire [8:0] pc_bus;		// Output: Drives the current value in the Program Counter.

	// These are the input signals to the Device Under Test:
	reg [7:0] alu_bus=0;		// Input: Used to load register values into PC.
	reg [8:0] ir_reg_bus=0;	// Input: Used to load literal values (constants in instructions) into PC.
	reg [1:0] pc_mux_sel=0;	// Input: Allows the PC to be loaded with data from the internal stack bus (0),
							// alu_bus (1) or ir_reg_bus (2).
	reg load_pc_reg=0;		// Input: Loads the PC from the input bus currently selected via pc_mux_sel.
	reg inc_pc=0;				// Input: Increments the program counter.
	reg inc_stack=0;			// Input: Increments the stack pointer.
	reg dec_stack=0;			// Input: Decrements the stack pointer.
	reg load_stack=0;			// Input: Loads the stack from the current PC value.
	reg reset=0;				// System reset.
	reg clk=0;				// System clock.

	// This is the Device Under Test.
	pic10_pc_datapath pc_datapath(
		pc_bus,				// Output: Drives the current value in the Program Counter.
		alu_bus,			// Input: Used to load register values into PC.
		ir_reg_bus,			// Input: Used to load literal values (constants in instructions) into PC.
		pc_mux_sel,			// Input: Allows the PC to be loaded with data from the internal stack bus (0),
							// alu_bus (1) or ir_reg_bus (2).
		load_pc_reg,		// Input: Loads the PC from the input bus currently selected via pc_mux_sel.
		inc_pc,				// Input: Increments the program counter.
		inc_stack,			// Input: Increments the stack pointer.
		dec_stack,			// Input: Decrements the stack pointer.
		load_stack,			// Input: Loads the stack from the current PC value.
		reset,				// System reset.
		clk);				// System clock.

	// Generate the clock signal.
	always #1 clk = ~clk;

	// Assert reset from t=5 to t=10. The 'pc_bus' should 
	// become 9'h000 at the next posedge(clk).
	initial 
	begin
		reset <= #5 1;
		reset <= #10 0;
	end	

	// Load a known value into PC at t=20 (simulate an instruction fetch load).
	// The 'pc_bus' output should become 9'h123 at the next posedge(clk).
	initial 
	begin
		ir_reg_bus <= #20 9'h123;
		pc_mux_sel <= #20 2;	// 2 = ir_reg_bus.
		load_pc_reg <= #20 1;
		load_pc_reg <= #22 0;
	end	
	
	// Push PC onto the stack at t=30 (stack position 0). NOTE that we first 
	// increment the stack location and then load the stack. This simulates 
	// the stack-related operations of a CALL instruction. The 'pc_bus' 
	// output should be unchanged (9'h0AA).
	initial 
	begin
		inc_stack <= #30 1;
		inc_stack <= #32 0;
		
		load_stack <= #32 1;
		load_stack <= #34 0;
	end	
	
	// Load a known value into PC via the 'alu_bus' at t=40 (simulates the 
	// PC-related operations of the CALL instruction). The 'pc_bus' output 
	// should change to 9'h0AA.
	initial 
	begin
		alu_bus <= #40 8'hAA;
		pc_mux_sel <= #40 1;	// 1 = alu_bus.
		load_pc_reg <= #40 1;
		load_pc_reg <= #42 0;
	end	

	// Push PC onto the stack at t=50 (stack position 1). NOTE that we first 
	// increment the stack location and then load the stack. This simulates 
	// the stack-related operations of a CALL instruction. The 'pc_bus' 
	// output should be unchanged (9'h0AA).
	initial 
	begin
		inc_stack <= #50 1;
		inc_stack <= #52 0;
	
		load_stack <= #52 1;
		load_stack <= #54 0;
	end	
	
	// Load a known value into PC via the 'alu_bus' at t=60 (simulates the 
	// PC-related operations of the CALL instruction). The 'pc_bus' output 
	// should change to 9'h055.
	initial 
	begin
		ir_reg_bus <= #60 9'h055;
		pc_mux_sel <= #60 2;	// 2 = ir_reg_bus.
		load_pc_reg <= #60 1;
		load_pc_reg <= #62 0;
	end	
	
	// Increment the PC at t=70. The 'pc_bus' output should change to 9'h056.
	initial 
	begin
		inc_pc <= #70 1;
		inc_pc <= #72 0;
	end	
	
	// Pop PC from stack location 1 at t=80. NOTE that we must first read out the value and then 
	// decrement the stack location. This simulates the RETLW instruction. 
	// The 'pc_bus' output should change back to 9'h0AA.
	initial 
	begin
		pc_mux_sel <= #80 0;	// 0 = stack_bus.
		load_pc_reg <= #80 1;
		load_pc_reg <= #82 0;

		dec_stack <= #82 1;
		dec_stack <= #84 0;
	end	
	 
	// Pop PC from stack location 0 at t=90. NOTE that we must first read out the value and then 
	// decrement the stack location. This simulates the RETLW instruction. 
	// The 'pc_bus' output should change back to 9'h123.
	initial 
	begin
		pc_mux_sel <= #90 0;	// 0 = stack_bus.
		load_pc_reg <= #90 1;
		load_pc_reg <= #92 0;

		dec_stack <= #92 1;
		dec_stack <= #94 0;
	end		

endmodule

// NOTE: VERIFIED OK.
module test_pic10_sfr_datapath;

	// These are the output signals from the Device Under Test:
	wire [7:0] fsr_reg_bus;		// Output: Contains the indirect register address in the FSR register.
								// Used by the controller to determine which load_XXX_reg signal to 
								// assert when writing to a SFR or general-purpose RAM register.
							
	wire [11:0] ir_reg_bus;		// Output: Contains the instruction word in the Instruction Register.
								// Used by the controller to determine which combinational signals
								// to assert to execute the instruction. Also used by the ALU which
								// needs to know the 'literal' constant operands (which some
								// instructions have encoded as part of the instruction).
								
	wire [7:0] ram_data_bus;	// Output: The output from the currently addressed general-purpose
								// RAM register. The address can either be a direct or indirect 
								// address (determined by the Register Address Mux).
				
	wire [2:0] reg_addr_bus;	// Output: This is the currently selected register address (either
								// a direct address	or an indirect address selected by the Register
								// Address Mux). Used as select signal for the SFR Data Mux. Note 
								// that a part-select of the reg_addr_bus is used. The SFR Data
								// Mux selects the SFR data to be used as the 2nd ALU operand.
								// NOTE that the 2nd ALU operand is further qualified by the ALU
								// Mux which selects between the output from the SFR Data Mux and 
								// the output from the General Purpose RAM Registers.

	// These are the input signals to the Device Under Test:
	reg [11:0] program_mux_bus = 0;	// Input: Carries the program word to load into the Instruction Register
								// (IR) when 'load_ir_reg' is asserted.
							
	reg [7:0] alu_bus = 0;		// Input: The output from the ALU. Loaded into a register when one of the
								// load_XXX_reg signals are asserted.					
							
	reg load_ir_reg = 0;		// Input: Loads program_mux_bus into IR on the next posedge(clk).
	reg load_fsr_reg = 0;		// Input: Loads alu_bus into FSR on the next posedge(clk).
	reg load_ram_reg = 0;		// Input: Loads alu_bus into a General-purpose RAM register with the address 
								// driven on reg_addr_bus. The data is clocked in on  next posedge(clk).
							
	reg reg_addr_mux_sel = 0;	// Input: Selects the register address to be used when reading or writing a
								// register. Can either be a direct address (0) or an indirect address (1).
								
	reg reset = 0;				// System reset.
	reg clk = 0;				// System clock.


	// This is the Device Under Test.
	pic10_sfr_datapath sfr_datapath(
		fsr_reg_bus,			// Output: Contains the indirect register address in the FSR register.
								// Used by the controller to determine which load_XXX_reg signal to 
								// assert when writing to a SFR or general-purpose RAM register.
								
		ir_reg_bus,				// Output: Contains the instruction word in the Instruction Register.
								// Used by the controller to determine which combinational signals
								// to assert to execute the instruction. Also used by the ALU which
								// needs to know the 'literal' constant operands (which some
								// instructions have encoded as part of the instruction).
									
		ram_data_bus,			// Output: The output from the currently addressed general-purpose
								// RAM register. The address can either be a direct or indirect 
								// address (determined by the Register Address Mux).
					
		reg_addr_bus,			// Output: This is the currently selected register address (either
								// a direct address	or an indirect address selected by the Register
								// Address Mux). Used as select signal for the SFR Data Mux. Note 
								// that a part-select of the reg_addr_bus is used. The SFR Data
								// Mux selects the SFR data to be used as the 2nd ALU operand.
								// NOTE that the 2nd ALU operand is further qualified by the ALU
								// Mux which selects between the output from the SFR Data Mux and 
								// the output from the General Purpose RAM Registers.
								
		program_mux_bus,		// Input: Carries the program word to load into the Instruction Register
								// (IR) when 'load_ir_reg' is asserted.
								
		alu_bus,				// Input: The output from the ALU. Loaded into a register when one of the
								// load_XXX_reg signals are asserted.					
								
		load_ir_reg,			// Input: Loads program_mux_bus into IR on the next posedge(clk).
		load_fsr_reg,			// Input: Loads alu_bus into FSR on the next posedge(clk).
		load_ram_reg,			// Input: Loads alu_bus into a General-purpose RAM register with the address 
								// driven on reg_addr_bus. The data is clocked in on  next posedge(clk).
								
		reg_addr_mux_sel,		// Input: Selects the register address to be used when reading or writing a
								// register. Can either be a direct address (0) or an indirect address (1).
									
		reset,					// System reset.
		clk);					// System clock.
		
	// Generate the clock signal.
	always #1 clk = ~clk;

	// Assert reset from t=5 to t=10. All bus outputs should become 0.
	initial 
	begin
		reset <= #5 1;
		reset <= #10 0;
	end	

	//////////////////////////////////////////////////////////////////////////
	// Test the IR, FSR and Register Address Mux bus and control signal routing.
	//////////////////////////////////////////////////////////////////////////
	
	// Load the IR at t=20. The 'ir_reg_bus' output should reflect the loaded value.
	initial 
	begin
		program_mux_bus <= #20 12'hABC;
		load_ir_reg <= #20 1;
		load_ir_reg <= #22 0;
	end		
	
	// Load the FSR at t=30. The 'fsr_reg_bus' output should reflect the loaded value.
	initial 
	begin
		alu_bus <= #30 12'hDEF;
		load_fsr_reg <= #30 1;
		load_fsr_reg <= #32 0;
	end		
	
	// Assert the 'reg_addr_mux_sel' at t=40. This will result in the current SFR and 
	// General-purpose register address to be taken from the FSR instead of from the IR.
	initial 
	begin
		reg_addr_mux_sel <= #40 1;
	end		

	//////////////////////////////////////////////////////////////////////////
	// Test the General-purpose register address and control signal routing.
	//////////////////////////////////////////////////////////////////////////
	
	// Load the FSR with a valid General-Purpose Register address (8..31) at t=50.
	initial 
	begin
		alu_bus <= #50 8'd10;
		load_fsr_reg <= #50 1;
		load_fsr_reg <= #52 0;
	end		
	
	// Load the addressed RAM register with data from the alu_bus at t=60.
	// The data loaded should be output at 'ram_reg_bus' as long as the
	// FSR and Register Address Mux control input are unchanged.
	initial 
	begin
		alu_bus <= #60 8'h55;
		load_ram_reg <= #60 1;
		load_ram_reg <= #62 0;
	end		
	
endmodule

// NOTE: VERIFIED OK.
module test_pic10_alu_datapath;

	// These are the output signals from the Device Under Test:
	wire [7:0] alu_bus;			// Output: The output from the ALU. This can either be
								// an unmodified, passed-through operand (W or register),
								// a literal (constant) from the instruction word or
								// the result from an arithmetic or logical operation
								// or some combination of the W reg, registers and a 
								// literal.
							
	wire [2:0] alu_status_bus;	// Output: Z, C and DC status bits. Driven by ALU when
								// an ALU operation affects any of the flags. The load_XXX
								// outputs will determine which of the alu_status_bus signals
								// are valid (and should be loaded into the status register).
								
	wire load_z;				// Output: The alu_status_bus.z bit is valid.
	wire load_c;				// Output: The alu_status_bus.c bit is valid.
	wire load_dc;				// Output: The alu_status_bus.dc bit is valid.

	// These are the input signals to the Device Under Test:
	reg [7:0] sfr_data_bus;		// Input: Operand data from the special function registers 0..7.
	reg [7:0] ram_data_bus;		// Input: Operand data from the general-purpose RAM registers.
	reg [11:0] ir_reg_bus;		// Input: This is the current instruction loaded into the 
								// instruction register. The ALU uses the 'literal' operand
								// fields in the instruction word.
								
	reg alu_mux_sel=0;			// Input: Selects the ALU's 2nd operand to be either the data on the 
								// sfr_data_bus (0) or the data on the ram_data_bus (1).
		
	reg load_w_reg=0;			// Input: Loads the data on the alu_bus into the W register.
	reg carry_bit=0;			// Input: The carry bit from the STATUS register.

	reg reset = 0;				// System reset.
	reg clk = 0;				// System clock.

	// This is the Device Under Test.
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
		
		reset,			// System reset.
		clk);				// System clock.

	////////////////////////////////////////////////////////////////////////////////////
	// Generate clock and reset.
	////////////////////////////////////////////////////////////////////////////////////
			
	// Generate the clock signal.
	always #1 clk = ~clk;

	// Assert reset from t=5 to t=10. All bus outputs should become 0.
	initial 
	begin
		reset <= #5 1;
		reset <= #10 0;
	end	
	
	////////////////////////////////////////////////////////////////////////////////////
	// Load the 'W' Register with an operand that we'll later Add with another operand.
	////////////////////////////////////////////////////////////////////////////////////
	
	// Apply the 'MOVF' Op code to the ir_reg_bus at t=20. This will cause the ALU to
	// pass through whatever data we pass in on the 2nd operand 'alu_mux_bus' bus.
	// NOTE that the five last 'f' bits in the instruction word is cleared out because
	// we manually activate the required source and destination signals (normally the
	// controller uses these bits to determine the source and destination registers).
	initial ir_reg_bus <= #20 12'b0010_0000_0000;

	// Drive the data to be loaded into the 'W' register onto the sfr_data_bus at t=20.
	// NOTE that the alu_mux_sel input is default 0 so the 2nd ALU operand will be
	// taken from the sfr_data_bus.
	initial sfr_data_bus <= #20 8'h22;
	
	// Load the 'W' Register at t=20.
	initial 
	begin
		load_w_reg <= #20 1;
		load_w_reg <= #22 0;
	end		
	
	////////////////////////////////////////////////////////////////////////////////////
	// Place the 2nd ALU operand on the sfr_data_bus at t=30.
	////////////////////////////////////////////////////////////////////////////////////
	
	initial sfr_data_bus <= #30 8'h44;
	
	////////////////////////////////////////////////////////////////////////////////////
	// Add the contents in the 'W' register with the content on 2nd ALU operand bus 
	// alu_mux_bus at t=40.
	////////////////////////////////////////////////////////////////////////////////////
	
	// Apply the 'ADDWF' Op Code to the ir_reg_bus at t=40. This will cause the ALU to
	// add the first and second operand, output the sum on alu_bus and update the status
	// flags. NOTE that the five last 'd' and 'f' bits in the instruction word are cleared 
	// out because we manually activate the required source and destination signals (normally 
	// the controller uses these bits to determine the source and destination registers).

	initial ir_reg_bus <= #40 12'b0001_1100_0000;
	
	// The ALU should now output the sum (66h) of the first operand (22h) and the 2nd
	// operand (44h). The C, DC and Z bits should all be cleared.
	
	////////////////////////////////////////////////////////////////////////////////////
	// Change the 2nd operand to 88h by placing the data onto the ram_data_bus and
	// then switching over the alu_mux by asserting the alu_mux_sel signal at t=50
	////////////////////////////////////////////////////////////////////////////////////
	
	initial 
	begin
		ram_data_bus <= #50 8'h88;
		alu_mux_sel <= #50 1;
	end		

	// The ALU should now output the sum (AAh) of the first operand (22h) and the 2nd
	// operand (88h). The C, DC and Z bits should all be cleared.
	
	////////////////////////////////////////////////////////////////////////////////////
	// Make sure that the carry-in bit works as intended at t=60
	////////////////////////////////////////////////////////////////////////////////////
	
	// Setting the carry-in bit should NOT affect the alu_bus content.
	initial carry_bit <= #60 1;

	////////////////////////////////////////////////////////////////////////////////////
	// Test the C and Z flags by changing the 2nd operand to DEh at t=70.
	// This should cause the alu_bus to contain 22h + DEh = 00.
	// All the flags should be set.
	////////////////////////////////////////////////////////////////////////////////////
	
	initial ram_data_bus <= #70 8'hDE;
	
endmodule

module test_pic10_datapath;

	////////////////////////////////////////////////////////////
	// These are the inout signals from the Device Under Test:
	////////////////////////////////////////////////////////////
	
	wire [7:0] gpio5_pin_bus;	// Inout: Port 5 pins connected to the outside world.
	wire [7:0] gpio6_pin_bus;	// Inout: Port 6 pins connected to the outside world.
	wire [7:0] gpio7_pin_bus;	// Inout: Port 7 pins connected to the outside world.

	// These are the output signals from the Device Under Test:
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
	
	////////////////////////////////////////////////////////////
	// These are the input signals to the Device Under Test:
	////////////////////////////////////////////////////////////
	
	reg load_status_reg = 0;	// Loads the alu_bus into STATUS register on next
								// posedge(clk).
								
	wire alu_mux_sel;			// Selects the 2nd operand to ALU to be 
								// sft_data_bus (0) or ram_data_bus(1).
								
	reg load_w_reg = 0;			// Loads alu_bus into W register on next 
								// posedge(clk).
								
	reg load_fsr_reg = 0;		// Loads alu_bus into FSR register on next 
								// posedge(clk). 
								
	reg load_ram_reg = 0;		// Loads alu_bus into the addressed general purpose
								// register on next posedge(clk). 
								
	wire reg_addr_mux_sel;		// Input: Selects the register address to be used when reading or writing a
								// register. Can either be a direct address (0) or an indirect address (1).
								
	reg load_tris5_reg = 0;		// Used to load the TRIS registers. A '1' in a
	reg load_tris6_reg = 0;		// bit position makes the corresponding bit in 
	reg load_tris7_reg = 0;		// the related GPIO register tri-stated (inputs).
								// The TRIS registers are reset as FFh (inputs).
								
	reg load_gpio5_reg = 0;		// Used to load the GPIO registers. Output data
	reg load_gpio6_reg = 0;		// is only driven out on the I/O pin if the
	reg load_gpio7_reg = 0;		// corresponding TRIS bit is '0' (output).

	reg inc_stack = 0;			// Increments or decrements the stack pointer on 
	reg dec_stack = 0;			// next posedge(clk). 
									
	reg load_stack = 0;			// Loads the current stack location with the
								// data on the pc_bus (used by the CALL instruction
								// to store the return address onto the stack).			
								
	reg load_pc_reg = 0;		// loads PC register from pc_mux_bus (used by
								// the CALL instruction to load the jump address
								// or used to directly load a new value into PCL
								// when PCL is used as the target register).
								
	reg inc_pc = 0;				// Increments the program counter (PC). Used
								// after instruction fetch as well as in 'skip'
								// instructions (DECFSZ, INCFSZ, BTFSC and BTFSS).
								
	reg [1:0] pc_mux_sel = 0;	// Chooses whether the load data to PC should come
								// from the stack_bus (0), alu_bus (1) or 
								// ir_reg_bus (2).
								
	reg load_ir_reg = 0;		// Loads IR with the contents of the program 
								// memory word currently being addressed by PC.
								
	wire [11:0] program_bus;	// Current instruction from the program memory at
								// the address output at pc_bus.

	reg skip_next_instruction = 0;
								
	reg reset = 1;				// System reset.
	reg clk = 0;				// System clock.

	////////////////////////////////////////////////////////////
	// This is debugging variables we use to monitor 
	// various signals in the waveform window.
	////////////////////////////////////////////////////////////

	reg [20*7:0] CurrentInstruction; 
	wire [20*7:0] NextInstruction; 

	////////////////////////////////////////////////////////////
	// This is the Device Under Test.
	////////////////////////////////////////////////////////////
	
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
	
	////////////////////////////////////////////////////////////////////////////////////
	// This is the memory from which the pic10_datapath will execute code
	// during this simulation.
	////////////////////////////////////////////////////////////////////////////////////
	
	// The encoded instructions are stored here.
	reg [11:0] ProgramStore [512];
	
	// The instructions are stored here in string format. This is so we can display the
	// current instructions and operands in the Wave window while debugging the code.
	// We here declare storage for 512 20-byte strings.
	reg [20*7:0]  ProgramStoreText [512];
	
	// Wire up the Program Store.
	assign program_bus = ProgramStore[pc_bus];
	
	// Wire up the current and next instruction debug strings.
	always @(posedge clk) begin
		if(load_ir_reg) CurrentInstruction = ProgramStoreText[pc_bus];
	end
	assign NextInstruction = ProgramStoreText[pc_bus];

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
	
	////////////////////////////////////////////////////////////////////////////////////
	// Generate clock and reset.
	////////////////////////////////////////////////////////////////////////////////////
			
	// Generate the clock signal.
	always #1 clk = ~clk;

	// Keep the datapath in reset until t=5. 
	initial 
	begin
		reset <= #5 0;
	end	

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
