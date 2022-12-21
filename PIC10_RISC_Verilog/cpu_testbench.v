/****************************************************************************
*
* By John.Gulbrandsen@SummitSoftConsulting.com, 1/17/2007.
*
* This file contains the top-level test bench for the 'pic10_cpu' module.
*
****************************************************************************/
module test_pic10_cpu;

	///////////////////////////////////////////////////////////////
	// These are the 'pic10' module inout ports.
	///////////////////////////////////////////////////////////////
	
	wire [7:0] gpio5_pin_bus;	// Inout: Port 5 pins connected to the outside world.
	wire [7:0] gpio6_pin_bus;	// Inout: Port 6 pins connected to the outside world.
	wire [7:0] gpio7_pin_bus;	// Inout: Port 7 pins connected to the outside world.

	///////////////////////////////////////////////////////////////
	// These are the 'pic10' module inputs.
	///////////////////////////////////////////////////////////////

	reg reset = 1;				// System reset.
	reg clk = 0;				// System clock.

   ////////////////////////////////////////////////////////////
   // Put pull-up resistors on all GPIO pins.
   ////////////////////////////////////////////////////////////

   pullup p5 [7:0] (gpio5_pin_bus);
   pullup p6 [7:0] (gpio6_pin_bus);
   pullup p7 [7:0] (gpio7_pin_bus);
   
	////////////////////////////////////////////////////////////
	// This is debugging variables we use to monitor 
	// various signals in the waveform window.
	////////////////////////////////////////////////////////////

	reg [20*7:0] CurrentInstruction; 
	wire [20*7:0] NextInstruction; 

	// Wire up the current and next instruction debug strings.
	always @(posedge clk) begin
		if(cpu.load_ir_reg) CurrentInstruction = cpu.program_store.ProgramStoreText[cpu.pc_bus];
	end
	
	assign NextInstruction = cpu.program_store.ProgramStoreText[cpu.pc_bus];


	////////////////////////////////////////////////////////////////////////////////////
	// Generate clock and reset.
	////////////////////////////////////////////////////////////////////////////////////
			
	// Generate the clock signal.
	always #1 clk = ~clk;

	// Keep the CPU in reset until t=5. 
	initial 
	begin
		reset <= #5 0;
	end	
	
	///////////////////////////////////////////////////////////////
	// This is the Device Under Test.
	///////////////////////////////////////////////////////////////

	pic10_cpu cpu(

		/////////////////////////// Inouts ////////////////////////////
	
		gpio5_pin_bus[7:0],		// Inout: Port 5 pins connected to the outside world.
		gpio6_pin_bus[7:0],		// Inout: Port 6 pins connected to the outside world.
		gpio7_pin_bus[7:0],		// Inout: Port 7 pins connected to the outside world.

		/////////////////////////// Inputs ////////////////////////////

		reset,					// System reset.
		clk						// System clock.

	);

endmodule