// This is part of the ECE241 final project
// Date created: November 14 2016
// This file contains the input mux for the ALU, the inpu can be either data register or SFR(Special Function Registers)

module cpu_alu_input_mux(data_reg_in, sfr_in, alu_mux_select, mux_to_alu);
    input [7:0]data_reg_in;
    input [7:0]sfr_in;
    input alu_mux_select;
    output [7:0]mux_to_alu;

    assign mux_to_alu = alu_mux_select ? data_reg_in : sfr_in;

endmodule