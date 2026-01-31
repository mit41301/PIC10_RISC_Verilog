// This is part of the ECE241 final project
// Date created: November 17 2016
// This file contains the mux to select a direct address or an address that was saved in the FSR register as the address to access RAM

module cpu_reg_address_mux(reg_address_mux_select, instruction_reg_output, fsr_reg_output, reg_address_mux_out);

    input reg_address_mux_select;
    input [4:0]instruction_reg_output;
    input [4:0]fsr_reg_output;
    output [4:0]reg_address_mux_out;

    assign reg_address_mux_out = reg_address_mux_select ? fsr_reg_output : instruction_reg_output;

endmodule