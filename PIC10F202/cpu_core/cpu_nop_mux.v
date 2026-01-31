// This is part of the ECE241 final project
// Date created: November 14 2016
// This file contains the mux that can insert NOP instruction if the previous instruction require this to happen

module cpu_nop_mux(nop_insert, program_bus_in, program_bus_out);
    `include "definition.vh"

    input nop_insert;
    input [11:0]program_bus_in;
    output [11:0]program_bus_out;
    assign program_bus_out = nop_insert ? `NOP : program_bus_in;
endmodule