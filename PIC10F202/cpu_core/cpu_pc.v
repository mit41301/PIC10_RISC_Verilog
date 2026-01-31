// This is part of the ECE241 final project
// Date created: November 14 2016
// This file contains the program counter (that can go from 0 to 511)

module cpu_pc(clk, rst, pc_inc, load_pc, pc_input_from_mux, pc_out);

    input clk;
    input rst;
    input pc_inc;
    input load_pc;
    input [8:0]pc_input_from_mux;
    output reg [8:0]pc_out;

    always @ (posedge clk)
    begin
        if (rst)
            pc_out <= 9'b0;
        else if (load_pc)
            pc_out <= pc_input_from_mux;
        else if (pc_inc)
            pc_out <= pc_out + 1'b1;
    end
endmodule