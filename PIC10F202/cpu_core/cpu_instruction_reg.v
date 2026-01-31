// This is part of the ECE241 final project
// Date created: November 12 2016
// This file contains the instruction register that have operation instructions from the program counter

module cpu_instruction_reg(clk, rst, load_instruction, program_to_reg, instruction_reg_out);
    input clk;
    input rst;
    input load_instruction;
    input [11:0]program_to_reg;
    output reg [11:0]instruction_reg_out;

    always @ (posedge clk)
    begin
        if (rst)
            instruction_reg_out <= 12'b0;
        else if (load_instruction)
            instruction_reg_out <= program_to_reg;
    end
endmodule