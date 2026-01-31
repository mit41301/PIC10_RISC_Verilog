// This is part of the ECE241 final project
// Date created: November 14 2016
// This file contains the mux for program counter which can select signal from stack, alu or instruction reg 

module cpu_pc_mux(pc_mux_select, stack_in, alu_in, instruction_in, pc_mux_out);
    input [1:0]pc_mux_select;
    input [8:0]stack_in;
    input [7:0]alu_in;
    input [8:0]instruction_in;
    output reg [8:0]pc_mux_out;
    
    always @ (*)
    begin
        case (pc_mux_select) 
          2'd0: pc_mux_out = stack_in;
          2'd1: pc_mux_out = {1'b0, alu_in};
          2'd2: pc_mux_out = instruction_in;
          default: pc_mux_out = 8'b0;
        endcase
    end
endmodule