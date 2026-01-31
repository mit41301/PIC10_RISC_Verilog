// This is part of the ECE241 final project
// Date created: November 14 2016
// This file contains the stack for the cpu

module cpu_stack(clk, rst, load_stack, dec_stack_ptr, inc_stack_ptr, pc_to_stack, stack_out);
    `include "definition.vh"

    input clk;
    input rst;
    input load_stack;
    input dec_stack_ptr;
    input inc_stack_ptr;
    input [8:0]pc_to_stack;
    output [8:0]stack_out;

    reg [8:0]stack[0:`STACK_DEPTH - 1];
    reg ptr; // This is a pointer used to find stack

    integer i;

    always @ (posedge clk)
    begin
        if (rst)
        begin
            ptr <= 1'b0;
            for (i = 0; i < `STACK_DEPTH; i = i + 1)
                stack[i] <= 9'b0;
        end
        else if (load_stack)
            stack[ptr] <= pc_to_stack;
        else if (dec_stack_ptr)
            ptr <= ptr - 1'b1;
        else if (inc_stack_ptr)
            ptr <= ptr + 1'b1;
    end

    assign stack_out = stack[ptr];

endmodule