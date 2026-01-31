// This is part of the ECE241 final project
// Date created: November 12 2016
// This file contains the W accumulator of CPU which saves the result coming out of ALU and have bus to send back to ALU

module cpu_w_accumulator(clk, rst, load_w, alu_to_w, w_accumulator_out);

    input clk;
    input rst;
    input load_w;
    input [7:0]alu_to_w;
    output reg [7:0]w_accumulator_out;

    always @ (posedge clk)
    begin
        if (rst)
            w_accumulator_out <= 8'b0;
        else if (load_w)
            w_accumulator_out <= alu_to_w;
    end
endmodule