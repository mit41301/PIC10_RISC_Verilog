// This is part of the ECE241 final project
// Date created: November 14 2016
// This file contains the FSR(File Select Register) which can provide inderect access to register locations

module cpu_fsr(clk, rst, alu_to_fsr, load_fsr, fsr_out);
    input clk;
    input rst;
    input [7:0]alu_to_fsr;
    input load_fsr;
    output reg [7:0]fsr_out;

    always @ (posedge clk)
    begin
        if (rst)
            fsr_out <= 8'b0;
        else if (load_fsr)
            fsr_out <= alu_to_fsr;
    end
endmodule