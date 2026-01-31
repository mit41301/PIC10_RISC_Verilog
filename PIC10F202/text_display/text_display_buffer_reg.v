// This is part of the ECE241 final project
// Date created: November 26 2016
// This file contains a buffer register to save the address for the font ROM

module text_display_buffer_reg(clk, rst, load_buff_reg, fifo_out, buff_reg_out);
    input clk;
    input rst;
    input load_buff_reg;
    input [6:0]fifo_out;
    output reg [6:0]buff_reg_out;

    always @ (posedge clk)
    begin
        if (rst)
            buff_reg_out <= 7'b0;
        else if (load_buff_reg)
            buff_reg_out <= fifo_out;
    end
endmodule