// This is part of the ECE241 final project
// Date created: November 27 2016
// This file contains the module that outputs signals to clear the screen

module text_display_clear(clk, rst, do_clear, done_clear, x, y, colour);
    input clk;
    input rst;
    input do_clear;
    output done_clear;
    output reg [9:0]x;
    output reg [8:0]y;
    output colour;

    assign colour = 1'b0;

    always @ (posedge clk)
    begin
        if (rst)
            x <= 10'b0;
        else if (x == 10'd639)
            x <= 10'b0;
        else if (do_clear)
            x <= x + 1'b1;
    end

    always @ (posedge clk)
    begin
        if (rst)
            y <= 9'b0;
        else if ((y == 9'd479) && (x == 10'd639))
            y <= 9'b0;
        else if (do_clear && (x == 10'd639))
            y <= y + 1'b1;
    end

    assign done_clear = (do_clear&&(x == 10'd639)&&(y == 9'd479));

endmodule