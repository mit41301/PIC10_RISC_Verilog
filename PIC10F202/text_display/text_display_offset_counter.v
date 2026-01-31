// This is part of the ECE241 final project
// Date created: November 25 2016
// This file contains the key counter that print 16*8 character onto the VGA

module text_display_offset_counter(clk, rst, display_char, done_char, c_x, c_y);
    input clk;
    input rst;
    input display_char;
    output done_char;
    output reg [2:0]c_x;
    output reg [3:0]c_y;

    always @ (posedge clk)
    begin
        if (rst)
            c_x <= 3'b0;
        else if (!display_char)
            c_x <= 3'b0;
        else if (c_x == 3'd7)
            c_x <= 3'b0;
        else if (display_char)
            c_x <= c_x + 1;
    end

    always @ (posedge clk)
    begin
        if (rst)
            c_y <= 4'b0;
        else if (!display_char)
            c_y <= 4'b0;
        else if ((c_y == 4'd15) && (c_x == 3'd7))
            c_y <= 4'b0;
        else if (display_char && (c_x == 3'd7))
            c_y <= c_y + 1;
    end

    assign done_char = (display_char&&(c_x == 3'd7)&&(c_y == 4'd15));

endmodule