// This is part of the ECE241 final project
// Date created: November 23 2016
// This file contains the cursor counter that save the current position for printing to the screen

module text_diaplay_cursor(clk, rst, inc_cursor, carriage_cursor, clear_screen, cursor_x, cursor_y);
    input clk;
    input rst;
    input inc_cursor;
    input carriage_cursor;

    output clear_screen;
    output reg [6:0]cursor_x;
    output reg [4:0]cursor_y;

    always @ (posedge clk)
    begin
        if (rst)
        begin
            cursor_x <= 7'b0;
            cursor_y <= 5'b0;
        end
        else if (inc_cursor)
        begin
            if (cursor_x < 7'd79)
                cursor_x <= cursor_x + 1'b1;
            else if ((cursor_x == 7'd79) && (cursor_y != 5'd29))
            begin
                cursor_x <= 7'b0;
                cursor_y <= cursor_y + 1'b1;
            end
            else if ((cursor_x == 7'd79) && (cursor_y == 5'd29))
            begin
                cursor_x <= 7'b0;
                cursor_y <= 5'b0;
            end
        end
        else if (carriage_cursor)
        begin
            if (cursor_y == 5'd29)
            begin
                cursor_x <= 7'b0;
                cursor_y <= 5'b0;
            end
            else
            begin
                cursor_x <= 7'b0;
                cursor_y <= cursor_y + 1'b1;
            end
        end
    end

    assign clear_screen = (inc_cursor && (cursor_x == 7'd79) && (cursor_y == 5'd29)) || (carriage_cursor && (cursor_y == 5'd29));
endmodule