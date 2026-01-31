// This is part of the ECE241 final project
// Date created: November 23 2016
// This file contains the components to print text from cpu to VGA

module text_display_datapath(clk, rst, data, rdreq, wrreq, load_buff_reg, inc_cursor, carriage_cursor, display_char, do_clear, plot, fifo_empty, new_line_ascii, clear_screen, done_char, done_clear, VGA_R, VGA_G, VGA_B, VGA_HS, VGA_VS, VGA_BLANK, VGA_SYNC, VGA_CLK);
    input clk;
    input rst;
    input [6:0]data;
    input rdreq;
    input wrreq;
    input load_buff_reg;
    input inc_cursor;
    input carriage_cursor;
    input display_char;
    input do_clear;
    input plot;

    output fifo_empty;
    output new_line_ascii;
    output clear_screen;
    output done_char;
    output done_clear;
    output [7:0]VGA_R;
    output [7:0]VGA_G;
    output [7:0]VGA_B;
    output VGA_HS;
    output VGA_VS;
    output VGA_BLANK;
    output VGA_SYNC;
    output VGA_CLK;

    wire [6:0]fifo_out;
    wire [6:0]buff_reg_out;
    wire [6:0]cursor_x;
    wire [4:0]cursor_y;
    wire [2:0]c_x;
    wire [3:0]c_y;
    wire [127:0]text_out;
    wire [9:0]x;
    wire [8:0]y;
    wire colour;
    wire [9:0]clear_x;
    wire [8:0]clear_y;
    wire clear_colour;

    wire [9:0]x_final;
    wire [8:0]y_final;
    wire colour_final;

    assign x_final = do_clear ? clear_x : x;
    assign y_final = do_clear ? clear_y : y;
    assign colour_final = do_clear ? clear_colour : colour;

    assign new_line_ascii = (buff_reg_out == 7'd10) ? 1'b1 : 1'b0;
    assign x = (cursor_x * 'd8) + c_x;
    assign y = (cursor_y * 'd16) + c_y;
    assign colour = text_out['d127 - ((c_y * 'd8) + c_x)];

    text_display_FIFO FIFO
    (
        .clock(clk),
        .data(data),
        .rdreq(rdreq),
        .sclr(rst),
        .wrreq(wrreq),
        .empty(fifo_empty),
	    .q(fifo_out)
    );

    text_display_buffer_reg buffer_reg
    (
        .clk(clk),
        .rst(rst),
        .load_buff_reg(load_buff_reg),
        .fifo_out(fifo_out),
        .buff_reg_out(buff_reg_out)
    );

    text_display_font_rom font_rom
    (
        .address(buff_reg_out),
        .clock(clk),
        .q(text_out)
    );

    text_diaplay_cursor cursor
    (
        .clk(clk),
        .rst(rst),
        .inc_cursor(inc_cursor),
        .carriage_cursor(carriage_cursor),
        .clear_screen(clear_screen),
        .cursor_x(cursor_x),
        .cursor_y(cursor_y)
    );

    text_display_offset_counter offset_counter
    (
        .clk(clk),
        .rst(rst),
        .display_char(display_char),
        .done_char(done_char),
        .c_x(c_x),
        .c_y(c_y)
    );

    text_display_clear clear
    (
        .clk(clk),
        .rst(rst),
        .do_clear(do_clear),
        .done_clear(done_clear),
        .x(clear_x),
        .y(clear_y),
        .colour(clear_colour)
    );

    vga_adapter adapter
    (
        .resetn(~rst),
        .clock(clk),
        .colour(colour_final),
        .x(x_final),
        .y(y_final),
        .plot(plot),
        .VGA_R(VGA_R),
        .VGA_G(VGA_G),
        .VGA_B(VGA_B),
        .VGA_HS(VGA_HS),
        .VGA_VS(VGA_VS),
        .VGA_BLANK(VGA_BLANK),
        .VGA_SYNC(VGA_SYNC),
        .VGA_CLK(VGA_CLK)
    );
endmodule