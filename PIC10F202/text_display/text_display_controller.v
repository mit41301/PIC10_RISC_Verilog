// This is part of the ECE241 final project
// Date created: November 27 2016
// This file contains the controller for the text_display module

module text_display_controller(clk, rst, fifo_empty, new_line_ascii, clear_screen, done_char, done_clear, rdreq, load_buff_reg, inc_cursor, carriage_cursor, display_char, do_clear, plot);
    input clk;
    input rst;
    input fifo_empty;
    input new_line_ascii;
    input clear_screen;
    input done_char;
    input done_clear;

    output reg rdreq;
    output reg load_buff_reg;
    output reg inc_cursor;
    output reg carriage_cursor;
    output reg display_char;
    output reg do_clear;
    output reg plot;

    localparam  WAIT = 4'd0,
                RDREQ = 4'd1,
                LOAD_BUFFER = 4'd2,
                WAIT_BUFF = 4'd3,
                WAIT_ROM = 4'd4,
                DRAW = 4'd5,
                INC_CURSOR = 4'd6,
                CARRIAGE_CURSOR = 4'd7,
                CLEAR_SCREEN = 4'd8;

    always @ (*)
    begin
        case (current_state)
        WAIT:           next_state = fifo_empty ? WAIT : RDREQ;
        RDREQ:          next_state = LOAD_BUFFER;
        LOAD_BUFFER:    next_state = WAIT_BUFF;
        WAIT_BUFF:      next_state = new_line_ascii ? CARRIAGE_CURSOR : WAIT_ROM;
        WAIT_ROM:       next_state = DRAW;
        DRAW:           next_state = done_char ? INC_CURSOR : DRAW;
        INC_CURSOR:     next_state = clear_screen ? CLEAR_SCREEN : WAIT;
        CARRIAGE_CURSOR:next_state = clear_screen ? CLEAR_SCREEN : WAIT;
        CLEAR_SCREEN:   next_state = done_clear ? WAIT : CLEAR_SCREEN;
        default:        next_state = WAIT;
        endcase
    end

    always @(*)
    begin
        rdreq = 1'b0;
        load_buff_reg = 1'b0;
        inc_cursor = 1'b0;
        carriage_cursor = 1'b0;
        display_char = 1'b0;
        do_clear = 1'b0;
        plot = 1'b0;

        case (current_state)
        WAIT:
        begin

        end
        RDREQ:
        begin
            rdreq = 1'b1;
        end
        LOAD_BUFFER:
        begin
            load_buff_reg = 1'b1;
        end
        WAIT_BUFF:
        begin

        end
        WAIT_ROM:
        begin

        end
        DRAW:
        begin
            plot = 1'b1;
            display_char = 1'b1;
        end
        INC_CURSOR:
        begin
            inc_cursor = 1'b1;
        end
        CARRIAGE_CURSOR:
        begin
            carriage_cursor = 1'b1;
        end
        CLEAR_SCREEN:
        begin
            plot = 1'b1;
            do_clear = 1'b1;
        end
        endcase
    end
    // Resister for state
	reg [3:0]current_state, next_state;
	always @ (posedge clk)
	begin
		if(rst)
			current_state <= WAIT;
		else
			current_state <= next_state;
	end
endmodule