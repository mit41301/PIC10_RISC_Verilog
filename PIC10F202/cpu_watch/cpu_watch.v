// This is part of the ECE241 final project
// Date created: November 21 2016
// This file contains functionalitys that helps to monitor all the registers in the cpu

module cpu_watch(pc, pcl, status, fsr, gpio0, gpio1, gpio2, w_reg_out, instruction_reg, all_reg_out, SW, LEDR, HEX0, HEX1, HEX2, HEX3, HEX4);

    input [8:0]pc;
    input [7:0]pcl;
    input [7:0]status;
    input [7:0]fsr;
    input [7:0]gpio0;
    input [7:0]gpio1;
    input [7:0]gpio2;
    input [7:0]w_reg_out;
    input [11:0]instruction_reg;
    input [191:0]all_reg_out;

    input [9:0]SW;
    output [9:0]LEDR;
    output [6:0]HEX0;
    output [6:0]HEX1;
    output [6:0]HEX2;
    output [6:0]HEX3;
    output [6:0]HEX4;

    reg [7:0]reg_out = 8'b0;
    wire [7:0]reg_display;

    always @ (*)
    begin
        case (SW[4:0])
            5'd0:   reg_out = 8'b0;
            5'd1:   reg_out = 8'b0;
            5'd2:   reg_out = pcl;
            5'd3:   reg_out = status;
            5'd4:   reg_out = fsr;
            5'd5:   reg_out = gpio0;
            5'd6:   reg_out = gpio1;
            5'd7:   reg_out = gpio2;
            5'd8:   reg_out = all_reg_out[7:0];
            5'd9:   reg_out = all_reg_out[15:8];
            5'd10:  reg_out = all_reg_out[23:16];
            5'd11:  reg_out = all_reg_out[31:24];
            5'd12:  reg_out = all_reg_out[39:32];
            5'd13:  reg_out = all_reg_out[47:40];
            5'd14:  reg_out = all_reg_out[55:48];
            5'd15:  reg_out = all_reg_out[63:56];
            5'd16:  reg_out = all_reg_out[71:64];
            5'd17:  reg_out = all_reg_out[79:72];
            5'd18:  reg_out = all_reg_out[87:80];
            5'd19:  reg_out = all_reg_out[95:88];
            5'd20:  reg_out = all_reg_out[103:96];
            5'd21:  reg_out = all_reg_out[111:104];
            5'd22:  reg_out = all_reg_out[119:112];
            5'd23:  reg_out = all_reg_out[127:120];
            5'd24:  reg_out = all_reg_out[135:128];
            5'd25:  reg_out = all_reg_out[143:136];
            5'd26:  reg_out = all_reg_out[151:144];
            5'd27:  reg_out = all_reg_out[159:152];
            5'd28:  reg_out = all_reg_out[167:160];
            5'd29:  reg_out = all_reg_out[175:168];
            5'd30:  reg_out = all_reg_out[183:176];
            5'd31:  reg_out = all_reg_out[191:184];
        endcase
    end

    assign LEDR[8:0] = pc;

    assign reg_display = SW[5] ? w_reg_out : reg_out;

    hex_decoder H0
    (
        .hex_digit(reg_display[3:0]), 
        .segments(HEX0)
    );

    hex_decoder H1
    (
        .hex_digit(reg_display[7:4]), 
        .segments(HEX1)
    );

    hex_decoder H2
    (
        .hex_digit(instruction_reg[3:0]), 
        .segments(HEX2)
    );

    hex_decoder H3
    (
        .hex_digit(instruction_reg[7:4]), 
        .segments(HEX3)
    );

    hex_decoder H4
    (
        .hex_digit(instruction_reg[11:8]), 
        .segments(HEX4)
    );

endmodule