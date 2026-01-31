// This is part of the ECE241 final project
// Date created: November 17 2016
// This file contains the gpio definition

module cpu_gpio(clk, rst, alu_output, load_tris_reg, load_gpio_reg, gpio_bus, gpio_input);
    input clk;
    input rst;
    input [7:0]alu_output;
    input load_tris_reg;
    input load_gpio_reg;
    inout [7:0]gpio_bus;
    output [7:0]gpio_input;

    reg [7:0]tris_reg;
    reg [7:0]gpio_reg;

    // Always block for tris_reg
    always @ (posedge clk)
    begin
        if (rst)
            tris_reg <= 8'hFF; //Reset as input port, follows datasheet
        else if (load_tris_reg)
            tris_reg <= alu_output;
    end

    // Always block for gpio_reg
    always @ (posedge clk)
    begin
        if (rst)
            gpio_reg <= 8'b0;
        else if (load_gpio_reg)
            gpio_reg <= alu_output;
    end

    // gpio_input will always equal to the input/output
    assign gpio_bus[0] = tris_reg[0] ? 1'bz : gpio_reg[0];
    assign gpio_bus[1] = tris_reg[1] ? 1'bz : gpio_reg[1];
    assign gpio_bus[2] = tris_reg[2] ? 1'bz : gpio_reg[2];
    assign gpio_bus[3] = tris_reg[3] ? 1'bz : gpio_reg[3];
    assign gpio_bus[4] = tris_reg[4] ? 1'bz : gpio_reg[4];
    assign gpio_bus[5] = tris_reg[5] ? 1'bz : gpio_reg[5];
    assign gpio_bus[6] = tris_reg[6] ? 1'bz : gpio_reg[6];
    assign gpio_bus[7] = tris_reg[7] ? 1'bz : gpio_reg[7];

    assign gpio_input = gpio_bus;
endmodule