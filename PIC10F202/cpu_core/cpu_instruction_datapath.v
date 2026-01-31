// This is part of the ECE241 final project
// Date created: November 14 2016
// This file contains the datapath for the program counter, instruction register and stack

module cpu_instruction_datapath(clk, rst, program_bus, nop_insert, load_instruction, pc_mux_select, load_pc, inc_pc, alu_output, inc_stack, dec_stack, load_stack, instruction_reg_out, pc_to_program_rom);

    input clk;
    input rst;
    input [11:0]program_bus;
    input nop_insert;
    input load_instruction;
    input [1:0]pc_mux_select;
    input load_pc;
    input inc_pc;
    input [7:0]alu_output;
    input inc_stack;
    input dec_stack;
    input load_stack;
    output [11:0]instruction_reg_out;
    output [8:0]pc_to_program_rom;

    wire [11:0]wire_nop_mux_to_instruction_reg;
    wire [11:0]wire_instruction_reg_to_pc_mux;
    wire [8:0]wire_stack_to_pc_mux;
    wire [8:0]wire_pc_mux_to_pc;
    wire [8:0]wire_pc_to_stack;

    // The nop_mux module
    cpu_nop_mux nop_mux
    (
        .nop_insert(nop_insert),
        .program_bus_in(program_bus),
        .program_bus_out(wire_nop_mux_to_instruction_reg)
    );

    // The instruction_reg module
    cpu_instruction_reg instruction_reg
    (
        .clk(clk),
        .rst(rst),
        .load_instruction(load_instruction),
        .program_to_reg(wire_nop_mux_to_instruction_reg),
        .instruction_reg_out(wire_instruction_reg_to_pc_mux)
    );

    // The pc_mux module
    cpu_pc_mux pc_mux
    (
        .pc_mux_select(pc_mux_select),
        .stack_in(wire_stack_to_pc_mux),
        .alu_in(alu_output),
        .instruction_in(wire_instruction_reg_to_pc_mux[8:0]),
        .pc_mux_out(wire_pc_mux_to_pc)
    );

    // The pc module
    cpu_pc pc
    (
        .clk(clk),
        .rst(rst),
        .pc_inc(inc_pc),
        .load_pc(load_pc),
        .pc_input_from_mux(wire_pc_mux_to_pc),
        .pc_out(wire_pc_to_stack)
    );

    // The stack module
    cpu_stack stack
    (
        .clk(clk),
        .rst(rst),
        .load_stack(load_stack),
        .dec_stack_ptr(dec_stack),
        .inc_stack_ptr(inc_stack),
        .pc_to_stack(wire_pc_to_stack),
        .stack_out(wire_stack_to_pc_mux)
    );

    assign instruction_reg_out = wire_instruction_reg_to_pc_mux;
    assign pc_to_program_rom = wire_pc_to_stack;
endmodule