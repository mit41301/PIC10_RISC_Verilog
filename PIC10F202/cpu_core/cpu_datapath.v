// This is part of the ECE241 final project
// Date created: November 17 2016
// This file contains the datapath for the core of the cpu

module cpu_datapath(clk, rst, store_alu_w, alu_in_select, load_status_reg, skip_next_instruction, load_instruction_reg, pc_mux_select, load_pc, inc_pc, inc_stack, dec_stack, load_stack, load_fsr, reg_address_mux_select, load_ram, program_bus, load_tris0, load_tris1, load_tris2, load_gpio0, load_gpio1, load_gpio2, gpio0_bus, gpio1_bus, gpio2_bus, reg_address, instruction_reg_out, program_address, zero_result, SW, LEDR, HEX0, HEX1, HEX2, HEX3, HEX4);
    `include "definition.vh"

    input clk;
    input rst;
    input store_alu_w;
    input alu_in_select;
    input load_status_reg;
    input skip_next_instruction;
    input load_instruction_reg;
    input [1:0]pc_mux_select;
    input load_pc;
    input inc_pc;
    input inc_stack;
    input dec_stack;
    input load_stack;
    input load_fsr;
    input reg_address_mux_select;
    input load_ram;
    input [11:0]program_bus;

    input load_tris0;
    input load_tris1;
    input load_tris2;
    input load_gpio0;
    input load_gpio1;
    input load_gpio2;

    inout [7:0]gpio0_bus;
    inout [7:0]gpio1_bus;
    inout [7:0]gpio2_bus;

    output [4:0]reg_address;
    output [11:0]instruction_reg_out;
    output [8:0]program_address;
    output zero_result;

    input [9:0]SW;
    output [9:0]LEDR;
    output [6:0]HEX0;
    output [6:0]HEX1;
    output [6:0]HEX2;
    output [6:0]HEX3;
    output [6:0]HEX4;

    // alu_datapath wires
    wire [11:0]wire_instruction_bus;
    wire [7:0]wire_sfr_bus;
    wire [7:0]wire_data_reg;
    wire [7:0]wire_alu_bus;
    wire wire_carry_bit;
    wire [2:0]wire_status_bus;
    wire wire_status_c_load;
    wire wire_status_dc_load;
    wire wire_status_z_load;
    wire [7:0]wire_w_reg_out;

    // status_reg wires
    wire [7:0]wire_status_reg;

    // sfr_mux wires
    wire [4:0]wire_reg_address;
    wire [7:0]wire_fsr_reg;

    // instruction_datapath wires
    wire [8:0]wire_program_address;

    // fsr_datapath wires
    wire [191:0]wire_all_reg_out;

    // GPIO wires
    wire [7:0]gpio0_input;
    wire [7:0]gpio1_input;
    wire [7:0]gpio2_input;

    assign reg_address = wire_reg_address;
    assign instruction_reg_out = wire_instruction_bus;
    assign program_address = wire_program_address;
    assign zero_result = wire_status_bus[`STATUS_Z];

    // The alu_datapath module
    cpu_alu_datapath alu_datapath
    (
        .clk(clk),
        .rst(rst),
        .instruction_in(wire_instruction_bus),
        .sfr_in(wire_sfr_bus),
        .data_reg_in(wire_data_reg),
        .carry_bit_in(wire_carry_bit),
        .store_alu_w(store_alu_w),
        .alu_in_select(alu_in_select),
        .alu_to_reg(wire_alu_bus),
        .status_bus_out(wire_status_bus),
        .status_c_load(wire_status_c_load),
        .status_dc_load(wire_status_dc_load),
        .status_z_load(wire_status_z_load),
        .w_reg_out(wire_w_reg_out)
    );

    // The status_reg module
    cpu_status_reg status_reg
    (
        .clk(clk),
        .rst(rst),
        .alu_to_status(wire_alu_bus),
        .status_bus(wire_status_bus),
        .load_status_reg(load_status_reg),
        .status_c_load(wire_status_c_load),
        .status_dc_load(wire_status_dc_load),
        .status_z_load(wire_status_z_load),
        .status_carry_bit(wire_carry_bit),
        .status_reg_out(wire_status_reg)
    );

    // The sfr_mux module
    reg [7:0]null_input = 8'b0;
    cpu_sfr_mux sfr_mux
    (
        .sfr_mux_select(wire_reg_address[2:0]),
        .indf_reg(null_input),
        .tmr0_reg(null_input),
        .pc_low_reg(wire_program_address[7:0]),
        .status_reg(wire_status_reg),
        .fsr_reg(wire_fsr_reg),
        .gpio0(gpio0_input),
        .gpio1(gpio1_input),
        .gpio2(gpio2_input),
        .sfr_mux_out(wire_sfr_bus)
    );

    // The instruction_datapath module
    cpu_instruction_datapath instruction_datapath
    (
        .clk(clk),
        .rst(rst),
        .program_bus(program_bus),
        .nop_insert(skip_next_instruction),
        .load_instruction(load_instruction_reg),
        .pc_mux_select(pc_mux_select),
        .load_pc(load_pc),
        .inc_pc(inc_pc),
        .alu_output(wire_alu_bus),
        .inc_stack(inc_stack),
        .dec_stack(dec_stack),
        .load_stack(load_stack),
        .instruction_reg_out(wire_instruction_bus),
        .pc_to_program_rom(wire_program_address)
    );

    // The fsr_datapath module
    cpu_fsr_datapath fsr_datapath
    (
        .clk(clk),
        .rst(rst),
        .load_fsr(load_fsr),
        .reg_address_mux_select(reg_address_mux_select),
        .instruction_reg_output(wire_instruction_bus),
        .load_ram(load_ram),
        .alu_output(wire_alu_bus),
        .reg_address_out(wire_reg_address),
        .fsr_reg_out(wire_fsr_reg),
        .ram_out(wire_data_reg),
        .all_reg_out(wire_all_reg_out)
    );

    // GPIO0
    cpu_gpio gpio0
    (
        .clk(clk),
        .rst(rst),
        .alu_output(wire_alu_bus),
        .load_tris_reg(load_tris0),
        .load_gpio_reg(load_gpio0),
        .gpio_bus(gpio0_bus),
        .gpio_input(gpio0_input)
    );

    // GPIO1
    cpu_gpio gpio1
    (
        .clk(clk),
        .rst(rst),
        .alu_output(wire_alu_bus),
        .load_tris_reg(load_tris1),
        .load_gpio_reg(load_gpio1),
        .gpio_bus(gpio1_bus),
        .gpio_input(gpio1_input)
    );

    // GPIO2
    cpu_gpio gpio2
    (
        .clk(clk),
        .rst(rst),
        .alu_output(wire_alu_bus),
        .load_tris_reg(load_tris2),
        .load_gpio_reg(load_gpio2),
        .gpio_bus(gpio2_bus),
        .gpio_input(gpio2_input)
    );

    // Monitor CPU's register
    cpu_watch watch
    (
        .pc(wire_program_address),
        .pcl(wire_program_address[7:0]),
        .status(wire_status_reg),
        .fsr(wire_fsr_reg),
        .gpio0(gpio0_input),
        .gpio1(gpio1_input),
        .gpio2(gpio2_input),
        .w_reg_out(wire_w_reg_out),
        .instruction_reg(wire_instruction_bus),
        .all_reg_out(wire_all_reg_out),
        .SW(SW),
        .LEDR(LEDR),
        .HEX0(HEX0),
        .HEX1(HEX1),
        .HEX2(HEX2),
        .HEX3(HEX3),
        .HEX4(HEX4)
    );
endmodule