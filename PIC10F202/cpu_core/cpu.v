// This is part of the ECE241 final project
// Date created: November 19 2016
// This file contains the core of the cpu
    
module cpu(clk, rst, GPIO0, GPIO1, GPIO2, SW, LEDR, HEX0, HEX1, HEX2, HEX3, HEX4);

    input clk;
    input rst;

    inout [7:0]GPIO0;
    inout [7:0]GPIO1;
    inout [7:0]GPIO2;

    input [9:0]SW;
    output [9:0]LEDR;
    output [6:0]HEX0;
    output [6:0]HEX1;
    output [6:0]HEX2;
    output [6:0]HEX3;
    output [6:0]HEX4;

    wire [4:0]reg_address;
    wire [11:0]instruction_reg_out;
    wire zero_result;
    wire store_alu_w;
    wire alu_in_select;
    wire load_status_reg;
    wire skip_next_instruction;
    wire load_instruction_reg;
    wire [1:0]pc_mux_select;
    wire load_pc;
    wire inc_pc;
    wire inc_stack;
    wire dec_stack;
    wire load_stack;
    wire load_fsr;
    wire reg_address_mux_select;
    wire load_ram;
    wire load_tris0;
    wire load_tris1;
    wire load_tris2;
    wire load_gpio0;
    wire load_gpio1;
    wire load_gpio2;

    wire [11:0]program_bus;
    wire [8:0]program_address;

    cpu_controller controller
    (
        .clk(clk),
        .rst(rst),
        .reg_address(reg_address),
        .instruction_reg_out(instruction_reg_out),
        .zero_result(zero_result),
        .store_alu_w(store_alu_w),
        .alu_in_select(alu_in_select),
        .load_status_reg(load_status_reg),
        .skip_next_instruction(skip_next_instruction),
        .load_instruction_reg(load_instruction_reg),
        .pc_mux_select(pc_mux_select),
        .load_pc(load_pc),
        .inc_pc(inc_pc),
        .inc_stack(inc_stack),
        .dec_stack(dec_stack),
        .load_stack(load_stack),
        .load_fsr(load_fsr),
        .reg_address_mux_select(reg_address_mux_select),
        .load_ram(load_ram),
        .load_tris0(load_tris0),
        .load_tris1(load_tris1),
        .load_tris2(load_tris2),
        .load_gpio0(load_gpio0),
        .load_gpio1(load_gpio1),
        .load_gpio2(load_gpio2)
    );


    cpu_datapath datapath
    (
        .clk(clk),
        .rst(rst),
        .store_alu_w(store_alu_w),
        .alu_in_select(alu_in_select),
        .load_status_reg(load_status_reg),
        .skip_next_instruction(skip_next_instruction),
        .load_instruction_reg(load_instruction_reg),
        .pc_mux_select(pc_mux_select),
        .load_pc(load_pc),
        .inc_pc(inc_pc),
        .inc_stack(inc_stack),
        .dec_stack(dec_stack),
        .load_stack(load_stack),
        .load_fsr(load_fsr),
        .reg_address_mux_select(reg_address_mux_select),
        .load_ram(load_ram),
        .program_bus(program_bus),
        .load_tris0(load_tris0),
        .load_tris1(load_tris1),
        .load_tris2(load_tris2),
        .load_gpio0(load_gpio0),
        .load_gpio1(load_gpio1),
        .load_gpio2(load_gpio2),
        .gpio0_bus(GPIO0),
        .gpio1_bus(GPIO1),
        .gpio2_bus(GPIO2),
        .reg_address(reg_address),
        .instruction_reg_out(instruction_reg_out),
        .program_address(program_address),
        .zero_result(zero_result),
        .SW(SW),
        .LEDR(LEDR),
        .HEX0(HEX0),
        .HEX1(HEX1),
        .HEX2(HEX2),
        .HEX3(HEX3),
        .HEX4(HEX4)
    );

    cpu_program_rom program_rom
    (
        .address(program_address),
        .clock(~clk),
        .q(program_bus)
    );

endmodule