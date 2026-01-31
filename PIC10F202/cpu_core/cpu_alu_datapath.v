// This is part of the ECE241 final project
// Date created: November 14 2016
// This file contains the ALU(Arithmetic logic Unit) datapath

module cpu_alu_datapath(clk, rst, instruction_in, sfr_in, data_reg_in, carry_bit_in, store_alu_w, alu_in_select, alu_to_reg, status_bus_out, status_c_load, status_dc_load, status_z_load, w_reg_out);

    input clk;
    input rst;
    input [11:0]instruction_in;
    input [7:0]sfr_in;
    input [7:0]data_reg_in;
    input carry_bit_in;
    input store_alu_w;
    input alu_in_select;
    output [7:0]alu_to_reg;
    output [2:0]status_bus_out;
    output status_c_load;
    output status_dc_load;
    output status_z_load;
    output [7:0]w_reg_out;

    wire [7:0]wire_w_accumulator;
    wire [7:0]wire_mux_to_alu;
    wire [7:0]wire_alu_output;

    // The alu module
    cpu_alu alu
    (
        .alu_op_in(instruction_in),
        .alu_in_w(wire_w_accumulator),
        .alu_in_mux(wire_mux_to_alu),
        .status_carry_in(carry_bit_in),
        .alu_out(wire_alu_output),
        .status_c_load(status_c_load),
        .status_dc_load(status_dc_load),
        .status_z_load(status_z_load),
        .alu_status_out(status_bus_out)
    );

    // The alu_input_mux module
    cpu_alu_input_mux alu_mux
    (
        .data_reg_in(data_reg_in),
        .sfr_in(sfr_in),
        .alu_mux_select(alu_in_select),
        .mux_to_alu(wire_mux_to_alu)
    );

    // The w_accumulator module
    cpu_w_accumulator w_accumulator
    (
        .clk(clk),
        .rst(rst),
        .load_w(store_alu_w),
        .alu_to_w(wire_alu_output),
        .w_accumulator_out(wire_w_accumulator)
    );
    assign alu_to_reg = wire_alu_output; // The ouput of ALU
    assign w_reg_out = wire_w_accumulator;
endmodule