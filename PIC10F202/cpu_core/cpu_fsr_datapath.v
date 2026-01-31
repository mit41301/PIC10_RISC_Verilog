// This is part of the ECE241 final project
// Date created: November 17 2016
// This file contains the datapath for the file select register , general purpose registers(data_reg) and related circuit

module cpu_fsr_datapath(clk, rst, load_fsr, reg_address_mux_select, instruction_reg_output, load_ram, alu_output, reg_address_out, fsr_reg_out, ram_out, all_reg_out);

    input clk;
    input rst;
    input load_fsr;
    input reg_address_mux_select;
    input [11:0]instruction_reg_output;
    input load_ram;
    input [7:0]alu_output;
    output [4:0]reg_address_out;
    output [7:0]fsr_reg_out;
    output [7:0]ram_out;
    output [191:0]all_reg_out;

    wire [7:0]wire_fsr_to_address_mux;
    wire [4:0]wire_address_mux_to_data_reg;

    // The fsr module
    cpu_fsr fsr
    (
        .clk(clk),
        .rst(rst),
        .alu_to_fsr(alu_output),
        .load_fsr(load_fsr),
        .fsr_out(wire_fsr_to_address_mux)
    );

    // The reg_address_mux module
    cpu_reg_address_mux reg_address_mux
    (
        .reg_address_mux_select(reg_address_mux_select),
        .instruction_reg_output(instruction_reg_output[4:0]),
        .fsr_reg_output(wire_fsr_to_address_mux[4:0]),
        .reg_address_mux_out(wire_address_mux_to_data_reg)
    );

    // The data_reg module
    cpu_data_reg data_reg
    (
        .clk(clk),
        .rst(rst),
        .alu_out_to_reg(alu_output),
        .reg_address(wire_address_mux_to_data_reg),
        .write_enable(load_ram),
        .data_reg_out(ram_out),
        .all_reg_out(all_reg_out)
    );

    assign fsr_reg_out = wire_fsr_to_address_mux;
    assign reg_address_out = wire_address_mux_to_data_reg;
endmodule