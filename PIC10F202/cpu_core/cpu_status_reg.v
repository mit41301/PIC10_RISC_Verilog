// This is part of the ECE241 final project
// Date created: November 14 2016
// This file contains the status register which saves all status output from ALU

module cpu_status_reg(clk, rst, alu_to_status, status_bus, load_status_reg, status_c_load, status_dc_load, status_z_load, status_carry_bit, status_reg_out);
    `include "definition.vh"

    input clk;
    input rst;
    input [7:0]alu_to_status;
    input [2:0]status_bus;
    input load_status_reg;
    input status_c_load;
    input status_dc_load;
    input status_z_load;
    output status_carry_bit;
    output reg [7:0]status_reg_out;

    always @ (posedge clk)
    begin
        if (rst)
            status_reg_out <= 8'b0;
        else
        begin
            if (load_status_reg)
            begin
                if(status_c_load||status_dc_load||status_z_load)
                    status_reg_out[7:3] <= alu_to_status[7:3]; //Write to C,DC,Z is disabled if any of the status_X_load is high
                else
                    status_reg_out <= alu_to_status;
            end
            if(status_c_load)
                status_reg_out[`STATUS_C] <= status_bus[`STATUS_C];
            if(status_dc_load)
                status_reg_out[`STATUS_DC] <= status_bus[`STATUS_DC];
            if(status_z_load)
                status_reg_out[`STATUS_Z] <= status_bus[`STATUS_Z];
        end
    end
    assign status_carry_bit = status_reg_out[`STATUS_C];
endmodule
