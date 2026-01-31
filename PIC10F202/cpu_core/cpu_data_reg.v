// This is part of the ECE241 final project
// Date created: November 14 2016
// This file contains the data registers, there are 24 in total

module cpu_data_reg(clk, rst, alu_out_to_reg, reg_address, write_enable, data_reg_out, all_reg_out);

    input clk;
    input rst;
    input [7:0]alu_out_to_reg;
    input [4:0]reg_address;
    input write_enable;
    output [7:0]data_reg_out;
    output [191:0]all_reg_out;

    assign all_reg_out = {data_reg[23], data_reg[22], data_reg[21], data_reg[20], data_reg[19], data_reg[18], data_reg[17], data_reg[16], data_reg[15], data_reg[14], data_reg[13], data_reg[12], data_reg[11], data_reg[10], data_reg[9], data_reg[8], data_reg[7], data_reg[6], data_reg[5], data_reg[4], data_reg[3], data_reg[2], data_reg[1], data_reg[0]};

    reg [7:0]data_reg[0:23];

    always @ (posedge clk)
    begin
        if (rst)
        begin
            data_reg[0] <= 8'b0;
            data_reg[1] <= 8'b0;
            data_reg[2] <= 8'b0;
            data_reg[3] <= 8'b0;
            data_reg[4] <= 8'b0;
            data_reg[5] <= 8'b0;
            data_reg[6] <= 8'b0;
            data_reg[7] <= 8'b0;
            data_reg[8] <= 8'b0;
            data_reg[9] <= 8'b0;
            data_reg[10] <= 8'b0;
            data_reg[11] <= 8'b0;
            data_reg[12] <= 8'b0;
            data_reg[13] <= 8'b0;
            data_reg[14] <= 8'b0;
            data_reg[15] <= 8'b0;
            data_reg[16] <= 8'b0;
            data_reg[17] <= 8'b0;
            data_reg[18] <= 8'b0;
            data_reg[19] <= 8'b0;
            data_reg[20] <= 8'b0;
            data_reg[21] <= 8'b0;
            data_reg[22] <= 8'b0;
            data_reg[23] <= 8'b0;
        end
        else if (write_enable)
        begin
            data_reg[reg_address - 8] <= alu_out_to_reg;
        end
    end
    assign data_reg_out = (reg_address >= 5'd8) ? data_reg[reg_address - 8] : 8'b0; // Output 
endmodule