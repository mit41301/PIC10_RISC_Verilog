// This is part of the ECE241 final project
// Date created: November 10 2016
// This file contains the ALU(Arithmetic logic Unit) of The CPU

module cpu_alu(alu_op_in, alu_in_w, alu_in_mux, status_carry_in, alu_out, status_c_load, status_dc_load, status_z_load, alu_status_out);
    `include "definition.vh"

    input [11:0]alu_op_in;
    input [7:0]alu_in_w;
    input [7:0]alu_in_mux;
    input status_carry_in;
    output reg [7:0]alu_out;
    output reg status_c_load, status_dc_load, status_z_load;
    output reg [2:0]alu_status_out;

    reg [3:0]dump; //In some operation some values are not useful
    reg [2:0]b; //b represent the position in f
    reg [7:0]mask; //Mask to clear and set

    //The always block for the ALU
    always @ (*)
    begin
        //The default control signals and status output is set to 0
        status_c_load = 1'b0;
        status_dc_load = 1'b0;
        status_z_load = 1'b0;
        alu_status_out = 3'b0;

        //Default output for ALU
        alu_out = 8'b0;

        //Default for local things
        dump = 4'b0;
        b = 3'b0;
        mask = 8'b0;

        casex(alu_op_in)
            `ADDWF: //ADDWF status register C,DC,Z
            begin
                {alu_status_out[`STATUS_C], alu_out} = alu_in_w + alu_in_mux;
                {alu_status_out[`STATUS_DC], dump} = alu_in_w[3:0] + alu_in_mux[3:0];
                alu_status_out[`STATUS_Z] = (alu_out == 12'b0);
                status_c_load = 1'b1;
                status_dc_load = 1'b1;
                status_z_load = 1'b1;
            end
            `ANDWF: //ANDWF status register Z
            begin
                alu_out = alu_in_w & alu_in_mux;
                alu_status_out[`STATUS_Z] = (alu_out == 12'b0);
                status_z_load = 1'b1;
            end
            `CLRF: //CLRF status register Z
            begin
                alu_out = 8'b0;
                alu_status_out[`STATUS_Z] = 1'b1;
                status_z_load = 1'b1;
            end
            `CRLW: //CRLW status register Z
            begin
                alu_out = 8'b0;
                alu_status_out[`STATUS_Z] = 1'b1;
                status_z_load = 1'b1;
            end
            `COMF: //COMF status resigter Z
            begin
                alu_out = ~alu_in_mux;
                alu_status_out[`STATUS_Z] = (alu_out == 12'b0);
                status_z_load = 1'b1;
            end
            `DECF: //DECF status register Z
            begin
                alu_out = alu_in_mux - 1'b1;
                alu_status_out[`STATUS_Z] = (alu_out == 12'b0);
                status_z_load = 1'b1;
            end
            `DECFSZ: //DECFSZ status none
            begin
                alu_out = alu_in_mux - 1'b1;
                alu_status_out[`STATUS_Z] = (alu_out == 12'b0);
            end
            `INCF: //INCF status register Z
            begin
                alu_out = alu_in_mux + 1'b1;
                alu_status_out[`STATUS_Z] = (alu_out == 12'b0);
                status_z_load = 1'b1;
            end
            `INCFSZ: //INCFSZ status none
            begin
                alu_out = alu_in_mux + 1'b1;
                alu_status_out[`STATUS_Z] = (alu_out == 12'b0);
            end
            `IORWF: //IORWF status Z
            begin
                alu_out = alu_in_w | alu_in_mux;
                alu_status_out[`STATUS_Z] = (alu_out == 12'b0);
                status_z_load = 1'b1;
            end
            `MOVF: //MOVF status Z
            begin
                alu_out = alu_in_mux;
                alu_status_out[`STATUS_Z] = (alu_out == 12'b0);
                status_z_load = 1'b1;
            end
            `MOVWF: //MOVWF status none
            begin
                alu_out = alu_in_w;
            end
            `NOP: //NOP status none
            begin
                alu_out = 8'b0;
            end
            `RLF: //RLF status C
            begin
                {alu_status_out[`STATUS_C], alu_out} = {alu_in_mux, status_carry_in};
                status_c_load = 1'b1;
            end
            `RRF: //RRF status C
            begin
                {alu_out, alu_status_out[`STATUS_C]} = {status_carry_in, alu_in_mux};
                status_c_load = 1'b1;
            end
            `SUBWF: //SUBWF status register C,DC,Z
            begin
                alu_out = alu_in_mux - alu_in_w;
                alu_status_out[`STATUS_C] = alu_in_mux >= alu_in_w;
                alu_status_out[`STATUS_DC] = alu_in_mux[3:0] >= alu_in_w[3:0];
                alu_status_out[`STATUS_Z] = (alu_out == 12'b0);
                status_c_load = 1'b1;
                status_dc_load = 1'b1;
                status_z_load = 1'b1;
            end
            `SWAPF: //SWAPF status none
            begin
                alu_out[3:0] = alu_in_mux[7:4];
                alu_out[7:4] = alu_in_mux[3:0];
            end
            `XORWF: //XORWF status register Z
            begin
                alu_out = alu_in_w ^ alu_in_mux;
                alu_status_out[`STATUS_Z] = (alu_out == 12'b0);
                status_z_load = 1'b1;
            end
            `BCF: //BCF status none
            begin
                b = alu_op_in[7:5];
                mask = ~(1<<b);
                alu_out = alu_in_mux & mask;
            end
            `BSF: //BSF status none
            begin
                b = alu_op_in[7:5];
                mask = (1<<b);
                alu_out = alu_in_mux | mask;
            end
            `BTFSC: //BTFSC status none
            begin
                b = alu_op_in[7:5];
                mask = (1<<b);
                alu_status_out[`STATUS_Z] = ((alu_in_mux & mask) != mask);
            end
            `BTFSS: //BTFSS status none
            begin
                b = alu_op_in[7:5];
                mask = (1<<b);
                alu_status_out[`STATUS_Z] = ((alu_in_mux & mask) != mask);
            end
            `ANDLW: //ANDLW status Z
            begin
                alu_out = alu_in_w & alu_op_in[7:0];
                alu_status_out[`STATUS_Z] = (alu_out == 12'b0);
                status_z_load = 1'b1;
            end
            `CALL: //CALL status none
            begin
                alu_out = alu_op_in[7:0];
            end
            `CLRWDT: //CLRWDT status TO,PD
            begin
                //WDT not implemented in this design
            end
            `GOTO: //GOTO status none
            begin
                //No operation, loaded outside already
            end
            `IORLW: //IORLW status Z
            begin
                alu_out = alu_in_w | alu_op_in[7:0];
                alu_status_out[`STATUS_Z] = (alu_out == 12'b0);
                status_z_load = 1'b1;
            end
            `MOVLW: //MOVLW status none
            begin
                alu_out = alu_op_in[7:0];
            end
            `OPTION: //OPTION status none
            begin
                alu_out = alu_in_w;
            end
            `RETLW: //RETLW status none
            begin
                alu_out = alu_op_in[7:0];
            end
            `SLEEP: //SLEEP status TO,PD
            begin
                //SLEEP not implemented in this design
            end
            `TRIS0: //TRIS0 status none
            begin
                alu_out = alu_in_w;
            end
            `TRIS1: //TRIS1 status none
            begin
                alu_out = alu_in_w;
            end
            `TRIS2: //TRIS2 status none
            begin
                alu_out = alu_in_w;
            end
            `XORLW: //XORLW status Z
            begin
                alu_out = alu_in_w ^ alu_op_in[7:0];
                alu_status_out[`STATUS_Z] = (alu_out == 12'b0);
                status_z_load = 1'b1;
            end
            default: ;//Do nothing
        endcase
    end
endmodule