// This is part of the ECE241 final project
// Date created: November 17 2016
// This file contains the mux that choose one of the sfr to feed into the ALU

module cpu_sfr_mux(sfr_mux_select, indf_reg, tmr0_reg, pc_low_reg, status_reg, fsr_reg, gpio0, gpio1, gpio2, sfr_mux_out);
    `include "definition.vh"

    input [2:0]sfr_mux_select;
    input [7:0]indf_reg;
    input [7:0]tmr0_reg;
    input [7:0]pc_low_reg;
    input [7:0]status_reg;
    input [7:0]fsr_reg;
    input [7:0]gpio0;
    input [7:0]gpio1;
    input [7:0]gpio2;
    output reg [7:0]sfr_mux_out;

    always @ (*)
    begin
        case (sfr_mux_select)
        `INDF:      sfr_mux_out = indf_reg;
        `TMR0:      sfr_mux_out = tmr0_reg;
        `PCL:       sfr_mux_out = pc_low_reg;
        `STATUS:    sfr_mux_out = status_reg;
        `FSR:       sfr_mux_out = fsr_reg;
        `GPIO0:     sfr_mux_out = gpio0;
        `GPIO1:     sfr_mux_out = gpio1;
        `GPIO2:     sfr_mux_out = gpio2;
        endcase
    end
endmodule