// This is part of the ECE241 final project
// Date created: November 18 2016
// This file contains the controller for cpu

module cpu_controller(clk, rst, reg_address, instruction_reg_out, zero_result, store_alu_w, alu_in_select, load_status_reg, skip_next_instruction, load_instruction_reg, pc_mux_select, load_pc, inc_pc, inc_stack, dec_stack, load_stack, load_fsr, reg_address_mux_select, load_ram, load_tris0, load_tris1, load_tris2, load_gpio0, load_gpio1, load_gpio2);
    `include "definition.vh"
    input clk;
    input rst;
    input [4:0]reg_address;
    input [11:0]instruction_reg_out;
    input zero_result;

    output reg store_alu_w;
    output alu_in_select;
    output reg load_status_reg;
    output reg skip_next_instruction; 
    output reg load_instruction_reg;
    output reg [1:0]pc_mux_select;
    output reg load_pc;
    output reg inc_pc;
    output reg inc_stack;
    output reg dec_stack;
    output reg load_stack;
    output reg load_fsr;
    output reg_address_mux_select;
    output reg load_ram;

    output reg load_tris0;
    output reg load_tris1;
    output reg load_tris2;
    output reg load_gpio0;
    output reg load_gpio1;
    output reg load_gpio2;

    reg initial_ir_load_done = 1'b0;

    // Select signal for address mux, direct mode/indirect mode
    // select = 0 direct
    // select = 1 indirect
    assign reg_address_mux_select = (instruction_reg_out[4:0] == 5'b0);

    // Select signal for alu input mux
    // select = 0 sfr
    // select = 1 data_reg
    // !!! Can only indirect access 24 data registers!!!
    // This might be a structural problem
    assign alu_in_select = (instruction_reg_out[4:0] > 5'd7) || (instruction_reg_out[4:0] == 5'b0);

    reg call_cycle = 1'b0; // The extra cycle for call
    reg goto_cycle = 1'b0; // The extra cycle for goto
    reg retlw_cycle1 = 1'b0;
    reg retlw_cycle2 = 1'b0;

    // negedge trigger, because we want to make sure thing are setup before posedge
    always @ (negedge clk)
    begin
        store_alu_w <= 0;
        load_status_reg <= 0;
        skip_next_instruction <= 0; 
        load_instruction_reg <= 0;
        pc_mux_select <= 0;
        load_pc <= 0;
        inc_pc <= 0;
        inc_stack <= 0;
        dec_stack <= 0;
        load_stack <= 0;
        load_fsr <= 0;
        load_ram <= 0;
        load_tris0 <= 0;
        load_tris1 <= 0;
        load_tris2 <= 0;
        load_gpio0 <= 0;
        load_gpio1 <= 0;
        load_gpio2 <= 0;

        if (rst)
            initial_ir_load_done = 1'b0;
        else if ((!rst) && (!initial_ir_load_done))
        begin
            load_instruction_reg <= 1'b1;
            inc_pc <= 1'b1;
            initial_ir_load_done <= 1'b1;
        end
        else if(call_cycle|goto_cycle|retlw_cycle1|retlw_cycle2)
        begin
            if (call_cycle)
            begin
                load_instruction_reg <= 1'b1;
                inc_pc <=1'b1;
                inc_stack <= 1'b1;
                call_cycle <= 1'b0;
            end
            if (goto_cycle)
            begin
                load_instruction_reg <= 1'b1;
                inc_pc <=1'b1;
                goto_cycle <= 1'b0;
            end
            if (retlw_cycle1)
            begin
                load_pc <= 1'b1;
                retlw_cycle2 <= 1'b1;
                retlw_cycle1 <= 1'b0;
            end
            if (retlw_cycle2)
            begin
                load_instruction_reg <= 1'b1;
                inc_pc <=1'b1;
                store_alu_w <= 1'b1;
                retlw_cycle2 <= 1'b0;
            end
        end
        else
        begin
            load_instruction_reg <= 1'b1;
            inc_pc <=1'b1;
            casex (instruction_reg_out)
            `ADDWF   :
            begin
                load_register();
            end
            `ANDWF   :
            begin
                load_register();
            end
            `CLRF    :
            begin
                load_register();
            end
            `CRLW    :
            begin
                load_register();
            end
            `COMF    :
            begin
                load_register();
            end
            `DECF    :
            begin
                load_register();
            end
            `DECFSZ  :
            begin
                skip_next_instruction <= zero_result;
                load_register();
            end
            `INCF    :
            begin
                load_register();
            end
            `INCFSZ  :
            begin
                skip_next_instruction <= zero_result;
                load_register();
            end
            `IORWF   :
            begin
                load_register();
            end
            `MOVF    :
            begin
                load_register();
            end
            `MOVWF   :
            begin
                load_register();
            end
            `NOP     :
            begin
                //DO nothing
            end
            `RLF     :
            begin
                load_register();
            end
            `RRF     :
            begin
                load_register();
            end
            `SUBWF   :
            begin
                load_register();
            end
            `SWAPF   :
            begin
                load_register();
            end
            `XORWF   :
            begin
                load_register();
            end
            `BCF     :
            begin
                load_f_register();
            end
            `BSF     :
            begin
                load_f_register();
            end
            `BTFSC   :
            begin
                skip_next_instruction <= zero_result;
            end
            `BTFSS   :
            begin
                skip_next_instruction <= !zero_result;
            end
            `ANDLW   :
            begin
                store_alu_w <= 1'b1;
            end
            `CALL    :
            begin
                pc_mux_select <= 2'd1; // load from alu
                inc_pc <= 1'b0;
                load_pc <= 1'b1;
                load_stack <= 1'b1;
                load_instruction_reg <= 1'b0; // Don't need to load the next instruction
                call_cycle <= 1'b1;
            end
            `CLRWDT  :
            begin
                // WDT not implemented in this design
            end
            `GOTO    :
            begin
                pc_mux_select <= 2'd2; // load from instruction_reg
                inc_pc <= 1'b0;
                load_pc <= 1'b1;
                load_instruction_reg <= 1'b0; // Don't need to load the next instruction
                goto_cycle <= 1'b1;
            end
            `IORLW   :
            begin
                store_alu_w <= 1'b1;
            end
            `MOVLW   :
            begin
                store_alu_w <= 1'b1;
            end
            `OPTION  :
            begin
                // OPTION not implemented in this design
            end
            `RETLW   :
            begin
                dec_stack <= 1'b1;
                load_instruction_reg <= 1'b0; // Don't need to load the next instruction
                retlw_cycle1 <= 1'b1;
            end
            `SLEEP   :
            begin
                // SLEEP not implemented in this design
            end
            `TRIS0   :
            begin
                load_tris0 <= 1'b1;
            end
            `TRIS1   :
            begin
                load_tris1 <= 1'b1;
            end
            `TRIS2   :
            begin
                load_tris2 <= 1'b1;
            end
            `XORLW   :
            begin
                store_alu_w <= 1'b1;
            end
            default: ; //DO nothing
            endcase
        end
    end

    task load_register();
    begin
        // if instruction_reg_out[5] == 0 load w accumulator
        if(instruction_reg_out[5] == 0)
            store_alu_w <= 1'b1;
        else
            // The result should be stored to f registers
            load_f_register();
        end
    endtask


    task load_f_register();
    begin
        // Choose from any of the 32 registers
        case(reg_address)
            5'd0:
                // INDF. Do Nothing since INDF is not writable.
                ;
            5'd1:
                // TMR0. Do Nothing since TMR0 is not implemented.
                ;
            5'd2:
                // PCL
                load_pc <= 1'b1;
            5'd3:
                // STATUS.
                load_status_reg <= 1'b1;
            5'd4:
                // FSR.
                load_fsr <= 1'b1;
            5'd5:
                // GPIO0.
                load_gpio0 <= 1'b1;
            5'd6:
                // GPIO1.
                load_gpio1 <= 1'b1;
            5'd7:
                // GPIO2.
                load_gpio2 <= 1'b1;
            default:
                // RAM Register.
                load_ram <= 1'b1;
        endcase
    end
    endtask
endmodule
