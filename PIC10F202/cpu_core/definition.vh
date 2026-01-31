`ifndef DEFINITION_VH
`define DEFINITION_VH

// The constants for status_register
`define STATUS_C    0
`define STATUS_DC   1
`define STATUS_Z    2

// The definition for the depth of the stack
`define STACK_DEPTH 2

// The definition for instructions
`define ADDWF   12'b000111xxxxxx
`define ANDWF   12'b000101xxxxxx
`define CLRF    12'b0000011xxxxx
`define CRLW    12'b000001000000
`define COMF    12'b001001xxxxxx
`define DECF    12'b000011xxxxxx
`define DECFSZ  12'b001011xxxxxx
`define INCF    12'b001010xxxxxx
`define INCFSZ  12'b001111xxxxxx
`define IORWF   12'b000100xxxxxx
`define MOVF    12'b001000xxxxxx
`define MOVWF   12'b0000001xxxxx
`define NOP     12'b000000000000
`define RLF     12'b001101xxxxxx
`define RRF     12'b001100xxxxxx
`define SUBWF   12'b000010xxxxxx
`define SWAPF   12'b001110xxxxxx
`define XORWF   12'b000110xxxxxx
`define BCF     12'b0100xxxxxxxx
`define BSF     12'b0101xxxxxxxx
`define BTFSC   12'b0110xxxxxxxx
`define BTFSS   12'b0111xxxxxxxx
`define ANDLW   12'b1110xxxxxxxx
`define CALL    12'b1001xxxxxxxx
`define CLRWDT  12'b000000000100
`define GOTO    12'b101xxxxxxxxx
`define IORLW   12'b1101xxxxxxxx
`define MOVLW   12'b1100xxxxxxxx
`define OPTION  12'b000000000010
`define RETLW   12'b1000xxxxxxxx
`define SLEEP   12'b000000000011
`define TRIS0   12'b000000000101 // GPIO0
`define TRIS1   12'b000000000110 // GPIO1
`define TRIS2   12'b000000000111 // GPIO2
`define XORLW   12'b1111xxxxxxxx

// The definition for sfr
`define INDF    3'd0
`define TMR0    3'd1
`define PCL     3'd2
`define STATUS  3'd3
`define FSR     3'd4
`define GPIO0   3'd5
`define GPIO1   3'd6
`define GPIO2   3'd7

`endif