module toplevel(CLOCK_50, KEY, SW, GPIO_1, LEDR, HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, VGA_R, VGA_G, VGA_B, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK);
    input CLOCK_50;
    input [3:0]KEY;
    input [9:0]SW;
    inout [15:0]GPIO_1;
    output [9:0]LEDR;
    output [6:0]HEX0;
    output [6:0]HEX1;
    output [6:0]HEX2;
    output [6:0]HEX3;
    output [6:0]HEX4;
    output [6:0]HEX5;
    output [7:0]VGA_R;
    output [7:0]VGA_G;
    output [7:0]VGA_B;
    output VGA_HS;
    output VGA_VS;
    output VGA_BLANK_N;
    output VGA_SYNC_N;
    output VGA_CLK;

    wire cpu_clk;
    wire [7:0]gpio_to_display;

    assign cpu_clk = SW[9] ? (KEY[1]) : CLOCK_50;

    cpu cpu_core
    (
        .clk(cpu_clk),
        .rst(~KEY[0]),
        .GPIO0(gpio_to_display),
        .GPIO1(GPIO_1[7:0]),
        .GPIO2(GPIO_1[15:8]),
        .SW(SW),
        .LEDR(LEDR),
        .HEX0(HEX0),
        .HEX1(HEX1),
        .HEX2(HEX2),
        .HEX3(HEX3),
        .HEX4(HEX4)
    );

    text_display display
    (
        .clk(CLOCK_50),
        .rst(~KEY[0]),
        .GPIO(gpio_to_display),
        .VGA_R(VGA_R),
        .VGA_G(VGA_G),
        .VGA_B(VGA_B),
        .VGA_HS(VGA_HS),
        .VGA_VS(VGA_VS),
        .VGA_BLANK_N(VGA_BLANK_N),
        .VGA_SYNC_N(VGA_SYNC_N),
        .VGA_CLK(VGA_CLK)
    );

endmodule