#include <htc.h>
unsigned char output_char = 0;

void init()
{
#asm
    tris 0x05
#endasm
}

void printc()
{
#asm
    movlw   _output_char
    movwf   _FSR
    movf    _INDF,w
    movwf   0x05
    bsf     0x05,0x07
    bcf     0x05,0x07
#endasm
}
void main(void)
{
    init();
    output_char = 'H';
    printc();
    output_char = 'e';
    printc();
    output_char = 'l';
    printc();
    output_char = 'l';
    printc();
    output_char = 'o';
    printc();
    output_char = ',';
    printc();
    output_char = ' ';
    printc();
    output_char = 'W';
    printc();
    output_char = 'o';
    printc();
    output_char = 'r';
    printc();
    output_char = 'l';
    printc();
    output_char = 'd';
    printc();
    output_char = '!';
    printc();
    while (1)
        ;
}

