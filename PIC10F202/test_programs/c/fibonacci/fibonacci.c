#include <htc.h>

#define number 7

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
    volatile unsigned char count = 0;
    volatile unsigned char first = 0;
    volatile unsigned char second = 1;
    volatile unsigned char next = 0;
    init();
    output_char = 'F';
    printc();
    output_char = 'i';
    printc();
    output_char = 'b';
    printc();
    output_char = 'o';
    printc();
    output_char = 'n';
    printc();
    output_char = 'a';
    printc();
    output_char = 'c';
    printc();
    output_char = 'c';
    printc();
    output_char = 'i';
    printc();
    output_char = ' ';
    printc();
    output_char = 'S';
    printc();
    output_char = 'e';
    printc();
    output_char = 'q';
    printc();
    output_char = 'u';
    printc();
    output_char = 'e';
    printc();
    output_char = 'n';
    printc();
    output_char = 'c';
    printc();
    output_char = 'e';
    printc();
    output_char = ':';
    printc();
    output_char = '\n';
    printc();
    
    for ( count = 0; count < number; count++)
    {
        if (count <= 1)
            next = count;
        else
        {
            next = first + second;
            first = second;
            second = next;
        }
        output_char = '0' + next;
        printc();
        output_char = '\n';
        printc();
    }
    while (1)
        ;
}

