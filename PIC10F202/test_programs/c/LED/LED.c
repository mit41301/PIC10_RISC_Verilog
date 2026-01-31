#include <htc.h>

#define inteval 50000000

unsigned char led = 0;
unsigned char state = 0;
unsigned char input = 0;
void init()
{
#asm
    tris 0x06
#endasm
}

void blink()
{
#asm
    movlw   _led
    movwf   _FSR
    movf    _INDF,w
    movwf   0x06
#endasm
    _delay(inteval);
}

void read_sw()
{
#asm
    btfss 0x07,0
    goto    setzero
    movlw   _input
    movwf   _FSR
    movlw   0x01
    movwf   _INDF
    goto    finish
setzero
    movlw   _input
    movwf   _FSR
    movlw   0x00
    movwf   _INDF
finish
#endasm
}

void main(void)
{
    init();
    while (1)
    {
        if (state)
        {
            led = 1;
            blink();
            led = 2;
            blink();
            led = 4;
            blink();
            led = 0;
            blink();
            state = 0;
        }
        read_sw();
        if (input)
        {
            state = 1;
        }
    }
}