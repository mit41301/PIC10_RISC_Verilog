     #include <p10F202.inc>

        ORG         0x00
start
        tris        0x05
        ;H
        movlw       0x48
        movwf       0x05
        BSF         0x05, 0x07
        BCF         0x05, 0x07
        ;e
        movlw       0x65
        movwf       0x05
        BSF         0x05, 0x07
        BCF         0x05, 0x07
        ;l
        movlw       0x6C
        movwf       0x05
        BSF         0x05, 0x07
        BCF         0x05, 0x07
        ;l
        movlw       0x6C
        movwf       0x05
        BSF         0x05, 0x07
        BCF         0x05, 0x07
        ;o
        movlw       0x6F
        movwf       0x05
        BSF         0x05, 0x07
        BCF         0x05, 0x07
        ;,
        movlw       0x2C
        movwf       0x05
        BSF         0x05, 0x07
        BCF         0x05, 0x07
        ;(space)
        movlw       0x20
        movwf       0x05
        BSF         0x05, 0x07
        BCF         0x05, 0x07
        ;W
        movlw       0x57
        movwf       0x05
        BSF         0x05, 0x07
        BCF         0x05, 0x07
        ;o
        movlw       0x6F
        movwf       0x05
        BSF         0x05, 0x07
        BCF         0x05, 0x07
        ;r
        movlw       0x72
        movwf       0x05
        BSF         0x05, 0x07
        BCF         0x05, 0x07
        ;l
        movlw       0x6C
        movwf       0x05
        BSF         0x05, 0x07
        BCF         0x05, 0x07
        ;d
        movlw       0x64
        movwf       0x05
        BSF         0x05, 0x07
        BCF         0x05, 0x07
        ;!
        movlw       0x21
        movwf       0x05
        BSF         0x05, 0x07
        BCF         0x05, 0x07
        goto        $
        END