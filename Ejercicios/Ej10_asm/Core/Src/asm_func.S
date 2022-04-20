.syntax unified
.thumb
.text

@ Simbolos exportados
.global asm_svc
.global asm_sum
.global asm_eco
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void asm_svc (void)
@
.thumb_func
    asm_svc:
        svc 0
        bx lr


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   uint32_t asmSum (uint32_t firstOperand, uint32_t secondOperand)
@
@ Parametros recibidos:
@   r0: firstOperand
@   r1: secondOperand
@
@ Valor de retorno:
@   r0: resultado de la suma de firstOperand y secondOperand
@
.thumb_func
    asm_sum:
        add r0, r1  @ r0 = r0 + r1
        bx lr       @ vuelve adonde fue llamada (especificamente, si "pc"
                    @ es la direccion, en el codigo del programa, de la
                    @ instruccion "bl asm_sum" usada para saltar a
                    @ esta funcion, entonces "lr" contiene ese "pc" + 4).
                    @ el valor de retorno debe quedar en el registro r0.
                    @ En este caso, r0 es el resultado de r0 + r1.

        @ Otras alternativas para el retorno
        @ mov pc,lr
        @ bx lr
        @ push {lr} ... pop {pc}



@   void c_eco (uint16_t * vector, uint32_t longitud)
@
@ Parametros recibidos:
@   r0: *vectorIn
@   r1: longitud
@
.equ    SAMPLE_DELAY,882

.thumb_func
    asm_eco:
        push {r4-r6}
        sub r1, 2
        mov r3, SAMPLE_DELAY
        mov r6, 0
        add r0, r0, r1, lsl 1   @ me posiciono en la ultima posicion del vector
        sub r3, r0, r3, lsl 1   @ calculo la posicion del eco
        sub r1, SAMPLE_DELAY    @ las primeras SAMPLE_DELAY muestras no se le agrega eco
        .ej10_loop:
            ldr r4, [r0]        @ cargo la ultima posicion del vector
            ldr r5, [r3], -4    @ cargo la posicion del eco
            shadd16 r5, r5, r6  @ r5/2
            sadd16 r4, r4, r5   @ sumo la muestra con el eco
            str r4, [r0], -4    @ lo guardo en el vector original
            subs r1, 2
            bne .ej10_loop
        pop {r4-r6}
        bx lr
