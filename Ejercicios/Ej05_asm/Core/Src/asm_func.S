.syntax unified
.thumb
.text

@ Simbolos exportados
.global asm_svc
.global asm_sum
.global asm_filtroVentana10
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

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@Prototipo en "C":
@ void asm_filtroVentana10(uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitudVectorIn);
@
@ Parametros recibidos:
@   r0: *vectorIn
@   r1: *vectorOut
@   r2: longitud
@
@ Constante para Assembler
.equ    WINDOWS_SIZE,10 @ Longitud de la ventana

.thumb_func
    asm_filtroVentana10:
        push {r4-r11}
        mov r5, r0
        mov r7, WINDOWS_SIZE
        mov r6, 0                  @ i = 0
        .vector_loop:
            mov r3, 0               @ suma parcial = 0
            mov r9, r6              @ j = i
            mov r5, r0
            .windows_loop:
                @ calculo el modulo sobre la longitud del vector: j % longitud
                udiv r10, r9, r2
                mul r11, r10, r2
                sub r10, r9, r11

                add r5, r0, r10, LSL #1     @ Incremento la direccion de la posicion a leer del VectorIn
                add r9, 1                   @ j++

                ldrh r4, [r5]       @ cargo el valor del VectorIn
                add r3, r4          @ lo sumo a lo acumulado

                subs r7, 1
                bne .windows_loop
            mov r7, WINDOWS_SIZE

            udiv r8, r3, r7         @ hago el promedio de los valores sumados
            strh r8, [r1], 2        @ lo guardo en el vector de salida
            add r6,1                @ i++
            cmp r2, r6              @ comparo si ya se alcanzo la longitud del vector
            bne .vector_loop
        pop {r4-r11}
        bx lr
