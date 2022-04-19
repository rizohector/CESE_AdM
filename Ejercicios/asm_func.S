

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
@ Prototipo en "C":
@   void asm_svc (void)
@
.thumb_func
    asm_svc:
        svc 0
        bx lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void asm_zero (uint32_t *vector, uint32_t longitud)
@
@ Parametros recibidos:
@   r0: *vector
@   r1: longitud
@
.thumb_func
    asm_zeros:
        @ Registros usados
        @   r2: valor cero
        mov r2, 0
    .loop:
        str r2, [r0], 4    @ sizeof(uint32_t) == 4 , *vector++ = r2
        subs r1, 1           @ --longitud        
        bne .loop
        bx lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void asm_productoEscalar32 (uint32_t * vectorIn, uint32_t * vectorOut, uint32_t longitud, uint32_t escalar)
@
@ Parametros recibidos:
@   r0: *vectorIn
@   r1: *vectorOut
@   r2: longitud
@   r3: escalar
@
.thumb_func
    asm_productoEscalar32:
        @ Registros usados
        @   r4: resultado de la multiplicacion
        @   r5: contenido de cada posicion del vectorIn
        push {r4, r5}
        .ej2_loop:
            ldr r5, [r0], 4     @ r5 = *r0; r0+=4
            mul r4, r5, r3      @ r4 = r5*r3
            str r4, [r1], 4     @ *r1 = r4; r1+=4
            subs r2, 1          @ r2-=1
            bne .ej2_loop
        pop {r4, r5}
        bx lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void asm_productoEscalar16 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar)
@
@ Parametros recibidos:
@   r0: *vectorIn
@   r1: *vectorOut
@   r2: longitud
@   r3: escalar
@
.thumb_func
    asm_productoEscalar16:
        @ Registros usados
        @   r4: resultado de la multiplicacion
        push {r4}
        .ej3_loop:
            ldrh r4, [r0], 2    @ r4 = *r0; r0+=2 (half word)
            mul r4, r3          @ r4 = r4*r3
            strh r4, [r1], 2    @ *r1 = r4; r1+=2 (half word)
            subs r2, 1          @ r2-=1
            bne .ej3_loop
        pop {r4}
        bx lr            

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void asm_productoEscalar12 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar)
@
@ Parametros recibidos:
@   r0: *vectorIn
@   r1: *vectorOut
@   r2: longitud
@   r3: escalar
@
.thumb_func
    asm_productoEscalar12:
        @ Registros usados
        @   r4: resultado de la multiplicacion
        @   r5: aulixiar para comparar contra cero
        push {r4, r5}
        mov r5, 0
        .ej4_loop:
            ldrh r4, [r0], 2    @ r4 = *r0; r0+=2 (half word)
            mul r4, r3          @ r4 = r4*r3
            cmp r5, r4, lsr #12 @ if(r4 >> 12 == 0)
            beq .not_saturate
            mov r4, #4095
        .not_saturate:
            strh r4, [r1], 2    @ *r1 = r4; r1+=2 (half word)
            subs r2, 1          @ r2-=1
            bne .ej4_loop
        pop {r4, r5}
        bx lr    


