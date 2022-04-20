.syntax unified
.thumb
.text

@ Simbolos exportados
.global asm_svc
.global asm_sum
.global asm_max
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



@  int32_t max (int32_t * vectorIn, uint32_t longitud)
@
@ Parametros recibidos:
@   r0: *vectorIn
@   r1: longitud
@
@   return r0

.thumb_func
    asm_max:
        @ Registros usados
        @   r2: maximo temporal
        @   r3: posicion del maximo
        @   r5: indice del vector
        push {r4, r5}
        ldr r2, [r0]            @ inicializo el maximo con el valor de la primera posicion del vector
        mov r3, 0               @ inicializo en 0
        mov r5, 0               @ inicializo en 0
        .ej7_loop:
            ldr r4, [r0], 4     @ guardo el valor del vector de entrada
            cmp r4, r2          @ comparo si el valor leido es mayor al maximo temporal
            blt .is_less
            mov r2, r4          @ si es mayor guardo el nuevo maximo
            mov r3, r5          @ actualizo la posicion del maximo
        .is_less:
            add r5, 1           @ incremento el indice del vector
            subs r1, 1          @ decremento el contador del loop
            bne .ej7_loop
        mov r0, r3            @ return max pos
        pop {r4, r5}
        bx lr



