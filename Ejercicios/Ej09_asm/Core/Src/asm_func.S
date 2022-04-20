.syntax unified
.thumb
.text

@ Simbolos exportados
.global asm_svc
.global asm_sum
.global asm_invertir
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



@   void invertir (uint16_t * vector, uint32_t longitud)
@
@ Parametros recibidos:
@   r0: *vectorIn
@   r1: longitud
@
.thumb_func
    asm_invertir:
        @ Registros usados
        @   r2: utilizado para recorrer el vector de atras hacia adelante
        @   r3: utilizado para guardar el valor de adelante
        @   r4: utilizado para guardar el valor de atras

        push {r4}
        mov r2, r0              @ hago una copia de la direccion del vector
        add r2, r2, r1, lsl 1   @ me posiciono en la ultima posicion
        sub r2, 2               @ la ultima posicion esta en longitud-1
        lsr r1, 1               @ La cantidad de iteraciones es la mitad de la longitud ya que
                                @ voy cambiando los valores de adelante hacia atras y atras hacia
                                @ adelante en cada ciclo.
        .ej9_loop:
            ldrh r3, [r0]        @ guardo el valor de adelante en una variable temporal
            ldrh r4, [r2]        @ guardo el valor de atras en una variable temporal
            strh r4, [r0], 2     @ el valor de atras pasa adelante
            strh r3, [r2], -2    @ el valor de adelante pasa atras
            subs r1, 1
            bne .ej9_loop
        pop {r4}
        bx lr


