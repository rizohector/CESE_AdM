#pragma once

#include <stdint.h>


void asm_svc (void);
uint32_t asm_sum (uint32_t firstOperand, uint32_t secondOperand);
int32_t asm_max (int32_t * vectorIn, uint32_t longitud);
