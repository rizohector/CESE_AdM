#pragma once

#include <stdint.h>


void asm_svc (void);
uint32_t asm_sum (uint32_t firstOperand, uint32_t secondOperand);
void asm_pack32to16(int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud);
