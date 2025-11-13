#ifndef UART2_H
#define UART2_H

#include "stm32f4xx.h"
void UART2_Init(void);
void uart2_send_string(const char *s);
void uart2_send_char(char c);
void uart2_send_int32(int32_t val);
#endif