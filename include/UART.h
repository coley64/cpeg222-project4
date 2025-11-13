#ifndef UART_H
#define UART_H

#include "stm32f4xx.h"
void configure_uart(void);
void uart_send_string(const char *s);
void uart_send_char(char c);
void uart_send_int32(int32_t val);
#endif