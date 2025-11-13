#include "stm32f4xx.h"
#include <stdio.h>

#define UART_TX_PIN 2 // PA2
#define UART_RX_PIN 3 // PA3
#define UART_PORT   GPIOA
#define FREQUENCY   16000000UL // 16 MHz
#define BAUDRATE    115200

void UART2_Init(void) {
	// Enable GPIOA and USART2 clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	// Set PA2 (TX) and PA3 (RX) to alternate function AF7
	GPIOA->MODER &= ~((0x3 << (UART_TX_PIN * 2)) | (0x3 << (UART_RX_PIN * 2)));
	GPIOA->MODER |=  (0x2 << (UART_TX_PIN * 2)) | (0x2 << (UART_RX_PIN * 2));
	GPIOA->AFR[0] &= ~((0xF << (UART_TX_PIN * 4)) | (0xF << (UART_RX_PIN * 4)));
	GPIOA->AFR[0] |=  (0x7 << (UART_TX_PIN * 4)) | (0x7 << (UART_RX_PIN * 4));

	// Configure USART2: 115200 baud, 8N1
	USART2->BRR = FREQUENCY / BAUDRATE; // Assuming 16 MHz clock
	USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable TX, RX, USART
}

void uart2_send_char(char c) {
	while (!(USART2->SR & USART_SR_TXE));
	USART2->DR = c;
}

void uart2_send_string(const char *s) {
	while (*s) uart2_send_char(*s++);
}

void uart2_send_int32(int32_t val) {
	char buf[12];
	sprintf(buf, "%ld", (long)val);
	uart2_send_string(buf);
}