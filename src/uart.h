#include <stdint.h>

#ifndef UART_H
#define UART_H

void uart_init();
void uart_write(char str[]);
void uart_writen(char* str, int n);
void uart_writec(char data);
void uart_printhex8(uint8_t n);
void uart_printhex32(uint32_t n);

#endif
