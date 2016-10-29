#include <msp430.h>
#include "uart.h"
#include "mcu.h"

void uart_init() {
  P1SEL  |=  BIT1 + BIT2;            // P1.1 UCA0RXD input
  P1SEL2 |=  BIT1 + BIT2;            // P1.2 UCA0TXD output

  UCA0CTL1 |=  UCSSEL_2 + UCSWRST;  // USCI Clock = SMCLK,USCI_A0 disabled

  UCA0BR0   =  0x82;                // 0x0682 From datasheet table
  UCA0BR1   =  0x06;                // selects baudrate = 9600bps, clk = SMCLK

  UCA0MCTL  =  UCBRS_1;             // Modulation value = 1 from datasheet

  UCA0STAT |=  UCLISTEN;            // Loop back mode enabled

  UCA0CTL1 &= ~UCSWRST;             // Clear UCSWRST to enable USCI_A0
}

void uart_write(char* str) {
  for(int i = 0; str[i] != '\0'; i++) {
    while (!(IFG2 & UCA0TXIFG));    // TX buffer ready?
    UCA0TXBUF = str[i];
  }
}

void uart_writen(char* data, int n) {
  while(n--) {
    while (!(IFG2 & UCA0TXIFG));
    UCA0TXBUF = *data++;
  }
}

void uart_writec(char data) {
  while (!(IFG2 & UCA0TXIFG));
  UCA0TXBUF = data;
}

void uart_printhex8(uint8_t n) {
  char buf[2 + 1];
  char *str = &buf[3 - 1];

  *str = '\0';

  uint8_t base = 16;

  do {
    uint8_t m = n;
    n /= base;
    char c = m - base * n;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);

  uart_write(str);
}

void uart_printhex32(uint32_t n) {
  char buf[8 + 1];
  char *str = &buf[9 - 1];

  *str = '\0';

  uint32_t base = 16;

  do {
    uint32_t m = n;
    n /= base;
    char c = m - base * n;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);

  uart_write(str);
}
