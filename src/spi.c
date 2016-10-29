#include <msp430.h>
#include <stdint.h>
#include "spi.h"

volatile uint8_t spi_buf = 0;

#define SCLK    BIT5
#define SDI     BIT6
#define SDO     BIT7
#define CS      BIT0

//#define INT1    BIT4
//#define INT2    BIT3

void spi_init() {
  UCB0CTL1 = UCSWRST;
  UCB0CTL0 = UCSYNC + UCSPB + UCMSB + UCCKPH;   //UCMSB + UCMST + UCSYNC; // 3-pin, 8-bit SPI master
  UCB0CTL1 |= UCSSEL_2;                         // SMCLK
  UCB0BR0 = 0x02;                               // Frequency CPU / 2 (16Mhz / 2 = 8 Mhz SPI)
  UCB0BR1 = 0;

  P1SEL  |= SCLK | SDI | SDO;                   // P1.6 is MISO and P1.7 is MOSI
  P1SEL2 |= SCLK | SDI | SDO;

  P1DIR |= SCLK | SDO;
  P1DIR &= ~SDI;

  P2DIR |= CS;// | CS2;                           // P2.0 CS (chip select)
  P2OUT |= CS;// | CS2;

  //P1DIR &= ~(INT1 | INT2);                      // P1.4 and P1.3 as INT (INTERRUPT, not used yet)

  UCB0CTL1 &= ~UCSWRST;                         // Initialize USCI state machine
}

void spi_txready() {
  while (!(IFG2 & UCB0TXIFG)); // TX buffer ready?
}

void spi_rxready() {
  while (!(IFG2 & UCB0RXIFG)); // RX Received?
}

void spi_send(uint8_t data) {
  spi_txready();
  UCB0TXBUF = data;            // Send data over SPI to Slave
}

void spi_recv() {
  spi_rxready();
  spi_buf = UCB0RXBUF;         // Store received data
}

void spi_transfer(uint8_t data) {
  spi_send(data);
  spi_recv();
}

void spi_chipEnable() {
  P2OUT &= ~CS;
}

void spi_chipDisable() {
   P2OUT |= CS;
}
