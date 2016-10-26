#include <stdint.h>

#ifndef SPI_H
#define SPI_H

extern volatile uint8_t spi_buf;

void spi_init();

void spi_txready();
void spi_rxready();

void spi_send(uint8_t data);
void spi_recv();

void spi_transfer(uint8_t data);

void spi_chipEnable();
void spi_chipDisable();

#endif
