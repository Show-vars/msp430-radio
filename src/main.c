#include <msp430.h>
#include <stdint.h>
#include "mcu.h"
#include "uart.h"
#include "spi.h"
#include "sx1276.h"
#include "sx1276regs-fsk.h"

#define RF_FREQUENCY   434000000 // Hz

#define FSK_FDEV                          25e3      // Hz
#define FSK_DATARATE                      50e3      // bps
#define FSK_BANDWIDTH                     50e3      // Hz
#define FSK_AFC_BANDWIDTH                 83.333e3  // Hz
#define FSK_PREAMBLE_LENGTH               5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON         false

#define RX_TIMEOUT_VALUE                  1000
#define TX_OUTPUT_POWER                   14        // dBm
#define BUFFER_SIZE                       32 // Define the payload size here

uint8_t buffer[BUFFER_SIZE];

static radio_events_t radio_events;

void OnTxDone() {
  uart_write("GSV OUT\n");
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  uart_write("GSV IN:");
  uart_writen(payload, size);
  uart_write("\n");
}

void main(void) {
  mcu_init();

  P1DIR |= BIT0;

  uart_init();
  uart_write("\n\n");
  spi_init();

  uart_write("Init radio0: ");

  radio_events.TxDone = OnTxDone;
  radio_events.RxDone = OnRxDone;
  //radio_events.TxTimeout = OnTxTimeout;
  //radio_events.RxTimeout = OnRxTimeout;
  //radio_events.RxError = OnRxError;

  sx1276_init(radio_events);
  sx1276_set_channel(RF_FREQUENCY);

  sx1276_set_txconfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                  FSK_DATARATE, 0,
                                  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, 0, 3000);

  sx1276_set_rxconfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                  0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                                  0, 0, false, true);

  uart_write("done\n");

  sx1276_set_rx(0);

  while(1){mcu_delayms(100);}

/*
  buffer[0] = 'G';
  buffer[1] = 'S';
  buffer[2] = 'V';
  buffer[3] = '0';
  buffer[4] = '0';
  buffer[5] = '0';
  buffer[6] = '0';
  buffer[7] = '0';

  while(1) {
    P1OUT |= BIT0;
    sx1276_send(buffer, 3);

    mcu_delayms(100);
    P1OUT &= ~BIT0;
    mcu_delayms(1900);
  }
  */
}
