# msp430-radio

SX1276 transceiver driver implementation for MSP430G2553.

Driver was written from scratch using official [Semtech code](https://github.com/Lora-net/LoRaMac-node) of LoRaWAN endpoint stack implementation. Receiving and transmitting works properly.

In this implementation timers and most of sx1276 interrupts not used at all (although it necessary). So this code need to be supplemented before using in production.
