#ifndef SX1276REGISTERS_H
#define SX1276REGISTERS_H

// SX1276 registers
#define COM_RegFifo                      0x00 // FIFO r/w access
#define COM_RegOpMode                    0x01 // LoRa/FSK

#define FSK_RegBitrateMsb                0x02
#define LOR_Reserved02                   0x02

#define FSK_RegBitrateLsb                0x03
#define LOR_Reserved03                   0x03

#define FSK_RegFdevMsb                   0x04 // freq deviation
#define LOR_Reserved04                   0x04

#define FSK_RegFdevLsb                   0x05
#define LOR_Reserved05                   0x05

#define COM_RegFrfMsb                    0x06 // carrier freq
#define COM_RegFrfMid                    0x07
#define COM_RegFrfLsb                    0x08
#define COM_RegPaConfig                  0x09
#define COM_RegPaRamp                    0x0a

#define COM_RegOcp                       0x0b // overcurrent protection
#define COM_RegLna                       0x0c

#define FSK_RegRxConfig                  0x0d
#define LOR_RegFifoAddrPtr               0x0d

#define FSK_RegRssiConfg                 0x0e
#define LOR_RegFifoTxBaseAddr            0x0e

#define FSK_RegRssiCollision             0x0f
#define LOR_RegFifoRxBaseAddr            0x0f

#define FSK_RegRssiThresh                0x10
#define LOR_RegFifoRxCurrentAddr         0x10

#define FSK_RegRssiValue                 0x11
#define LOR_RegIrqFlagsMask              0x11

#define FSK_RegRxBw                      0x12
#define LOR_RegIrqFlags                  0x12

#define FSK_RegAfcBw                     0x13 // automatic freq cntrl
#define LOR_RegRxNbBytes                 0x13 // received pkt len

#define FSK_RegOokPeak                   0x14
#define LOR_RegRxHeaderCntValueMsb       0x14

#define FSK_RegOokFix                    0x15
#define LOR_RegRxHeaderCntValueLsb       0x15

#define FSK_RegOokAvg                    0x16
#define LOR_RegRxPacketCntValueMsb       0x16

#define FSK_Reserved17                   0x17
#define LOR_RegRxPacketCntValueLsb       0x17

#define FSK_Reserved18                   0x18
#define LOR_RegModemStat                 0x18

#define FSK_Reserved19                   0x19
#define LOR_RegPktSnrValue               0x19

#define FSK_RegAfcFei                    0x1a
#define LOR_RegPktRssiValue              0x1a

#define FSK_RegAfcMsb                    0x1b
#define LOR_RegRssiValue                 0x1b

#define FSK_RegAfcLsb                    0x1c
#define LOR_RegHopChannel                0x1c

#define FSK_RegFeiMsb                    0x1d
#define LOR_RegModemConfig1              0x1d

#define FSK_RegFeiLsb                    0x1e
#define LOR_RegModemConfig2              0x1e

#define FSK_RegPreambleDetect            0x1f
#define LOR_RegSymbTimeoutLsb            0x1f

#define FSK_RegRxTimeout1                0x20
#define LOR_RegPreambleMsb               0x20

#define FSK_RegRxTimeout2                0x21
#define LOR_RegPreambleLsb               0x21

#define FSK_RegRxTimeout3                0x22
#define LOR_RegPayloadLength             0x22

#define FSK_RegRxDelay                   0x23
#define LOR_RegMaxPayloadLength          0x23

#define FSK_RegOsc                       0x24
#define LOR_RegHopPeriod                 0x24

#define FSK_RegPreambleMsb               0x25
#define LOR_RegFifoRxByteAddr            0x25

#define FSK_RegPreambleLsb               0x26
#define LOR_RegModemConfig3              0x26

#define FSK_RegSyncConfig                0x27
#define LOR_Reserved27                   0x27

#define FSK_RegSyncValue1                0x28
#define LOR_RegFeiMsb                    0x28

#define FSK_RegSyncValue2                0x29
#define LOR_RegFeiMid                    0x29

#define FSK_RegSyncValue3                0x2a
#define LOR_RegFeiLsb                    0x2a

#define FSK_RegSyncValue4                0x2b
#define LOR_Reserved2b                   0x2b

#define FSK_RegSyncValue5                0x2c
#define LOR_RegRssiWideband              0x2c

#define FSK_RegSyncValue6                0x2d
#define LOR_Reserved2d                   0x2d

#define FSK_RegSyncValue7                0x2e
#define LOR_Reserved2e                   0x2e

#define FSK_RegSyncValue8                0x2f
#define LOR_Reserved2f                   0x2f

#define FSK_RegPacketConfig1             0x30
#define LOR_Reserved30                   0x30

#define FSK_RegPacketConfig2             0x31
#define LOR_RegDetectOptimize            0x31

#define FSK_RegPayloadLength             0x32
#define LOR_Reserved32                   0x32

#define FSK_RegNodeAddr                  0x33
#define LOR_RegInvertIQ                  0x33

#define FSK_RegBroadcastAddr             0x34
#define LOR_Reserved34                   0x34

#define FSK_RegFifoThresh                0x35
#define LOR_Reserved35                   0x35

#define FSK_RegSeqConfig1                0x36
#define LOR_Reserved36                   0x36

#define FSK_RegSeqConfig2                0x37
#define LOR_RegDetectionThreshold        0x37

#define FSK_RegTimerResol                0x38
#define LOR_Reserved38                   0x38

#define FSK_RegTimer1Coeff               0x39
#define LOR_RegSyncWord                  0x39

#define FSK_RegTimer2Coeff               0x3a
#define LOR_Reserved3a                   0x3a

#define FSK_RegImageCal                  0x3b
#define LOR_Reserved3b                   0x3b // reserved (in datasheet)?
#define LOR_RegInvertIQ2                 0x3b // does not exist in datasheet
                                         // but used in Semtech code.
                                         // UNDOCUMENTED

#define FSK_RegTemp                      0x3c
#define LOR_Reserved3c                   0x3c

#define FSK_RegLowBat                    0x3d
#define LOR_Reserved3d                   0x3d

#define FSK_RegIrqFlags1                 0x3e
#define LOR_Reserved3e                   0x3e

#define FSK_RegIrqFlags2                 0x3f
#define LOR_Reserved3f                   0x3f

#define COM_RegDioMapping1               0x40 // DIO0-DIO3
#define COM_RegDioMapping2               0x41 // DIO4-DIO5, clk out freq

#define COM_RegVersion                   0x42 // Semtech ID (silicon revision)

// 0x43 reserved

// The data sheet says this is FSK only, but the semtech code
// implies this is only valid for LoRa.  So for now, assume the
// datasheet is wrong.
//
// #define FSK_RegPllHop                    0x44
// #define LOR_Reserved44                   0x44

#define FSK_Reserved44                   0x44
#define LOR_RegPllHop                    0x44

// 0x45-0x4a reserved

#define COM_RegTcxo                      0x4b

// 0x4c reserved

#define COM_RegPaDac                     0x4d

// 0x4e-0x5a reserved

#define COM_RegFormerTemp                0x5b

// 0x5c reserved

#define FSK_RegBitRateFrac               0x5d
#define LOR_Reserved5d                   0x5d // reserved

// 0x5e-0x60 reserved

#define COM_RegAgcRef                    0x61
#define COM_RegAgcThresh1                0x62
#define COM_RegAgcThresh2                0x63
#define COM_RegAgcThresh3                0x64

// 0x65-0x6f reserved

#define COM_RegPll                       0x70

// SPI Single operation masks
#define SX1276_REG_READ     0x7f
#define SX1276_REG_WRITE    0x80

#endif
