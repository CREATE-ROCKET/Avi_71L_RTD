#pragma once
#ifndef SLIP_
#define SLIP_
#include <Arduino.h>

// SLIP CODE
#define END_CODE 0xC0
#define ESC_CODE 0xDB
#define ESC_END 0xDC
#define ESC_ESC 0xDD

class SLIP
{
    HardwareSerial *ser;
    uint8_t tx_data[256];
    uint8_t tx_len = 0;

public:
    uint8_t rx_data[256];
    uint8_t rx_len = 0;
    void begin(HardwareSerial *serial);
    bool encode();
    bool decode();
    bool addtx(uint8_t data);
    bool addtx(int32_t data);
    bool addtx(uint8_t *data, uint8_t size);
    void sendMsg();
    void send();
    void send(uint8_t data);
    void send(int32_t data);
    void send(uint8_t *data, uint8_t size);
    bool receive();
    bool read();
};

#endif