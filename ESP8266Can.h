
#pragma once

#include <Arduino.h>
#include <stdint.h>

/* per CANopen spec it's 87.5% at 500kbps some websites say :) */
#define BIT_SAMPLING_POINT (87.5f)

/* error/status codes for SendMessage and internal functions */
typedef enum
{
    ERR_OK          = 0xFFFFFF00,
    ERR_BUSY_LINE   = 0xFFFFFF01,
    ERR_ARBRITATION = 0xFFFFFF02,
    ERR_COLLISION   = 0xFFFFFF03,
    ERR_NO_ACK      = 0xFFFFFF04,
    ERR_TRCV_ERR    = 0xFFFFFF05
} can_error_t;


class ESP8266Can
{
public:
    ESP8266Can(uint32_t rate, uint8_t gpio_tx, uint8_t gpio_rx);
    ~ESP8266Can();
    can_error_t SendMessage(uint16_t id, uint8_t length, uint8_t *data, bool req_remote = false, bool self_ack = true);
    void BuildCanFrame(uint8_t *buffer, uint16_t id, uint8_t length, uint8_t *data);

private:
    uint32_t _rate; 
    uint8_t _gpio_tx;
    uint8_t _gpio_rx;
    uint32_t _maxTries;
    uint8_t _canBuffer[14];
    
    uint32_t cyclesBit() { return F_CPU / _rate; }
    uint32_t cyclesSample() { return (uint32_t)(cyclesBit() * BIT_SAMPLING_POINT / 100.0f); }
    uint32_t pinRegisterTx() { return _BV(_gpio_tx); }
    uint32_t pinRegisterRx() { return _BV(_gpio_rx); }

    void CalcCRCBit(uint16_t *crc, uint16_t bit);
    uint16_t CalcCRC(uint8_t *data, uint16_t startBit, uint16_t bitCount);
    uint32_t BuildCanFrame(uint8_t *buffer, uint16_t id, uint8_t length, uint8_t *data, bool req_remote, bool self_ack);
};


