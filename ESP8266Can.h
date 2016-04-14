
#pragma once

#include <Arduino.h>
#include <stdint.h>

/* per CANopen spec it's 87.5% at 500kbps some websites say :) */
#define BIT_SAMPLING_POINT (15.0f)

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


struct slc_queue_item 
{
    uint32  blocksize:12;
    uint32  datalen:12;
    uint32  unused:5;
    uint32  sub_sof:1;
    uint32  eof:1;
    uint32  owner:1;
    uint32  buf_ptr;
    uint32  next_link_ptr;
};

class ESP8266Can
{
public:
    ESP8266Can(uint32_t rate, uint8_t gpio_tx, uint8_t gpio_rx);
    void StartI2S();
    void StopI2S();
    void InitI2S();
    void StartRx();
    void Loop();
    ~ESP8266Can();
    can_error_t SendMessage(uint16_t id, uint8_t length, uint8_t *data, bool req_remote = false, bool self_ack = true);
    void BuildCanFrame(uint8_t *buffer, uint16_t id, uint8_t length, uint8_t *data);
    
    uint32_t cyclesBit() { return (uint32_t)(F_CPU / _rate); }
    uint32_t cyclesSample() { return (uint32_t)(cyclesBit() * BIT_SAMPLING_POINT / 100.0f); }
    
    uint32_t IntCount = 0;

private:
    /* n queue items share the whole buffer */
    struct slc_queue_item I2SQueueRx[8];
    struct slc_queue_item I2SQueueTx[8];
    uint8_t I2SBufferTxData[8192];
    uint8_t I2SBufferRxData[8192];
    
    /*
        CLK_I2S = 160MHz  / I2S_CLKM_DIV_NUM
        BCLK    = CLK_I2S / I2S_BCK_DIV_NUM
        WS      = BCLK/2  / (16 + I2S_BITS_MOD)
        I2S_CLKM_DIV_NUM - 5-127  must be >5 for I2S data
        I2S_BCK_DIV_NUM - 2-127
    */
    uint32_t bestClkmDiv = 8;
    uint32_t bestBckDiv = 4;
    
    uint32_t _rate; 
    uint32_t _oversampling; 
    uint8_t _gpio_tx;
    uint8_t _gpio_rx;
    uint32_t _maxTries;
    uint8_t _canBuffer[14];
    
    uint32_t pinRegisterTx() { return _BV(_gpio_tx); }
    uint32_t pinRegisterRx() { return _BV(_gpio_rx); }

    void PrepareQueue(const char *name, struct slc_queue_item *queue, uint32_t queueLength, void *buffer, uint32_t bufferLength, uint32_t eof);
    void CalcCRCBit(uint16_t *crc, uint16_t bit);
    uint16_t CalcCRC(uint8_t *data, uint16_t startBit, uint16_t bitCount);
    uint32_t BuildCanFrame(uint8_t *buffer, uint16_t id, uint8_t length, uint8_t *data, bool req_remote, bool self_ack);
};


