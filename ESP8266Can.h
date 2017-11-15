
#pragma once

#include <Arduino.h>
#include <stdint.h>

/* per CANopen spec it's 87.5% at 500kbps some websites say :) */
#define BIT_SAMPLING_POINT (15.0f)

#define CAN_RX_BUFFERS     (32)
#define CAN_RX_BUFFER_SIZE (14) /* worst case: 1+11+3+4+64+15+1+2+10 bits */

/* use how many buffers at which size for I2S raw data.
   choose sizes 128..2048, dividable by 4
     min: 128, guess because of I2S internal RX FIFO has a depth of 128 * 32 bits (=128 bytes)
     max: SLC queue size is max 4096, but will crash with more than 2048
   */
#define CAN_BUFFER_SIZE    (1024)  
#define CAN_BUFFER_COUNT   (8)

#define CHECK_SIZE         (0x40)


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
    ~ESP8266Can();
    
    void StartRx();
    void StopRx();
    void Loop(void (*cbr)(uint16_t id, bool req, uint8_t length, uint8_t *payload, bool ack));
    can_error_t SendMessage(uint16_t id, uint8_t length, uint8_t *data, bool req_remote = false, bool self_ack = true);

    /* internal, make them private somewhen. ISR accesses some of them. */
    void StartI2S();
    void StopI2S();
    void RestartI2S();
    void InitI2S();
    
    void BuildCanFrame(uint8_t *buffer, uint16_t id, uint8_t length, uint8_t *data);
    uint32_t DecodeCanFrame(uint8_t *buffer, uint16_t *id, uint8_t *length, uint8_t *data, bool *req_remote, bool *ack, bool onlyBasic);
    
    uint32_t cyclesBit() { return (uint32_t)(F_CPU / _rate); }
    uint32_t cyclesSample() { return (uint32_t)(cyclesBit() * BIT_SAMPLING_POINT / 100.0f); }
    
    /* error counters Rx path */
    uint32_t RxSuccess = 0;
    uint32_t RxErrStuffBits = 0;
    uint32_t RxErrCrc = 0;
    uint32_t RxErrFormat = 0;
    uint32_t RxErrAck = 0;
    uint32_t RxQueueOverflows = 0;
    uint32_t RxErrors() { return RxErrStuffBits + RxErrCrc + RxErrFormat + RxErrAck; }
    uint32_t RxCount() { return RxSuccess; }
    
    /* error counters Tx path */
    uint32_t TxSuccess = 0;
    uint32_t TxErrLineBusy = 0;
    uint32_t TxErrTransceiver = 0;
    uint32_t TxErrNoAck = 0;
    uint32_t TxErrArbitration = 0;
    uint32_t TxErrCollision = 0;
    uint32_t TxErrors() { return TxErrLineBusy + TxErrTransceiver + TxErrNoAck + TxErrArbitration + TxErrCollision; }
    uint32_t TxCount() { return TxSuccess; }

    uint32_t IntLoadCount = 0;
    uint32_t IntLoads() { return IntErrorCount; }
    uint32_t IntErrorCount = 0;
    uint32_t IntErrors() { return IntErrorCount; }
    uint32_t BusLoadInternal = 0;
    uint32_t BusLoad() { return BusLoadInternal; }
    
    uint32_t LedPin = 0;
    struct slc_queue_item *CurrentQueueItem = NULL;
    
    /*
        CLK_I2S = 160MHz  / I2S_CLKM_DIV_NUM
        BCLK    = CLK_I2S / I2S_BCK_DIV_NUM
        WS      = BCLK/2  / (16 + I2S_BITS_MOD)
        I2S_CLKM_DIV_NUM - 5-127  must be >5 for I2S data
        I2S_BCK_DIV_NUM - 2-127
    */
    const uint32_t bestClkmDiv = 13;
    const uint32_t bestBckDiv = 8;
    bool _rxStarted = false; 
    bool _rxRunning = false; 
    
private:
    /* n queue items share the whole buffer */
    struct slc_queue_item I2SQueueTx[CAN_BUFFER_COUNT];
    uint8_t *I2SBufferTxData;
    
    bool _debug = false; 
    uint32_t _rate; 
    uint8_t _gpio_tx;
    uint8_t _gpio_rx;
    uint32_t _maxTries;
    uint8_t _canBuffer[14];
    
    uint32_t pinRegisterTx() { return _BV(_gpio_tx); }
    uint32_t pinRegisterRx() { return _BV(_gpio_rx); }

    void PrepareQueue(const char *name, struct slc_queue_item *queue, uint32_t queueLength, void *buffer, uint32_t eof);
    void CalcCRCBit(uint16_t *crc, uint16_t bit);
    uint16_t CalcCRC(uint8_t *data, uint16_t startBit, uint16_t bitCount);
    uint32_t BuildCanFrame(uint8_t *buffer, uint16_t id, uint8_t length, uint8_t *data, bool req_remote, bool self_ack);
};


