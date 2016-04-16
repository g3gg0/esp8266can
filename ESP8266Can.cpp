/*
ESP8266Can.cpp - CAN library for ESP8266 using cycle count
Copyright (c) 2016 by g3gg0.de. All right reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#if defined(ESP8266)

#include <Arduino.h>
#include <ESP8266Can.h>
#include <eagle_soc.h>

#define LEADING_BITS            (5U)
#define BASEFREQ                (160000000L)

#define COUNT(x) (sizeof(x) / sizeof((x)[0]))

#define CLEAR_SET_REG_POS(reg,pos,mask,set) \
    do { \
        CLEAR_PERI_REG_MASK(reg,(mask)<<(pos)); \
        SET_PERI_REG_MASK(reg, (((uint32_t)(set))&(mask))<<(pos)); \
    } while(0)
        
    
/* intentionally keeping them as global variable because having them in class makes the code slower. 
   Defining as class members has no benefit as we can only be instanciated once anyway. */
static uint32_t InterruptReceiveBuffers[INT_RX_BUFFERS][INT_RX_BUFFER_SIZE];
static uint8_t ReceiveBuffersReadNum = 0;
static uint8_t ReceiveBuffersWriteNum = 0;
static uint8_t ReceiveBuffersWriteEntry = 0;
static uint32_t InterruptRxCount = 0;
static uint32_t InterruptTxCount = 0;
static uint32_t RxOversampling; 

extern "C"
{    
    #include "i2s_reg.h"
    #include "i2s.h"
    #include "eagle_soc.h"
    #include "esp8266_peri.h"
    #include "slc_register.h"
    
    extern void rom_i2c_writeReg_Mask(uint32_t block, uint32_t host_id, uint32_t reg_add, uint32_t Msb, uint32_t Lsb, uint32_t indata);

    static inline uint32_t _getCycleCountRaw()
    {
        uint32_t ccount;
        
        __asm__ __volatile__("rsr %0,ccount":"=a" (ccount));
        
        return ccount;
    }
    
    static inline uint64_t _getCycleCount()
    {
        static uint32_t lastValue = 0;
        static uint32_t overflows = 0;
        uint32_t ccount;
        
        __asm__ __volatile__("rsr %0,ccount":"=a" (ccount));
        
        if(ccount < lastValue)
        {
            overflows++;
        }
        lastValue = ccount;
        
        return (((uint64_t)overflows) << 32) | lastValue;
    }

    can_error_t ICACHE_RAM_ATTR sendRawData(uint8_t* buffer, uint32_t cyclesBit, uint32_t cyclesSample, uint16_t skip, uint16_t bits, uint8_t pinRegisterTx, uint8_t pinRegisterRx)
    {
        uint8_t ackSlot = bits - 8;
        volatile uint64_t cyclesStart = _getCycleCount();
        
        /* disable interrupts */
        uint32_t _state = xt_rsil(15);
        
        /* make sure the line is recessive for n consecutive bits */
        cyclesStart += (30 * cyclesBit);
        
        while (_getCycleCount() < cyclesStart)
        {
        #if 0
            if(!(GPIO_REG_READ(GPIO_IN_ADDRESS) & pinRegisterRx))
            {
                /* return busy */
                xt_wsr_ps(_state);
                return ERR_BUSY_LINE;
            }
        #endif
        }

        /* first bit always dominant */
        GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, pinRegisterTx);
        
        /* now write the bits */
        can_error_t ret = ERR_NO_ACK;
        uint32_t bitNum = 0;
        uint8_t dominantBits = 0;
        uint8_t recessiveBits = 0;
        
        while (1)
        {
            uint8_t data = *buffer++;

            for (uint8_t mask = 0x80; mask != 0; mask >>= 1)
            {
                /* silently skip the first bits in input buffer */
                if(skip)
                {
                    skip--;
                    continue;
                }
                
                bool txRecessive = ((data & mask) != 0);
                
                /* no stuffing in ACK slot */
                if(bitNum >= ackSlot - 1)
                {
                    recessiveBits = 0;
                    dominantBits = 0;
                }
                /* after 5 consecutive bits the same polarity, insert a stuff bit */
                else if(recessiveBits == 5)
                {
                    /* first check recessive bits */
                    dominantBits = 1;
                    recessiveBits = 0;
                    
                    /* wait till last bit ends */
                    while (_getCycleCount() < cyclesStart)
                    {
                    }
                
                    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, pinRegisterTx);
                    cyclesStart += cyclesBit;
                }
                else if(dominantBits == 5)
                {
                    /* then dominant bits */
                    dominantBits = 0;
                    recessiveBits = 1;
                    
                    /* wait till last bit ends */
                    while (_getCycleCount() < cyclesStart)
                    {
                    }
                    
                    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, pinRegisterTx);
                    cyclesStart += cyclesBit;
                }
                
                /* set the TX line */
                if(txRecessive)
                {
                    dominantBits = 0;
                    recessiveBits++;
                    
                    /* wait till last bit (or stuff bit) ends */
                    while (_getCycleCount() < cyclesStart)
                    {
                    }
                    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, pinRegisterTx);
                }
                else
                {
                    recessiveBits = 0;
                    dominantBits++;
                    
                    /* wait till last bit (or stuff bit) ends */
                    while (_getCycleCount() < cyclesStart)
                    {
                    }
                    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, pinRegisterTx);
                }
                
                /* wait until we shall sample the RX line */
                while (_getCycleCount() < (cyclesStart + cyclesSample))
                {
                }

                /* do so, read the RX line status */
                bool rxRecessive = ((GPIO_REG_READ(GPIO_IN_ADDRESS) & pinRegisterRx) != 0);
                
                /* and check if the RX line is the same as the TX line */
                if(/* !!!!!!!!!!!!!!!!!!TEST!!!!!!!!!!!!!!!!!! */ 0 && rxRecessive != txRecessive)
                {
                    if(rxRecessive)
                    {
                        /* abort, transceiver seems not to send. sent dominant, but received recessive */
                        GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, pinRegisterTx);
                        xt_wsr_ps(_state);
                        return ERR_TRCV_ERR;
                    }
                    else
                    {
                        if(bitNum == ackSlot)
                        {
                            ret = ERR_OK;
                        }
                        else
                        {
                            /* abort, there was a collision */
                            GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, pinRegisterTx);
                            xt_wsr_ps(_state);
                            return (can_error_t)bitNum;
                        }
                    }                    
                }
                
                /* increment bit position */
                bitNum++;
                if(bitNum >= bits)
                {
                    /* if all bits were transferred, return */
                    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, pinRegisterTx);
                    xt_wsr_ps(_state);
                    return ERR_OK;
                }
                
                /* update start timestamp so the next cycle will wait till the bit time is done */
                cyclesStart += cyclesBit;
            }
        }
    }
    
    static void ICACHE_RAM_ATTR parseBuffer(ESP8266Can *can, uint16_t *data, uint32_t length)
    {
        static uint32_t bits = 0;
        static uint32_t prevBit = 0;
        
        /* parse the received buffer, count all bits in it */
        for(uint32_t bytePos = 0; bytePos < length; bytePos++)
        {
            for(uint32_t bitPos = 0; bitPos < 16; bitPos++)
            {
                /* where to store the bit count */
                uint32_t *buf = &InterruptReceiveBuffers[ReceiveBuffersWriteNum][ReceiveBuffersWriteEntry];
                uint32_t thisBit = (data[bytePos ^ 1] & (1<<(15 - bitPos))) != 0;
                
                /* if the bit level changed, store how many bits were consecutive high or low */
                if(thisBit != prevBit)
                {
                    /* append (oversampled) bit count or terminate buffer */
                    if(ReceiveBuffersWriteEntry < INT_RX_BUFFER_SIZE - 2)
                    {
                        *buf = bits;
                        ReceiveBuffersWriteEntry++;
                    }
                    else
                    {
                        /* just terminate, buffer is already full */
                        can->RxQueueOverflows++;
                        *buf = 0xFFFFFFFF;
                    }
                    
                    /* there was a level change, so its the first bit of this level now */
                    bits = 1;
                    prevBit = thisBit;
                }
                else
                {
                    /* another bit of the same level */
                    bits++;
                    
                    /* message ends, too many bits with same level */
                    if((ReceiveBuffersWriteEntry > 0) && (bits > 8 * RxOversampling))
                    {
                        *buf = 0xFFFFFFFF;
                        ReceiveBuffersWriteNum = ((ReceiveBuffersWriteNum + 1) % INT_RX_BUFFERS);
                        ReceiveBuffersWriteEntry = 0;
                    }
                }
            }
        }
    }
    
    static void ICACHE_RAM_ATTR i2sInterrupt(ESP8266Can *can) 
    {
        uint32_t slc_intr_status = READ_PERI_REG(SLC_INT_STATUS);
        WRITE_PERI_REG(SLC_INT_CLR, 0xFFFFFFFF);
        
        /* wait, that should not happen? */
        if(!slc_intr_status)
        {
            Serial.printf("zero status\n"); 
            return;            
        }

        /* the Rx interrupt happened - this is TRANSMITTED data! */
        if((slc_intr_status & SLC_RX_EOF_INT_ST))
        {
            /* reset flag */
            slc_intr_status &= ~SLC_RX_EOF_INT_ST;
            InterruptRxCount++;     
            
            /* prepare item for requeue */
            struct slc_queue_item *completed = (struct slc_queue_item*)READ_PERI_REG(SLC_RX_EOF_DES_ADDR);
            completed->owner = 1;
            
        }
        
        /* the Tx interrupt handled, which is the data we RECEIVE. weird, ya. */
        if((slc_intr_status & SLC_TX_EOF_INT_ST))
        {
            /* reset flag */
            slc_intr_status &= ~SLC_TX_EOF_INT_ST;
            InterruptTxCount++;
            
            /* fetch filled buffer */
            struct slc_queue_item *completed = (struct slc_queue_item*)READ_PERI_REG(SLC_TX_EOF_DES_ADDR);
            
            /* go through all bits and count them, passing information to main loop context */
            parseBuffer(can, (uint16_t *)completed->buf_ptr, completed->datalen / 2);
            
            /* prepare item for requeue */
            completed->owner = 1;
        }
        
        if(slc_intr_status)
        {
            Serial.printf("unknown status: 0x%08X\n", slc_intr_status);        
        }
    }
}

ESP8266Can::ESP8266Can(uint32_t rate, uint8_t gpio_tx) : 
    _rate(rate), 
    _gpio_tx(gpio_tx), 
    _gpio_rx(D6),
    _maxTries(1024)
{
    digitalWrite(_gpio_tx, HIGH);
    pinMode(_gpio_rx, INPUT);
    pinMode(_gpio_tx, OUTPUT);
}

ESP8266Can::~ESP8266Can()
{
}

void ESP8266Can::StartRx()
{
    Serial.printf("ESP8266Can::StartRx\n");
    RxOversampling = (BASEFREQ / bestClkmDiv / bestBckDiv) / _rate;
    Serial.printf("   Rate:         %d\n", _rate);
    Serial.printf("   MDIV:         %d\n", bestClkmDiv);
    Serial.printf("   BCK:          %d\n", bestBckDiv);
    Serial.printf("   Oversampling: %d\n", RxOversampling);
    
    InitI2S();
    StartI2S();
    Serial.printf("Started\n");
}

void ESP8266Can::StartI2S()
{
	/* start transmission for RX and TX */
	SET_PERI_REG_MASK(SLC_RX_LINK, SLC_RXLINK_START);
	SET_PERI_REG_MASK(SLC_TX_LINK, SLC_TXLINK_START);
    
	SET_PERI_REG_MASK(I2SCONF, I2S_I2S_RX_START);
	SET_PERI_REG_MASK(I2SCONF, I2S_I2S_TX_START);
}

void ESP8266Can::StopI2S()
{
	/* stop transmission for RX and TX */
	CLEAR_PERI_REG_MASK(I2SCONF, I2S_I2S_RX_START);
	CLEAR_PERI_REG_MASK(I2SCONF, I2S_I2S_TX_START);
    
	CLEAR_PERI_REG_MASK(SLC_RX_LINK, SLC_RXLINK_START);
	CLEAR_PERI_REG_MASK(SLC_TX_LINK, SLC_TXLINK_START);
}

void ESP8266Can::PrepareQueue(const char *name, struct slc_queue_item *queue, uint32_t queueLength, void *buffer, uint32_t bufferLength, uint32_t eof)
{
    int blockSize = bufferLength / queueLength;
    
    /* prepare linked DMA descriptors, having EOF set for all */
    for(int num = 0; num < queueLength; num++)
    {
        int nextNum = (num + 1) % queueLength;
        
        queue[num].owner = 1;
        queue[num].eof = eof;
        queue[num].sub_sof = 0;
        queue[num].datalen = blockSize;
        queue[num].blocksize = blockSize;
        queue[num].buf_ptr = (uint32_t)buffer + num * blockSize;
        queue[num].unused = 0;
        queue[num].next_link_ptr = (uint32_t)&queue[nextNum];
        
        //Serial.printf("%s[%d] (0x%08X) with %d bytes, buf_ptr (0x%08X), next %d (0x%08X)\n", name, num, &queue[num], queue[num].datalen, queue[num].buf_ptr, nextNum, queue[num].next_link_ptr);
    }
}

void ESP8266Can::InitI2S(void) 
{
    /* ----------------- setup buffers ----------------- */
    
    /* fill with dummy data */
    memset((void *)I2SBufferRxData, 0xDE, sizeof(I2SBufferRxData));
    memset((void *)I2SBufferTxData, 0xAD, sizeof(I2SBufferTxData));

    /* prepare linked DMA descriptors, having EOF set for all RX slots */
    PrepareQueue("I2SQueueRx", I2SQueueRx, COUNT(I2SQueueRx), I2SBufferRxData, sizeof(I2SBufferRxData), 1);
    PrepareQueue("I2SQueueTx", I2SQueueTx, COUNT(I2SQueueTx), I2SBufferTxData, sizeof(I2SBufferTxData), 1);
    
    
    /* ----------------- setup IO ----------------- */

    /* configure IO pins */
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_I2SI_DATA);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_I2SI_WS);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_I2SI_BCK);
    
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_I2SO_DATA);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_I2SO_WS);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_I2SO_BCK);
    
    
    /* ----------------- setup I2S ----------------- */
    
    /* configure I2S subsystem */
    I2S_CLK_ENABLE();
    CLEAR_PERI_REG_MASK(I2SCONF, I2S_I2S_RESET_MASK);
    SET_PERI_REG_MASK(I2SCONF, I2S_I2S_RESET_MASK);
    CLEAR_PERI_REG_MASK(I2SCONF, I2S_I2S_RESET_MASK);

    /* configure the operating mode */
    CLEAR_PERI_REG_MASK(I2SCONF, I2S_RECE_SLAVE_MOD|I2S_TRANS_SLAVE_MOD);
    SET_PERI_REG_MASK(I2SCONF, I2S_RIGHT_FIRST|I2S_MSB_RIGHT|I2S_RECE_MSB_SHIFT|I2S_TRANS_MSB_SHIFT);
    
    /* set speed and bit count (16 bit: 0 extra bits) */
	CLEAR_SET_REG_POS(I2SCONF, I2S_BITS_MOD_S, I2S_BITS_MOD, 0);
	CLEAR_SET_REG_POS(I2SCONF, I2S_BCK_DIV_NUM_S, I2S_BCK_DIV_NUM, (bestBckDiv)&I2S_BCK_DIV_NUM);
	CLEAR_SET_REG_POS(I2SCONF, I2S_CLKM_DIV_NUM_S, I2S_CLKM_DIV_NUM, (bestClkmDiv)&I2S_CLKM_DIV_NUM);
    
    
    /* Select 16bits per channel (FIFO_MOD=0), no DMA access (FIFO only) */
	CLEAR_SET_REG_POS(I2S_FIFO_CONF, I2S_I2S_RX_FIFO_MOD_S, I2S_I2S_RX_FIFO_MOD, 0);
	CLEAR_SET_REG_POS(I2S_FIFO_CONF, I2S_I2S_TX_FIFO_MOD_S, I2S_I2S_TX_FIFO_MOD, 0);
	CLEAR_SET_REG_POS(I2S_FIFO_CONF, I2S_I2S_RX_DATA_NUM_S, I2S_I2S_RX_DATA_NUM, 0);
	CLEAR_SET_REG_POS(I2S_FIFO_CONF, I2S_I2S_TX_DATA_NUM_S, I2S_I2S_TX_DATA_NUM, 0);
    
    /* Enable SLC DMA in I2S subsystem */
	CLEAR_SET_REG_POS(I2S_FIFO_CONF, 0, I2S_I2S_DSCR_EN, I2S_I2S_DSCR_EN);
    
    /* set dual channel data (CHAN_MOD=0) */
	CLEAR_SET_REG_POS(I2SCONF_CHAN, I2S_RX_CHAN_MOD_S, I2S_RX_CHAN_MOD, 0);
	CLEAR_SET_REG_POS(I2SCONF_CHAN, I2S_TX_CHAN_MOD_S, I2S_TX_CHAN_MOD, 0);
    
    /* set maximum I2S FIFO size to be transferred at once (in units of 4 bytes) */
	CLEAR_SET_REG_POS(I2SRXEOF_NUM, I2S_I2S_RX_EOF_NUM_S, I2S_I2S_RX_EOF_NUM, 128);
    
    
    /* ----------------- setup SLC ----------------- */

    /* reset DMA */
	SET_PERI_REG_MASK(SLC_CONF0, SLC_RXLINK_RST|SLC_TXLINK_RST|SLC_AHBM_RST|SLC_AHBM_FIFO_RST);
    CLEAR_PERI_REG_MASK(SLC_CONF0, SLC_RXLINK_RST|SLC_TXLINK_RST|SLC_AHBM_RST|SLC_AHBM_FIFO_RST);
    
	SET_PERI_REG_MASK(SLC_CONF0, SLC_TX_LOOP_TEST|SLC_RX_LOOP_TEST|SLC_RX_AUTO_WRBACK|SLC_RX_NO_RESTART_CLR|SLC_DATA_BURST_EN|SLC_DSCR_BURST_EN);
  
	/* Enable and configure DMA */
	CLEAR_SET_REG_POS(SLC_CONF0, SLC_MODE_S, SLC_MODE, 1);
    
	SET_PERI_REG_MASK(SLC_RX_DSCR_CONF, SLC_INFOR_NO_REPLACE|SLC_TOKEN_NO_REPLACE);

    /* configure the first descriptors */
	CLEAR_SET_REG_POS(SLC_RX_LINK, 0, SLC_RXLINK_DESCADDR_MASK, I2SQueueRx);
	CLEAR_SET_REG_POS(SLC_TX_LINK, 0, SLC_TXLINK_DESCADDR_MASK, I2SQueueTx);

    /* set up SLC interrupt */
    ETS_SLC_INTR_DISABLE();
    SET_PERI_REG_MASK(SLC_INT_CLR, 0xffffffff);
    WRITE_PERI_REG(SLC_INT_ENA, SLC_INTEREST_EVENT);
    ETS_SLC_INTR_ATTACH((int_handler_t)i2sInterrupt, (void *)this);
    ETS_SLC_INTR_ENABLE();
}


void ESP8266Can::Loop()
{
    /* dump CAN frames - to console for now */
    while(ReceiveBuffersReadNum != ReceiveBuffersWriteNum)
    {
        uint8_t stuffEnd = 0xFF;
        uint8_t stuffBitCount = 0;
        uint8_t bitPos = 5;
        bool undoStuff = false;
        bool bitLevel = false;
        uint8_t frame_buffer[32];
        
        /* CAN message data */
        uint16_t id = 0;
        uint8_t length = 0xFF;
        uint8_t payload[8];
        bool req = false;
        bool ack = false;
        
        memset(frame_buffer, 0x00, sizeof(frame_buffer));
        
        if(_debug)
        {
            Serial.printf("[Buffer %02d] ", ReceiveBuffersReadNum);
        }
        
        for(int pos = 0; pos < INT_RX_BUFFER_SIZE; pos++)
        {
            uint32_t bits = InterruptReceiveBuffers[ReceiveBuffersReadNum][pos];
            
            /* when enough bits received, we can extract the DLC */
            if((bitPos >= (19 + 5)) && (length == 0xFF))
            {
                DecodeCanFrame(frame_buffer, &id, &length, payload, &req, &ack, false);
                stuffEnd = 5 + 19 + (length * 8) + 16;
            }
            
            /* reached end of frame bitstream, decode */
            if(bits == 0xFFFFFFFF)
            {
                uint32_t ret = DecodeCanFrame(frame_buffer, &id, &length, payload, &req, &ack, true);
                bool dump = false;

                if(ret)
                {
                    dump = true;
                }
                else
                {
                    RxSuccess++;
                    
                    if(_debug)
                    {
                        dump = true;
                    }
                }
                
                if(dump)
                {
                    Serial.printf("ID: 0x%03X, len: %d, %s %s, data: ", id, length, req ? "  REQ" : "noREQ", ack ? "  ACK" : "noACK");
                    
                    for(int pos = 0; pos < length; pos++)
                    {
                        Serial.printf("%02X ", payload[pos]);
                    }
                    Serial.printf("\n");
                }
                
                break;
            }
            
            /* first entry is crap */
            if(pos < 1)
            {
                continue;
            }
            
            /* calculate the number of bits */
            bits = (bits + (RxOversampling / 2)) / RxOversampling;
            
        
            for(uint32_t bitNum = 0; bitNum < bits; bitNum++)
            {
                if(!undoStuff)
                {
                    if(bitLevel)
                    {
                        frame_buffer[bitPos / 8] |= (1 << (7 - (bitPos % 8)));
                    }
                    bitPos++;
                }
                
                undoStuff = false;
                
                /* only handle stuff bits until reaching ACK */
                if(bitPos < stuffEnd)
                {
                    /* 5th bit in row, insert a stuff bit */
                    if(bitNum == 4)
                    {
                        undoStuff = true;
                    }
                    else if(bitNum >= 5)
                    {
                        /* must not happen */
                        RxErrStuffBits++;
                        Serial.printf(" (stuff err) ");
                    }
                }
                
            }
            
            bitLevel = !bitLevel;
        }
        
        /* next buffer */
        ReceiveBuffersReadNum = (ReceiveBuffersReadNum + 1) % INT_RX_BUFFERS;
    }
}

/* one CRC calculation step */
void ESP8266Can::CalcCRCBit(uint16_t *crc, uint16_t bit)
{
    uint16_t crcBit;
    
    crcBit = bit ^ ((*crc >> 14) & 1);
    *crc <<= 1;
    
    if(crcBit)
    {
        *crc = (*crc ^ 0x4599) & 0x7fff;
    }
}

/* calculate CRC for a given data buffer */
uint16_t ESP8266Can::CalcCRC(uint8_t *data, uint16_t startBit, uint16_t bitCount)
{
    uint16_t crc = 0;
    
    for(uint16_t bitPos = startBit; bitPos < startBit + bitCount; bitPos++)
    {
        uint16_t byteNum = bitPos / 8;
        uint16_t bitNum = bitPos % 8;
        uint8_t bitMask = (1 << (7 - bitNum));
        
        uint8_t bit = ((data[byteNum] & bitMask) != 0);
        
        CalcCRCBit(&crc, bit);
    }
    
    return crc & 0x7FFF;
}

/* assemble the transmission buffer and return the bit count. the first LEADING_BITS bits are not meant to be sent */
uint32_t ESP8266Can::BuildCanFrame(uint8_t *buffer, uint16_t id, uint8_t length, uint8_t *data, bool req_remote, bool self_ack)
{
    /* ensure fields do not exceed bit ranges */
    id &= 0x07FF;
    length &= 0x0F;
    
    /* place, but not send LEADING_BITS in front to make alignment easier */
    buffer[0] = 0xf8 | (0 << 2) |(id >> 9);
    buffer[1] = (id >> 1);
    buffer[2] = (id << 7) | ((req_remote ? 1 : 0) << 6) | length;
    
    memcpy(&buffer[3], data, length);
    
    uint16_t crc = CalcCRC(buffer, LEADING_BITS, 19 + length * 8);
    
    buffer[3 + length] = (crc >> 7);
    buffer[4 + length] = (crc << 1) | (1 << 0);
    buffer[5 + length] = ((self_ack ? 0 : 1) << 7) | 0x7F;
    buffer[6 + length] = 0xFF;
    buffer[7 + length] = 0xFF;
    
    /* return the number of bits to be sent (without trailing recessives) */
    return (1 + 11 + 1 + 1 + 1 + 4 + (8 * length) + 15 + 1 + 1 + 1 + 7);
}

/*  */
uint32_t ESP8266Can::DecodeCanFrame(uint8_t *buffer, uint16_t *id, uint8_t *length, uint8_t *data, bool *req_remote, bool *ack, bool errors)
{
    *id = ((uint16_t)buffer[0] << 9) | ((uint16_t)buffer[1] << 1) | (buffer[2] >> 7);
    *req_remote = (buffer[2] & 0x40) != 0;
    *length = (buffer[2] & 0x0F);
    
    bool id_ext = (buffer[2] & 0x20) != 0;
    bool reserved = (buffer[2] & 0x10) != 0;
    
    memcpy(data, &buffer[3], *length);
    
    uint16_t crc = ((uint16_t)buffer[3 + *length] << 7) | (buffer[4 + *length] >> 1);
    uint16_t crc_calced = CalcCRC(buffer, LEADING_BITS, 19 + *length * 8);
    
    bool crc_delim = (buffer[4 + *length] & 0x01) != 0;
    *ack = !((buffer[5 + *length] & 0x80) != 0);
    
    if(!errors)
    {
        return 0;
    }
    
    /* compare CRC with calculated one */
    if(crc != crc_calced)
    {
        Serial.printf("CRC: 0x%04X/0x%04X ", crc, crc_calced);
        
        RxErrCrc++;
        return 1;
    }
    
    /* CRC delimiter is always dominant */
    if(!crc_delim)
    {
        Serial.printf("CRC delim err");
        RxErrFormat++;
        return 1;
    }
    
    /* ACK slot must be dominant */
    if(!*ack)
    {
        Serial.printf("ACK err");
        RxErrAck++;
        return 1;
    }
    
    return 0;
}

/* try to get a free slot and transmit the CAN frame */
can_error_t ESP8266Can::SendMessage(uint16_t id, uint8_t length, uint8_t *data, bool req_remote, bool self_ack)
{
    /* a buffer of 32 bytes for the CAN frame is more than enough */
    uint8_t buffer[32];
    uint32_t bits = BuildCanFrame(buffer, id, length, data, req_remote, self_ack);

    /* will only get returned in case of _maxTries == 0 */
    can_error_t ret = ERR_BUSY_LINE;
    
    /* this is not very elegant and probably violates the standard, but it's good enough */
    for(int tries = 0; tries < _maxTries; tries++)
    {
        Serial.printf("[CAN] Send ID:0x%03X, %d byte: ", id, length);
        
        Serial.printf("\n");
        can_error_t bitPos = sendRawData(buffer, cyclesBit(), cyclesSample(), LEADING_BITS, bits, pinRegisterTx(), pinRegisterRx());
        Serial.printf("\n");
        
        /* check why transmission failed */
        if(bitPos == ERR_BUSY_LINE)
        {
            /* the line wasn't recessive for some time */
            Serial.printf("Line is busy\n");
            TxErrLineBusy++;
            ret = ERR_BUSY_LINE;
        }
        else if(bitPos == ERR_TRCV_ERR)
        {
            /* the line wasn't recessive for some time */
            Serial.printf("Transceiver error\n");
            TxErrTransceiver++;
            ret = ERR_TRCV_ERR;
            break;
        }
        else if(bitPos == ERR_NO_ACK)
        {
            /* the line wasn't recessive for some time */
            Serial.printf("No ACK error\n");
            TxErrNoAck++;
            ret = ERR_NO_ACK;
            break;
        }
        else if(bitPos < 12)
        {
            /* it was within arbitration field */
            Serial.printf("Arbitration error (bit %d / %d)\n", bitPos, bits);
            TxErrArbitration++;
            ret = ERR_ARBRITATION;
        }
        else if(bitPos >= bits)
        {
            /* all bits transferred */
            Serial.printf("Transferred\n");
            TxSuccess++;
            ret = ERR_OK;
            break;
        }
        else
        {
            Serial.printf("Collision (bit %d / %d)\n", bitPos, bits);
            TxErrCollision++;
            ret = ERR_COLLISION;
        }
        
        /* wait a msec and try again */
        delay(10);
        
        ESP.wdtFeed();
    }
    
    return ret;
}

#endif
