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
        
    
extern "C"
{    
    #include "i2s_reg.h"
    #include "i2s.h"
    #include "eagle_soc.h"
    #include "esp8266_peri.h"
    #include "slc_register.h"
        
    /* intentionally keeping them as global variable because having them in class makes the code slower. 
       Defining as class members has no benefit as we can only be instanciated once anyway. */
    static uint8_t InterruptReceiveBuffers[CAN_RX_BUFFERS][CAN_RX_BUFFER_SIZE];
    static uint32_t ReceiveBuffersReadNum = 0;
    static uint32_t ReceiveBuffersWriteNum = 0;
    static uint32_t ReceiveBitNum = 0;
    static uint32_t InterruptRxCount = 0;
    static uint32_t InterruptTxCount = 0;
    static uint32_t RxOversampling = 0;
    static uint8_t BitSamplingTable[128];
    
    static uint32_t RxActiveBits = 0;
    static uint32_t RxTotalBits = 0;
    
    static uint32_t config_cyclesBit = 0;
    static uint32_t config_cyclesSample = 0;
    static uint32_t config_skip = 0;
    static uint32_t config_pinRegisterTx = 0;
    static uint32_t config_pinRegisterRx = 0;
    static const uint32_t config_lineFreeBitCount = 10;

    extern void rom_i2c_writeReg_Mask(uint32_t block, uint32_t host_id, uint32_t reg_add, uint32_t Msb, uint32_t Lsb, uint32_t indata);

    #define xt_ccount() (__extension__({uint32_t ccount; __asm__ __volatile__("rsr %0,ccount":"=a" (ccount)); ccount;}))
    
    static inline uint32_t getCycleCount()
    {
        return xt_ccount();
    }    
    
    static inline uint32_t intDisable()
    {
        return xt_rsil(15);
        
    }
    static inline void intEnable(uint32_t state)
    {
        xt_wsr_ps(state);
    }

    can_error_t ICACHE_RAM_ATTR sendRawData(uint8_t* buffer, uint32_t bits)
    {
        /* disable interrupts */
        uint32_t old_ints = intDisable();
        
        /* wait till timer overflows */
        while(getCycleCount() > (0xFFFFFFFF - ((121 + config_lineFreeBitCount + 10 /* backup */) * config_cyclesBit)));
        
        uint32_t ackSlot = bits - 8;
        uint32_t cyclesStart = getCycleCount();
        uint32_t skip = config_skip;
        
        /* make sure the line is recessive for n consecutive bits */
        cyclesStart += (config_lineFreeBitCount * config_cyclesBit);
        
        while (getCycleCount() < cyclesStart)
        {
            if(!(GPIO_REG_READ(GPIO_IN_ADDRESS) & config_pinRegisterRx))
            {
                /* return busy */
                intEnable(old_ints);
                return ERR_BUSY_LINE;
            }
        }

        /* first bit always dominant */
        GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, config_pinRegisterTx);
        
        /* now write the bits */
        can_error_t ret = ERR_NO_ACK;
        uint32_t bitNum = 0;
        uint32_t dominantBits = 0;
        uint32_t recessiveBits = 0;
        
        while (1)
        {
            uint8_t data = *buffer++;

            for (uint32_t mask = 0x80; mask != 0; mask >>= 1)
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
                    while (getCycleCount() < cyclesStart)
                    {
                    }
                
                    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, config_pinRegisterTx);
                    cyclesStart += config_cyclesBit;
                }
                else if(dominantBits == 5)
                {
                    /* then dominant bits */
                    dominantBits = 0;
                    recessiveBits = 1;
                    
                    /* wait till last bit ends */
                    while (getCycleCount() < cyclesStart)
                    {
                    }
                    
                    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, config_pinRegisterTx);
                    cyclesStart += config_cyclesBit;
                }
                
                /* set the TX line */
                if(txRecessive)
                {
                    dominantBits = 0;
                    recessiveBits++;
                    
                    /* wait till last bit (or stuff bit) ends */
                    while (getCycleCount() < cyclesStart)
                    {
                    }
                    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, config_pinRegisterTx);
                }
                else
                {
                    recessiveBits = 0;
                    dominantBits++;
                    
                    /* wait till last bit (or stuff bit) ends */
                    while (getCycleCount() < cyclesStart)
                    {
                    }
                    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, config_pinRegisterTx);
                }
                
                /* wait until we shall sample the RX line */
                while (getCycleCount() < (cyclesStart + config_cyclesSample))
                {
                }

                /* do so, read the RX line status */
                bool rxRecessive = ((GPIO_REG_READ(GPIO_IN_ADDRESS) & config_pinRegisterRx) != 0);
                
                /* and check if the RX line is the same as the TX line */
                if(rxRecessive != txRecessive)
                {
                    if(rxRecessive)
                    {
                        /* abort, transceiver seems not to send. sent dominant, but received recessive */
                        GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, config_pinRegisterTx);
                        intEnable(old_ints);
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
                            GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, config_pinRegisterTx);
                            intEnable(old_ints);
                            return (can_error_t)bitNum;
                        }
                    }                    
                }
                
                /* increment bit position */
                bitNum++;
                if(bitNum >= bits)
                {
                    /* if all bits were transferred, return */
                    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, config_pinRegisterTx);
                    intEnable(old_ints);
                    return ERR_OK;
                }
                
                /* update start timestamp so the next cycle will wait till the bit time is done */
                cyclesStart += config_cyclesBit;
            }
        }
    }
    
    static void ICACHE_RAM_ATTR parseBuffer(ESP8266Can *can, uint16_t *data, uint32_t data_length)
    {
        static uint32_t consecutiveBits = 0;
        static uint32_t prevBit = 0;
        static uint32_t stuffEnd = 0xFFFFFFFF;
        static uint32_t undoStuff = 0;
        
        RxTotalBits += (data_length * 16) / RxOversampling;
        
        /* where to store the CAN frame data */
        uint8_t *buf = InterruptReceiveBuffers[ReceiveBuffersWriteNum];
        
        /* parse the received buffer, count all bits in it */
        for(uint32_t wordPos = 0; wordPos < data_length; wordPos++)
        {
            /* swap words */
            uint32_t thisWord = data[wordPos ^ 1];
            
            /* every element is 16 bits */
            for(uint32_t bitPos = 0; bitPos < 16; bitPos++)
            {
                uint32_t thisBit = thisWord & 0x8000;
                
                /* if the bit level changed, check how many bits were consecutive high or low */
                if(thisBit != prevBit)
                {
                    /* append (oversampled) bit count or terminate buffer */
                    if((ReceiveBitNum == 0) && prevBit)
                    {
                        /* received n bits of *recessive* data before our frame starts.
                           this is the "inactive" bus. just throw these bits away. */
                    }
                    else
                    {
                        /* now calculate how many bits were received */
                        uint32_t bits = BitSamplingTable[consecutiveBits];
                        
                        /* process that number of bits */
                        for(uint32_t bit = 0; bit < bits; bit++)
                        {
                            /* if this will be a stuff bit, ignore it */
                            if(!undoStuff)
                            {
                                /* in the output buffer, there are some dummy bits for padding data properly */
                                uint32_t bufBit = LEADING_BITS + ReceiveBitNum;
                                uint32_t bufByte = (bufBit >> 3);
                                
                                if(bufByte < CAN_RX_BUFFER_SIZE)
                                {
                                    /* now write the bit value of the "last" bit level */
                                    if(prevBit)
                                    {
                                        buf[bufByte] |= (0x80 >> (bufBit & 0x07));
                                    }
                                    ReceiveBitNum++;
                
                                    /* when enough bits received, we can extract the DLC. just do once until stuffEnd is known */
                                    if((stuffEnd == 0xFFFFFFFF) && (ReceiveBitNum >= 19))
                                    {
                                        /* we only need the length to know when there is no stuffing anymore */
                                        uint32_t length = can->DecodeCanFrame(buf, NULL, NULL, NULL, NULL, NULL, true);
                                        stuffEnd = 19 + (length * 8) + 16;
                                    }
                                }
                            }
                            undoStuff = 0;
                            
                            /* only handle stuff bits until reaching ACK */
                            if(ReceiveBitNum < stuffEnd)
                            {
                                /* 5th bit in row of the same level, insert a stuff bit */
                                if(bit == 4)
                                {
                                    undoStuff = 1;
                                }
                                else if(bit >= 5)
                                {
                                    /* 5 bits in a row? must not happen */
                                    can->RxErrStuffBits++;
                                }
                            }
                        }
                    }
                    
                    /* there was a level change, so its the first bit of this level now */
                    consecutiveBits = 1;
                    prevBit = thisBit;
                }
                else if(consecutiveBits < 9 * RxOversampling)
                {
                    /* another bit of the same level until 8 bits of the same level were seen */
                    consecutiveBits++;
                }
                else if(ReceiveBitNum > 0 && thisBit)
                {
                    /* finish frame when line is going recessive for more than 8 bits */
                    RxActiveBits += ReceiveBitNum;
                    ReceiveBitNum = 0;
                    ReceiveBuffersWriteNum = ((ReceiveBuffersWriteNum + 1) % CAN_RX_BUFFERS);
                    buf = InterruptReceiveBuffers[ReceiveBuffersWriteNum];
                    memset(buf, 0x00, CAN_RX_BUFFER_SIZE);
                    
                    /* reset per-frame information */
                    stuffEnd = 0xFFFFFFFF;
                    
                    /* if we now - after incrementing - hit again the read buffer, we have an overrun */
                    if(ReceiveBuffersWriteNum == ReceiveBuffersReadNum)
                    {
                        can->RxQueueOverflows++;
                    }
                }
                
                /* evaluate the next bit */
                thisWord <<= 1;
            }
        }
    }
    
    void processQueueItem(ESP8266Can *can) 
    {
        /* go through all bits and count them, passing information to main loop context */
        parseBuffer(can, (uint16_t *)can->CurrentQueueItem->buf_ptr, can->CurrentQueueItem->datalen / 2);
        
        can->CurrentQueueItem->owner = 1;
        can->CurrentQueueItem = (struct slc_queue_item *)can->CurrentQueueItem->next_link_ptr;
    }
    
    static void ICACHE_RAM_ATTR i2sInterrupt(ESP8266Can *can) 
    {
        uint32_t slc_intr_status = READ_PERI_REG(SLC_INT_STATUS);
        WRITE_PERI_REG(SLC_INT_CLR, 0xFFFFFFFF);
        
        static uint32_t last_exec_time = 0;
        uint32_t this_exec_time = getCycleCount();
        
        /* make sure we didn't run only half a msec ago */
        if((this_exec_time - last_exec_time) < (160000000 / 2000))
        {
            can->RestartI2S();
            can->IntErrorCount++;
            return;
        }

#ifdef ISR_TIMING        
        GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, _BV(can->LedPin));
#endif
        
        /* wait, that should not happen? */
        if(!slc_intr_status)
        {
            can->RestartI2S();
            can->IntErrorCount++;
            return;
        }
        
        can->_rxRunning = true;

        /* the Tx interrupt handler, which is the data we RECEIVE. weird, ya. */
        if((slc_intr_status & SLC_TX_EOF_INT_ST))
        {
            /* reset flag */
            slc_intr_status &= ~SLC_TX_EOF_INT_ST;
            InterruptTxCount++;
            
            /* which one is the "last" finished one? */
            struct slc_queue_item *completed = (struct slc_queue_item*)READ_PERI_REG(SLC_TX_EOF_DES_ADDR);
            WRITE_PERI_REG(SLC_TX_EOF_DES_ADDR, 0x00000000);
            
            if(completed)
            {
                /* go through all previous items */
                while(can->CurrentQueueItem != completed)
                {
                    processQueueItem(can);
                }
                
                /* and the finished ones */
                processQueueItem(can);
            }
        }
        
        if(slc_intr_status)
        {
            can->RestartI2S();
            can->IntErrorCount++;
            return;
        }
        last_exec_time = getCycleCount();
        
#ifdef ISR_TIMING        
        GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, _BV(can->LedPin));
#endif
    }
}

ESP8266Can::ESP8266Can(uint32_t rate, uint8_t gpio_tx, uint8_t gpio_rx) : 
    _rate(rate), 
    _gpio_tx(gpio_tx), 
    _gpio_rx(gpio_rx),
    _maxTries(100)
{
    digitalWrite(_gpio_tx, HIGH);
    pinMode(_gpio_rx, INPUT);
    pinMode(_gpio_tx, OUTPUT);
    
    config_skip = LEADING_BITS;
    config_cyclesBit = cyclesBit();
    config_cyclesSample = cyclesSample();
    config_pinRegisterTx = pinRegisterTx();
    config_pinRegisterRx = pinRegisterRx();
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
    Serial.printf("   Samplingrate: %d\n", (BASEFREQ / bestClkmDiv / bestBckDiv));
    Serial.printf("   Oversampling: %d\n", RxOversampling);
    
    for(uint32_t pos = 0; pos < COUNT(BitSamplingTable); pos++)
    {
        uint8_t bits = (pos + RxOversampling / 2) / RxOversampling;
        BitSamplingTable[pos] = bits;
    }
    
    memset(InterruptReceiveBuffers, 0x00, sizeof(InterruptReceiveBuffers));
    InitI2S();
    RestartI2S();
    _rxStarted = true;
    Serial.printf("Started\n");
}

void ESP8266Can::StopRx()
{
    StopI2S();
    _rxStarted = false;
    _rxRunning = false;
    Serial.printf("Stopped\n");
}

void ESP8266Can::StartI2S()
{
	/* start transmission for I2S_RX and SLC_TX */
	SET_PERI_REG_MASK(SLC_TX_LINK, SLC_TXLINK_START);
	SET_PERI_REG_MASK(I2SCONF, I2S_I2S_RX_START);
}

void ESP8266Can::StopI2S()
{
	/* stop transmission for both I2S_RX and SLC_TX */
	CLEAR_PERI_REG_MASK(I2SCONF, I2S_I2S_RX_START);
	CLEAR_PERI_REG_MASK(SLC_TX_LINK, SLC_TXLINK_START);
    
    _rxRunning = false;
}

void ESP8266Can::RestartI2S()
{
    StopI2S();

    /* reset DMA */
    SET_PERI_REG_MASK(SLC_CONF0, SLC_RXLINK_RST|SLC_TXLINK_RST|SLC_AHBM_RST|SLC_AHBM_FIFO_RST);
    CLEAR_PERI_REG_MASK(SLC_CONF0, SLC_RXLINK_RST|SLC_TXLINK_RST|SLC_AHBM_RST|SLC_AHBM_FIFO_RST);
    SET_PERI_REG_MASK(SLC_CONF0, SLC_TX_LOOP_TEST|SLC_RX_LOOP_TEST|SLC_RX_AUTO_WRBACK|SLC_RX_NO_RESTART_CLR|SLC_DATA_BURST_EN|SLC_DSCR_BURST_EN);

    /* Enable and configure DMA */
    CLEAR_SET_REG_POS(SLC_CONF0, SLC_MODE_S, SLC_MODE, 1);

    /* configure the first descriptors */
    CLEAR_SET_REG_POS(SLC_TX_LINK, 0, SLC_TXLINK_DESCADDR_MASK, I2SQueueTx);
    CurrentQueueItem = I2SQueueTx;
    
    StartI2S();
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
        
        if(_debug)
        {
            Serial.printf("%s[%d] (0x%08X) with %d bytes, buf_ptr (0x%08X), next %d (0x%08X)\n", name, num, &queue[num], queue[num].datalen, queue[num].buf_ptr, nextNum, queue[num].next_link_ptr);
        }
    }
}

void ESP8266Can::InitI2S(void) 
{
    /* ----------------- setup buffers ----------------- */

    /* prepare linked DMA descriptors, having EOF set for all RX slots */
    PrepareQueue("I2SQueueTx", I2SQueueTx, COUNT(I2SQueueTx), I2SBufferTxData, sizeof(I2SBufferTxData), 1);
    CurrentQueueItem = I2SQueueTx;
    
    /* ----------------- setup IO ----------------- */

    /* configure IO pins */
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_I2SI_DATA);
    
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
	CLEAR_SET_REG_POS(I2S_FIFO_CONF, I2S_I2S_RX_DATA_NUM_S, I2S_I2S_RX_DATA_NUM, 0);
    
    /* Enable SLC DMA in I2S subsystem */
	CLEAR_SET_REG_POS(I2S_FIFO_CONF, 0, I2S_I2S_DSCR_EN, I2S_I2S_DSCR_EN);
    
    /* set dual channel data (CHAN_MOD=0) but doesn't seem to have any effect */
	CLEAR_SET_REG_POS(I2SCONF_CHAN, I2S_RX_CHAN_MOD_S, I2S_RX_CHAN_MOD, 0);
    
    /* ----------------- setup SLC ----------------- */

    /* reset DMA */
	SET_PERI_REG_MASK(SLC_CONF0, SLC_RXLINK_RST|SLC_TXLINK_RST|SLC_AHBM_RST|SLC_AHBM_FIFO_RST);
    CLEAR_PERI_REG_MASK(SLC_CONF0, SLC_RXLINK_RST|SLC_TXLINK_RST|SLC_AHBM_RST|SLC_AHBM_FIFO_RST);
	SET_PERI_REG_MASK(SLC_CONF0, SLC_TX_LOOP_TEST|SLC_RX_LOOP_TEST|SLC_RX_AUTO_WRBACK|SLC_RX_NO_RESTART_CLR|SLC_DATA_BURST_EN|SLC_DSCR_BURST_EN);
  
	/* Enable and configure DMA */
	CLEAR_SET_REG_POS(SLC_CONF0, SLC_MODE_S, SLC_MODE, 1);

    /* configure the first descriptors */
	CLEAR_SET_REG_POS(SLC_TX_LINK, 0, SLC_TXLINK_DESCADDR_MASK, I2SQueueTx);

    /* set up SLC interrupt */
    ETS_SLC_INTR_DISABLE();
    SET_PERI_REG_MASK(SLC_INT_CLR, 0xffffffff);
    WRITE_PERI_REG(SLC_INT_ENA, SLC_INTEREST_EVENT);
    ETS_SLC_INTR_ATTACH((int_handler_t)i2sInterrupt, (void *)this);
    ETS_SLC_INTR_ENABLE();
}

void ESP8266Can::Loop(void (*cbr)(uint16_t id, bool req, uint8_t length, uint8_t *payload, bool ack))
{
    static uint32_t lastTime = 0;
    uint32_t startTime = millis();
    uint32_t loops = 0;
    
    if(millis() - lastTime >= 1000)
    {
        uint32_t old_ints = intDisable();
        if(RxTotalBits)
        {
            BusLoadInternal = (RxActiveBits * 100 / (RxTotalBits + 1));
            RxActiveBits = 0;
            RxTotalBits = 0;
        }
        intEnable(old_ints);
        
        lastTime += 1000;
        Serial.printf("[ESP8266Can] [%08d] Rx: %d, Tx: %d, Load: %d%%  |  RxQueueErr: %d, RxErr: %d, TxErr: %d  |  IRQs: %d, Timer: 0x%08X, Errors: %d\n", lastTime, RxSuccess, TxSuccess, BusLoadInternal, RxQueueOverflows, RxErrors(), TxErrors(), InterruptTxCount + InterruptRxCount, getCycleCount(), IntErrorCount);
    }
    
    /* dump CAN frames - to console for now */
    while(ReceiveBuffersReadNum != ReceiveBuffersWriteNum)
    {
        if(millis() - startTime > 100)
        {
            Serial.printf("[ESP8266Can] Aborting after %d loops\n", loops);
            break;
        }
        loops++;
        
        uint8_t *frame_buffer = InterruptReceiveBuffers[ReceiveBuffersReadNum];
        
        /* try to decode frame */
        uint16_t id = 0;
        uint8_t length = 0xFF;
        uint8_t payload[8];
        bool req = false;
        bool ack = false;
        
        /* in case of error, dump frames */
        bool dump = false;

        if(DecodeCanFrame(frame_buffer, &id, &length, payload, &req, &ack, false))
        {
            /* failed. show raw data */
            dump = true;
        }
        else
        {
            /* success, call back with decoded data */
            if(cbr)
            {
                cbr(id, req, length, payload, ack);
            }
        }

        if(dump)
        {
            Serial.printf("raw: ");

            for(int pos = 0; pos < 16; pos++)
            {
                Serial.printf("%02X ", frame_buffer[pos]);
            }
            Serial.printf("\n");
        }
        
        /* next buffer */
        ReceiveBuffersReadNum = (ReceiveBuffersReadNum + 1) % CAN_RX_BUFFERS;
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
uint32_t ESP8266Can::DecodeCanFrame(uint8_t *buffer, uint16_t *id, uint8_t *length, uint8_t *data, bool *req_remote, bool *ack, bool lengthOnly)
{
    if(id)
    {
        *id = ((uint16_t)buffer[0] << 9) | ((uint16_t)buffer[1] << 1) | (buffer[2] >> 7);
    }
    uint8_t length_field = (buffer[2] & 0x0F);
    uint8_t length_value = (length_field <= 8) ? length_field : 0;
    
    if(length)
    {
        *length = length_value;
    }
    
    if(lengthOnly)
    {
        return length_value;
    }
    
    if(req_remote)
    {
        *req_remote = (buffer[2] & 0x40) != 0;
    }
    
    bool id_ext = (buffer[2] & 0x20) != 0;
    bool reserved = (buffer[2] & 0x10) != 0;
    
    if(data)
    {
        memcpy(data, &buffer[3], *length);
    }
    
    uint16_t crc = ((uint16_t)buffer[3 + *length] << 7) | (buffer[4 + *length] >> 1);
    uint16_t crc_calced = CalcCRC(buffer, LEADING_BITS, 19 + *length * 8);
    
    bool crc_delim = (buffer[4 + *length] & 0x01) != 0;
    if(ack)
    {
        *ack = !((buffer[5 + *length] & 0x80) != 0);
    }
    
    /* DLC can go from 0 to 8 */
    if(length_field > 8)
    {
        Serial.printf("length err (%d>8) ", length_field);
        RxErrFormat++;
        return 1;
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
    
    RxSuccess++;
    
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
    for(uint32_t tries = 0; tries < _maxTries; tries++)
    {
        if(_debug)
        {
            Serial.printf("[CAN] Send ID:0x%03X, %d byte: ", id, length);            
        }
        
        /* first wait for the interrupt doing it's work. might still have some jitter but should do its job most of the time. */
        if(_rxRunning)
        {
            struct slc_queue_item *lastCurrentQueueItem = CurrentQueueItem;
    
            while(lastCurrentQueueItem == CurrentQueueItem)
            {
                ESP.wdtFeed();
            }
        }
        
        /* now, probably right after the interrupt, send the data to be sent */
        can_error_t bitPos = sendRawData(buffer, bits);
        
        /* check why transmission failed */
        if(bitPos == ERR_BUSY_LINE)
        {
            /* the line wasn't recessive for some time */
            if(_debug)
            {
                Serial.printf("Line is busy\n");
            }
            TxErrLineBusy++;
            ret = ERR_BUSY_LINE;
        }
        else if(bitPos == ERR_TRCV_ERR)
        {
            /* the line wasn't recessive for some time */
            if(_debug)
            {
                Serial.printf("Transceiver error\n");
            }
            TxErrTransceiver++;
            ret = ERR_TRCV_ERR;
            break;
        }
        else if(bitPos == ERR_NO_ACK)
        {
            /* the line wasn't recessive for some time */
            if(_debug)
            {
                Serial.printf("No ACK error\n");
            }
            TxErrNoAck++;
            ret = ERR_NO_ACK;
            break;
        }
        else if(bitPos < 12)
        {
            /* it was within arbitration field */
            if(_debug)
            {
                Serial.printf("Arbitration error (bit %d / %d)\n", bitPos, bits);
            }
            TxErrArbitration++;
            ret = ERR_ARBRITATION;
        }
        else if(bitPos >= bits)
        {
            /* all bits transferred */
            if(_debug)
            {
                Serial.printf("Transferred\n");
            }
            TxSuccess++;
            return ERR_OK;
        }
        else
        {
            if(_debug)
            {
                Serial.printf("Collision (bit %d / %d)\n", bitPos, bits);
            }
            TxErrCollision++;
            ret = ERR_COLLISION;
        }
    }
    
    return ret;
}

#endif
