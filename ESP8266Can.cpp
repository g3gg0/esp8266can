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

#define INT_RX_BUFFERS 8
#define INT_RX_BUFFER_SIZE (1+11+3+4+64+15+1+2 + 10)

#define COUNT(x) (sizeof(x) / sizeof((x)[0]))

static uint8_t int_gpio_rx = 0;
static uint32_t int_bit_time = 0;
static uint32_t int_idle_time = 0;
static uint32_t int_receive_buf[INT_RX_BUFFERS][INT_RX_BUFFER_SIZE];
static uint8_t int_buffer_num_read = 0;
static uint8_t int_buffer_num = 0;
static uint8_t int_buffer_pos = 0;

static volatile uint32_t int_count = 0;
static volatile uint32_t int_bits_set = 0;

uint8_t I2SBufferTxData[512];
uint8_t I2SBufferRxData[16384];


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
            if(!(GPIO_REG_READ(GPIO_IN_ADDRESS) & pinRegisterRx))
            {
                /* return busy */
                xt_wsr_ps(_state);
                return ERR_BUSY_LINE;
            }
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
                Serial.printf(txRecessive ? "1" : "0");
                
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
                //volatile uint32_t dummy = GPIO_REG_READ(GPIO_IN_ADDRESS);
                bool rxRecessive = ((GPIO_REG_READ(GPIO_IN_ADDRESS) & pinRegisterRx) != 0);
                Serial.printf(rxRecessive ? "1 " : "0 ");
                
                /* and check if the RX line is the same as the TX line */
                if(0 && rxRecessive != txRecessive)
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
    
    void ICACHE_RAM_ATTR rxInterruptHandler()
    {
        static uint32_t lastTime = 0;
        static bool lastChangeRecessive = true;
        uint32_t lastChangeTime = _getCycleCountRaw();
        
        while(true)
        {
            bool rxRecessive = ((GPIO_REG_READ(GPIO_IN_ADDRESS) & int_gpio_rx) != 0);
            
            /* nothing changed... */
            if(lastChangeRecessive == rxRecessive)
            {
                /* no change for a few bit periods, return */
                if((_getCycleCountRaw() - lastChangeTime) > 10 * int_idle_time)
                {
                    return;
                }
                continue;
            }
            
            lastChangeRecessive = rxRecessive;
            lastChangeTime = _getCycleCountRaw();
            
            uint32_t thisTime = _getCycleCountRaw();
            uint32_t delta = (thisTime - lastTime);
            
            /* a new message appears */
            if(int_buffer_pos > 0 && delta > int_idle_time)
            {
                /* make sure we start with a dominant bit */
                if(rxRecessive)
                {
                    return;
                }
                
                int_receive_buf[int_buffer_num][int_buffer_pos] = 0xFFFFFFFF;
                int_buffer_num++;
                int_buffer_pos = 0;
                
                /* all buffers full, start over */
                if(int_buffer_num >= INT_RX_BUFFERS)
                {
                    int_buffer_num = 0;
                    int_buffer_pos = 0;
                }
            }
            else if(int_buffer_pos < INT_RX_BUFFER_SIZE - 1)
            {
                uint32_t bitCount = (delta + int_bit_time / 10) / int_bit_time;
                
                int_receive_buf[int_buffer_num][int_buffer_pos] = bitCount;
                int_buffer_pos++;
            }
            
            lastTime = thisTime;
        }
    }

    static void ICACHE_RAM_ATTR i2sInterrupt(ESP8266Can *can) 
    {
        //can->IntCount++;
        int_count++;
        
        uint32_t slc_intr_status = READ_PERI_REG(SLC_INT_STATUS);
        WRITE_PERI_REG(SLC_INT_CLR, 0xFFFFFFFF);
        
        if(!slc_intr_status)
        {
            Serial.printf("int %d, zero status: 0x%08X\n", int_count, slc_intr_status);       
        }

        if((slc_intr_status & SLC_RX_EOF_INT_ST))
        {
            struct slc_queue_item *completed = (struct slc_queue_item*)READ_PERI_REG(SLC_RX_EOF_DES_ADDR);
            
            for(int pos = 0; pos < completed->blocksize / 4; pos++)
            {
                if(((uint32_t*)completed->buf_ptr)[pos] != 0)
                {
                    int_bits_set++;
                }
            }
            
            //Serial.printf("int %d, completed: 0x%08X\n", int_count, completed);        
            slc_intr_status &= ~SLC_RX_EOF_INT_ST;
        }
        
        if(slc_intr_status)
        {
            Serial.printf("int %d, unknown status: 0x%08X\n", int_count, slc_intr_status);        
        }
    }
}

ESP8266Can::ESP8266Can(uint32_t rate, uint8_t gpio_tx, uint8_t gpio_rx) : 
    _rate(rate), 
    _gpio_tx(gpio_tx), 
    _gpio_rx(gpio_rx),
    _maxTries(1024)
{
    digitalWrite(_gpio_tx, HIGH);
    pinMode(D6, INPUT);
    pinMode(_gpio_rx, INPUT);
    pinMode(_gpio_tx, OUTPUT);
}

ESP8266Can::~ESP8266Can()
{
}

void ESP8266Can::StartRx()
{
    int_gpio_rx = pinRegisterRx();
    int_bit_time = cyclesBit();
    int_idle_time = 10 * int_bit_time;
    //attachInterrupt(_gpio_rx, &rxInterruptHandler, CHANGE);
    
    Serial.printf("InitI2S\n");
    InitI2S();
    //Serial.printf("StartI2S\n");
    //StartI2S();
    Serial.printf("Done\n");
}

void ESP8266Can::StartI2S()
{
	SET_PERI_REG_MASK(SLC_TX_LINK, SLC_TXLINK_START);
	SET_PERI_REG_MASK(SLC_RX_LINK, SLC_RXLINK_START);
}

void ESP8266Can::StopI2S()
{
	CLEAR_PERI_REG_MASK(SLC_TX_LINK, SLC_TXLINK_START);
	CLEAR_PERI_REG_MASK(SLC_RX_LINK, SLC_RXLINK_START);
}

void ESP8266Can::InitI2S(void) 
{   
    memset(I2SBufferTxData, 0xFF, sizeof(I2SBufferTxData));
    memset(I2SBufferRxData, 0xFF, sizeof(I2SBufferRxData));

    /* prepare linked DMA descriptors, having EOF set for all */
    for(int num = 0; num < COUNT(I2SBufferRx); num++)
    {
        int nextNum = (num + 1) % COUNT(I2SBufferRx);
        int blockSize = sizeof(I2SBufferRxData) / COUNT(I2SBufferRx);
        
        I2SBufferRx[num].owner = 1;
        I2SBufferRx[num].eof = 1;
        I2SBufferRx[num].sub_sof = 0;
        I2SBufferRx[num].datalen = blockSize;
        I2SBufferRx[num].blocksize = blockSize;
        I2SBufferRx[num].buf_ptr = (uint32_t)I2SBufferRxData + num * blockSize;
        I2SBufferRx[num].unused = 0;
        I2SBufferRx[num].next_link_ptr = (uint32_t)&I2SBufferRx[nextNum];
        
        Serial.printf("I2SBufferRx[%d] (0x%08X) with %d bytes, next %d (0x%08X)\n", num, &I2SBufferRx[num], I2SBufferRx[num].datalen, nextNum, I2SBufferRx[num].next_link_ptr);
    }
    
    /* this zero-buffer block implements the latch/reset signal */
    for(int num = 0; num < COUNT(I2SBufferTx); num++)
    {
        int nextNum = (num + 1) % COUNT(I2SBufferTx);
        int blockSize = sizeof(I2SBufferTxData) / COUNT(I2SBufferTx);
        
        I2SBufferTx[num].owner = 1;
        I2SBufferTx[num].eof = 0;
        I2SBufferTx[num].sub_sof = 0;
        I2SBufferTx[num].datalen = blockSize;
        I2SBufferTx[num].blocksize = blockSize;
        I2SBufferTx[num].buf_ptr = (uint32_t)I2SBufferTxData + num * blockSize;
        I2SBufferTx[num].unused = 0;
        I2SBufferTx[num].next_link_ptr = (uint32_t)&I2SBufferTx[nextNum];
        
        Serial.printf("I2SBufferTx[%d] (0x%08X) with %d bytes, next %d (0x%08X)\n", num, &I2SBufferTx[num], I2SBufferTx[num].datalen, nextNum, I2SBufferTx[num].next_link_ptr);
    }

    /* configure RDX0/GPIO3 for output. it is the only supported pin unfortunately. */
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_I2SI_DATA);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_I2SO_DATA);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_I2SI_WS);
    
    
	SET_PERI_REG_MASK(SLC_CONF0, SLC_TX_LOOP_TEST|SLC_RXLINK_RST|SLC_TXLINK_RST|SLC_AHBM_RST|SLC_AHBM_FIFO_RST);
	CLEAR_PERI_REG_MASK(SLC_CONF0, SLC_RXLINK_RST|SLC_TXLINK_RST|SLC_AHBM_RST|SLC_AHBM_FIFO_RST);
  
	/* Enable and configure DMA */
    //CLEAR_PERI_REG_MASK(SLC_CONF0, (SLC_MODE<<SLC_MODE_S));
    //SET_PERI_REG_MASK(SLC_CONF0, (1<<SLC_MODE_S));
    //SET_PERI_REG_MASK(SLC_RX_DSCR_CONF, SLC_INFOR_NO_REPLACE|SLC_RX_EOF_MODE|SLC_TOKEN_NO_REPLACE);
    //CLEAR_PERI_REG_MASK(SLC_RX_DSCR_CONF, SLC_RX_FILL_EN|SLC_RX_FILL_MODE);
    
    /* clear all interrupt flags */

    /* set up DMA */
    CLEAR_PERI_REG_MASK(SLC_CONF0, (SLC_MODE<<SLC_MODE_S));
    SET_PERI_REG_MASK(SLC_CONF0, (1<<SLC_MODE_S));
    SET_PERI_REG_MASK(SLC_RX_DSCR_CONF, SLC_INFOR_NO_REPLACE|SLC_TOKEN_NO_REPLACE);
    //CLEAR_PERI_REG_MASK(SLC_RX_DSCR_CONF, SLC_RX_FILL_EN|SLC_RX_EOF_MODE|SLC_RX_FILL_MODE);

    
    Serial.printf("SLC_RX_LINK\n");
    /* configure the first descriptor. TX_LINK isnt used, but has to be configured */
    CLEAR_PERI_REG_MASK(SLC_RX_LINK, SLC_RXLINK_DESCADDR_MASK);
    CLEAR_PERI_REG_MASK(SLC_TX_LINK, SLC_TXLINK_DESCADDR_MASK);
    SET_PERI_REG_MASK(SLC_RX_LINK, ((uint32)I2SBufferRx) & SLC_RXLINK_DESCADDR_MASK);
    SET_PERI_REG_MASK(SLC_TX_LINK, ((uint32)I2SBufferTx) & SLC_TXLINK_DESCADDR_MASK);

    /* set up interrupt */
    ETS_SLC_INTR_DISABLE();
    SET_PERI_REG_MASK(SLC_INT_CLR, 0xffffffff);
    WRITE_PERI_REG(SLC_INT_ENA, SLC_INTEREST_EVENT);
    ETS_SLC_INTR_ATTACH((int_handler_t)i2sInterrupt, (void *)this);
    ETS_SLC_INTR_ENABLE();
    
    
    /* configure I2S subsystem */
    I2S_CLK_ENABLE();
    CLEAR_PERI_REG_MASK(I2SCONF, I2S_I2S_RESET_MASK);
    SET_PERI_REG_MASK(I2SCONF, I2S_I2S_RESET_MASK);
    CLEAR_PERI_REG_MASK(I2SCONF, I2S_I2S_RESET_MASK);

    /* Select 16bits per channel (FIFO_MOD=0), no DMA access (FIFO only) */

	CLEAR_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN|(I2S_I2S_RX_FIFO_MOD<<I2S_I2S_RX_FIFO_MOD_S)|
			(I2S_I2S_TX_FIFO_MOD<<I2S_I2S_TX_FIFO_MOD_S)|
			(I2S_I2S_TX_DATA_NUM<<I2S_I2S_TX_DATA_NUM_S)|
			(I2S_I2S_RX_DATA_NUM<<I2S_I2S_RX_DATA_NUM_S) );

    /* Enable DMA in I2S subsystem */
    SET_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN);
	WRITE_PERI_REG(I2SRXEOF_NUM, sizeof(I2SBufferRxData) / COUNT(I2SBufferRx));

	//Playing with these shows no impact unless specifically selected to be wrong.  (no effect)
	CLEAR_PERI_REG_MASK(I2SCONF_CHAN, (I2S_TX_CHAN_MOD<<I2S_TX_CHAN_MOD_S)|(I2S_RX_CHAN_MOD<<I2S_RX_CHAN_MOD_S));
	SET_PERI_REG_MASK(I2SCONF_CHAN, (0<<I2S_TX_CHAN_MOD_S)|(0<<I2S_RX_CHAN_MOD_S));
    
	SET_PERI_REG_MASK(I2SINT_CLR, I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);
	CLEAR_PERI_REG_MASK(I2SINT_CLR, I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);

    Serial.printf("I2SCONF\n");
    uint32_t bestClkmDiv = 4;
    uint32_t bestBckDiv = 17;
    
    /* configure the rates */
    CLEAR_PERI_REG_MASK(I2SCONF, I2S_TRANS_SLAVE_MOD|
                        (I2S_BITS_MOD<<I2S_BITS_MOD_S)|
                        (I2S_BCK_DIV_NUM <<I2S_BCK_DIV_NUM_S)|
                        (I2S_CLKM_DIV_NUM<<I2S_CLKM_DIV_NUM_S));
    SET_PERI_REG_MASK(I2SCONF, I2S_RIGHT_FIRST|I2S_MSB_RIGHT|I2S_RECE_SLAVE_MOD|
                        I2S_RECE_MSB_SHIFT|I2S_TRANS_MSB_SHIFT|
                        (((bestBckDiv-1)&I2S_BCK_DIV_NUM )<<I2S_BCK_DIV_NUM_S)|
                        (((bestClkmDiv-1)&I2S_CLKM_DIV_NUM)<<I2S_CLKM_DIV_NUM_S));
                        
	//enable int
	SET_PERI_REG_MASK(I2SINT_ENA, I2S_I2S_TX_REMPTY_INT_ENA|I2S_I2S_TX_WFULL_INT_ENA|I2S_I2S_RX_REMPTY_INT_ENA|I2S_I2S_TX_PUT_DATA_INT_ENA|I2S_I2S_RX_TAKE_DATA_INT_ENA);  //I don't believe these are needed.

    /* set up DMA done, start */
    StartI2S();
    
	//Start transmission
	SET_PERI_REG_MASK(I2SCONF,I2S_I2S_TX_START|I2S_I2S_RX_START);
    Serial.printf("done\n");
}


void ESP8266Can::Loop()
{
    delay(250);
    Serial.printf("Interrupts: %d, bits: %d\n", int_count, int_bits_set);
    
    if(int_buffer_num_read != int_buffer_num)
    {
        int bitPos = 0;
        bool bitLevel = true;
        
        Serial.printf("Buffer %02d START ", int_buffer_num_read);
        
        for(int pos = 0; pos < INT_RX_BUFFER_SIZE; pos++)
        {
            uint32_t bits = int_receive_buf[int_buffer_num_read][pos];
            
            if(bits > 10)
            {
                break;
            }
            
            bitLevel = !bitLevel;
            
        
            for(int bitNum = 0; bitNum < bits; bitNum++)
            {
                switch(bitPos)
                {
                    case 1:
                    case 12:
                    case 15:
                    case 19:
                        Serial.printf(" ");
                        break;
                        
                    default:
                    break;
                }
                Serial.printf(bitLevel ? "1" : "0");
                bitPos++;
            }
        }
        Serial.printf("\n");
        
        int_buffer_num_read = (int_buffer_num_read + 1) % INT_RX_BUFFERS;
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
    
    return crc;
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
            ret = ERR_BUSY_LINE;
        }
        else if(bitPos == ERR_TRCV_ERR)
        {
            /* the line wasn't recessive for some time */
            Serial.printf("Transceiver error\n");
            ret = ERR_TRCV_ERR;
            break;
        }
        else if(bitPos == ERR_NO_ACK)
        {
            /* the line wasn't recessive for some time */
            Serial.printf("No ACK error\n");
            ret = ERR_NO_ACK;
            break;
        }
        else if(bitPos < 12)
        {
            /* it was within arbitration field */
            Serial.printf("Arbitration error (bit %d / %d)\n", bitPos, bits);
            ret = ERR_ARBRITATION;
        }
        else if(bitPos >= bits)
        {
            /* all bits transferred */
            Serial.printf("Transferred\n");
            ret = ERR_OK;
            break;
        }
        else
        {
            Serial.printf("Collision (bit %d / %d)\n", bitPos, bits);
            ret = ERR_COLLISION;
        }
        
        /* wait a msec and try again */
        delay(10);
        
        ESP.wdtFeed();
    }
    
    return ret;
}

#endif
