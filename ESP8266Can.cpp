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


extern "C"
{
    static inline uint32_t _getCycleCount()
    {
        uint32_t ccount;
        __asm__ __volatile__("rsr %0,ccount":"=a" (ccount));
        return ccount;
    }

    static can_error_t ICACHE_RAM_ATTR sendRawData(uint8_t* buffer, uint32_t cyclesBit, uint32_t cyclesSample, uint16_t skip, uint16_t bits, uint8_t pinRegisterTx, uint8_t pinRegisterRx)
    {
        uint32_t cyclesStart = _getCycleCount();
        
        /* make sure the line is recessive for 10 consecutive bits */
        while ((_getCycleCount() - cyclesStart) < 10 * cyclesBit)
        {
            if(!(GPIO_REG_READ(GPIO_IN_ADDRESS) & pinRegisterRx))
            {
                /* return collision */
                return ERR_BUSY_LINE;
            }
        }

        /* update bit timer */
        cyclesStart += 10 * cyclesBit;
        
        /* now write the bits */
        can_error_t collisionPos = ERR_OK;
        uint32_t bitNum = 0;
        uint32_t dominantBits = 0;
        uint32_t recessiveBits = 0;
        
        while (bitNum < bits)
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
                
                /* after 5 consecutive bits the same polarity, insert a stuff bit */
                if(recessiveBits == 5)
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
                
                /* wait till last bit (or stuff bit) ends */
                while (_getCycleCount() < cyclesStart)
                {
                }
                
                /* set the TX line */
                if(txRecessive)
                {
                    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, pinRegisterTx);
                    dominantBits = 0;
                    recessiveBits++;
                }
                else
                {
                    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, pinRegisterTx);
                    recessiveBits = 0;
                    dominantBits++;
                }
                
                /* wait until we shall sample the RX line */
                while ((_getCycleCount() - cyclesStart) < cyclesSample)
                {
                }

                /* do so, read the RX line status */
                bool rxRecessive = ((GPIO_REG_READ(GPIO_IN_ADDRESS) & pinRegisterRx) != 0);
                
                /* and check if the RX line is the same as the TX line */
                if(rxRecessive != txRecessive)
                {
                    if(rxRecessive)
                    {
                        /* abort, transceiver seems not to send. sent dominant, but received recessive */
                        collisionPos = ERR_TRCV_ERR;
                    }
                    else
                    {
                        /* abort, there was a collision */
                        collisionPos = (can_error_t)bitNum;
                    }
                    bits = 0;
                    break;
                }
                
                /* increment bit position */
                bitNum++;
                if(bitNum >= bits)
                {
                    /* if all bits were transferred, return */
                    break;
                }
                
                /* update start timestamp so the next cycle will wait till the bit time is done */
                cyclesStart += cyclesBit;
            }
        }

        /* reset TX line when done */
        GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, pinRegisterTx);
        
        return collisionPos;
    }
}

ESP8266Can::ESP8266Can(uint32_t rate, uint8_t gpio_tx, uint8_t gpio_rx) : 
    _rate(rate), 
    _gpio_tx(gpio_tx), 
    _gpio_rx(gpio_rx),
    _maxTries(1024)
{
  digitalWrite(_gpio_tx, HIGH);
  pinMode(_gpio_rx, INPUT);
  pinMode(_gpio_tx, OUTPUT);
}

ESP8266Can::~ESP8266Can()
{
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
    buffer[5 + length] = ((self_ack ? 0 : 1) << 8) | 0x7F;
    buffer[6 + length] = 0xF0;
    
    /* return the number of bits to be sent (without trailing recessives) */
    return (1 + 11 + 1 + 2 + 4 + (8 * length) + 15 + 1 + 2);
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
        
        can_error_t bitPos = sendRawData(buffer, cyclesBit(), cyclesSample(), LEADING_BITS, bits, pinRegisterTx(), pinRegisterRx());
        
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
        else if(bitPos < 12)
        {
            /* it was within arbitration field */
            Serial.printf("Arbitration error (bit %d)\n", bitPos);
            ret = ERR_ARBRITATION;
        }
        else if(bitPos == (1 + 11 + 3 + 4 + length * 8 + 15 + 1))
        {
            /* it was the acknowledge slot */
            Serial.printf("ACK was received (bit %d)\n", bitPos);
            ret = ERR_OK;
            break;
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
            Serial.printf("Collision (bit %d)\n", bitPos);
            ret = ERR_COLLISION;
        }
        
        /* wait a msec and try again */
        delay(1);
        
        ESP.wdtFeed();
    }
    
    return ret;
}

#endif
