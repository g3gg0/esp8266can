# ESP8266Can

WARNING! Current status: UNTESTED

Arduino CAN library for ESP8266 (TX only yet)

## Requirements

Pick any valid GPIOs and pass them to the constructor.

## API Documentation

#### SendMessage(uint16_t id, uint8_t length, uint8_t *data, bool req_remote = false, bool self_ack = true)
Send a CAN message to given 11 bit identifier and 0-8 bytes of payload