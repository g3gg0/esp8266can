# ESP8266Can

Arduino CAN library for ESP8266 (Rx and Tx)

## Description

This library provides basic CAN controller features for the ESP8266.
It features bit-banging Tx code and an interrupt driven Rx path using I2S.

Sounds good? Read the "Bugs / Deviations" chapter first.

For timing reasons, the Tx code is disabling interrupts for the time being in 

* busy detection,
* arbitration,
* and transmit phase

and can provide clean bit shape for up to 500 kBaud.

The Rx path uses the (hardwired) I2SI_DATA IO pin GPIO12 with a bit rate of
1.5 MBaud to get a 3x oversampling of the received bits.

The received I2S bitstream is being processed in interrupt context, as the data 
uses more than 3x it's net size. Processing raw I2S data in non-irq context thus
would require a lot of additional RAM. Processing time in user context would be
the same as in ISR context but due to RAM limitations, we could not delay 
processing some reasonable time. So the deterministic nature of processing is
the reason why it is better done in ISR context.

![Interrupt timing: High means ISR is running](/images/ISR_timing.png?raw=true "Interrupt timing: High means ISR is running")

The SLC TX interrupt (yeah, SLC's Tx is for I2S Rx) is firing every 1.33 msec and executes
635µs (idle) to 705µs (message received) which results in a CPU load of approx 47-52% just for the Rx path.

The good thing, the ISR is very stable in it's execution time and always is within the specified limits.

![Tx Message successfully decoded using PicoScope](/images/message.png?raw=true "Tx Message successfully decoded using PicoScope")

For arbitration and transceiver check during transmission, the bit-banging code requires
some extra Rx GPIO. We cannot re-use the GPIO12 for this "read-back" due to these reasons:

* would disconnect I2SI_DATA from the pin -> need to handle rx errors properly
* Rx pin is used during idle detection and arbitration -> would throw away all Rx messages while waiting for a free line

We also cannot magically use I2S "OUT" for Tx'ing because we always have to check what we send,
no matter if during arbitration or within payload.

The only workaround would be:

* XOR Rx and Tx line, lowpass, pass to a RS flip flop which again disables Tx to transceiver
* check Rx path if our message could be transmitted - or if it was cancelled during arbitration phase
Using this extra circuitry, we indeed could use I2S for Tx path too.
But this is a bit complex and bit banging works just good enough, being closer to the standard.
 

Hint:
The Tx code was (in some early state) tested against common vector tools and it has shown
to be stable enough that all messages were decodable. Also any case of arbitration
errors were handled gracefully. The current code base wasn't tested that detailed, but this 
will come as soon I receive an ordered 3.3V transceiver, which will simplify the wiring and setup.
Currently I need 3.3V for the ESP8266 and 5V/12V for the automotive transceiver.

However, you can connect Rx and Tx and see the library decoding it's own sent packets.

## Requirements

### Normal setup
You need an ESP8266 and a CAN transceiver (e.g. SN65HVD234)

Connect GPIO12 plus some other pin you have spare to the R pin (RX) and any other GPIO to 
the D (Tx) pin of the transceiver. Pass your custom GPIOs to the constructor.

### Loopback
Connect all pins, both Rx and your Tx pins together, you will now receive the messages you sent - without a transceiver.

### Software
To start Rx path, call StartRx() and then call Loop() peridically.
You have to pass the callback routine for new messages to the Loop() call.

It's prototype is:
  void cbr(uint16_t id, bool req, uint8_t length, uint8_t *payload, bool ack)
And will be called for every message received.

To send a message, call SendMessage with the right parameters.


## Bugs / Deviations

### I2S Engine
Currently the I2S engine bugs after ~20 minutes of operation.
This results in some weird behavior that raises interrupts instantly, signalling
that the current descriptor (2k buffer) was filled and can be processed.
This high interrupt load would block the user code from execution and causes a watchdog reset.

To mitigate this (yet unidentified) problem, the code detects this situation and resets the I2S engine.
This results in probably one buffer drop and if we have 100% bus load, drops up to 6 CAN messages.

![I2S error causing 100% load until I2S engine is restarted](/images/ISR_error.png?raw=true "I2S error")

This happens after 904.000 to 905.000 IRQs, which transferred 1.852.416.000 bytes or 463.104.000 I2S stereo frames.
With some code changes, it also sometimes happened after 10 minutes.

### No ACK for senders
Because this code is using the I2S engine to asynchronously sample the CAN Rx path, there is no
way of ACK'ing someone's message. If your ESP8266 will be the only listeners for this message,
the sender won't receive an ACK.
Troublesome for devices which insist on ACK and it is the only device on the bus beside the ESP8266.

### Incomplete code
This code is most probably not perfect. It may miss some essential stuff I didnt think of.
As it is not (yet) 100% tested in a productional environment, it could contain annoying bugs.

It does not handle:

* 29 bit identifiers
* error frames
* overload frames
* proper IFS timing

## API Documentation

#### SendMessage(uint16_t id, uint8_t length, uint8_t *data, bool req_remote = false, bool self_ack = true)
Send a CAN message to given 11 bit identifier and 0-8 bytes of payload

#### StartRx()
Setup I2S path and start I2S engine. Needs Loop() to be called periodically.

#### Loop(void (*cbr)(uint16_t id, bool req, uint8_t length, uint8_t *payload, bool ack))
Handle I2S status changes and process all messages received, passing them to the callback routine.

## Pictures of my prototype

My current prototype uses a ESP8266 NodeMcu (LoLin version) plus a SSD1306 display for visualizing statistics.

It uses the SSD1306 library from squix78 at https://github.com/squix78/esp8266-oled-ssd1306 which has nice
scrolling pages. Ideal during development and also when in use for serious stuff somewhen later this year.

![Whole setup, with SSD1306 blue/yellow](/images/Whole_prototype.jpg?raw=true "Whole setup, with SSD1306 blue/yellow")
![Scenario with Rx and Tx disconnected - detecting errors](/images/Display_errors.jpg?raw=true "Scenario with Rx and Tx disconnected - detecting errors")