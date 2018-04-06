---
layout: page
title: Supported Boards
permalink: /getting-started/supported-boards/
in_menu: false
weight: 100
in_book: 1
---

Currently TouchLib supports the following boards:

| Board | Processor | Capacitive sensing method | Resistive sensing method | Max number of sensors | Notes | 
|-----|-----|-----|-----|-----:|-----|
| [Arduino UNO](https://store.arduino.cc/arduino-uno-rev3) | ATmega328P | CVD method | analogRead() method | 6 | [1,2] |
| [Arduino Mega](https://www.arduino.cc/en/Main/ArduinoBoardMega2560) | ATmega2560 | CVD method | analogRead() method | 32 | [2] |
| [Arduino Lilypad USB](https://store.arduino.cc/lilypad-arduino-usb) | ATmega32u4 | CVD method | analogRead() method | 6 | [1,2] |
| [Teensy 3.2](https://www.pjrc.com/teensy/teensy31.html) | MK20DX256 | touchRead() method | analogRead() method | 32 | [3] |
| [Particle Photon](https://www.particle.io/products/hardware/photon-wifi-dev-kit) | STM32F205 | CVD method | analogRead() method | 16 | [4] |
| [ESP32Dev Board](https://github.com/espressif/arduino-esp32) | ESP32 | touchRead() method | analogRead() method | 32 | |

If a board is not listed but uses the same processor as one of the boards
listed above, there is a good chance that it will work as well.

Notes:

[1] TouchLib is large and the Atmel ATmega328 / ATmega32u4 processors only have
limited RAM and flash; TouchLib is therefore limited to only 6 sensors maximum
and consumes up to 77% of RAM and 86% of flash.

[2] ATmega328, ATmega32u4 and ATmega1280 / ATmega2560 are slow processors and
therefore update rate and noise performance are not as good as with some other
processors.

[3] TouchLib is developed and tested on a Teensy 3.2 but should also work on
Teensy LC, Teensy 3.0, 3.1 and 3.6. Note that the Teensy 3.5 doesn't have the
hardware peripheral to do capacitive sensing and is thus not supported at this
moment.

[4] TouchLib is developed and tested on a Particle Photon but should also work
on any Particle board that uses the STM32F205 (such as the Electron). Note that
the 1st generation Particle boards (Spark Core) and the 3rd generation boards
(Argon, Boron and Xenon) are using a different processor (STM32F103
respectively Nordic nRF52840) and are not supported at this moment.

## Untested boards
Boards that are untested but might work:
* Any Arduino compatible board based on an ATmega328 processor.
* Any Arduino compatible board based on an ATmega1280 or ATmega2560 processor.
* Any Teensy compatible board based on MKL26Z64 (Teensy LC), MK20DX128 (Teensy
  3.0), MK20DX256 (Teensy 3.1 / 3.2), or MK66FX1M0 (Teensy 3.6). Note that
  MK64FX512 (Teensy 3.5) is __not__ supported currently since it lacks the
  touchRead() function.
* Any Particle compatible board based on an STM32F2xx processor.
* Any Arduino compatible board based on an ESP32 processor.

When using an untested board, please check that the pin mappings from GPIO port
/ pin to Arduino pin number and from ADC input to Arduino analog pin number
(pin numbers that are prefixed with __A__) are the same as in the original
supported board.

## Unsupported boards
TouchLib does not work on ATtiny processors as these processors
simply have way to little RAM and flash for TouchLib.

