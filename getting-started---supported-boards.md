---
layout: page
title: Supported Boards
permalink: /getting-started/supported-boards/
in_menu: false
weight: 100
in_book: 1
---

Currently TouchLib supports the following boards:

| Board | Capacitive sensing method | Resistive sensing method | Max number of sensors | Notes | 
|-----|-----|-----|-----:|-----|
| [Arduino UNO](https://store.arduino.cc/arduino-uno-rev3) | CVD method | analogRead() method | 6 | [1,2] |
| [Arduino Mega](https://www.arduino.cc/en/Main/ArduinoBoardMega2560) | CVD method | analogRead() method | 32 | [2] |
| [Arduino Flora](https://www.adafruit.com/product/659) | CVD method | analogRead() method | 6 | [1,2] |
| [Teensy 3.2](https://www.pjrc.com/teensy/teensy31.html) | touchRead() method | analogRead() method | 32 | |
| [Particle Photon](https://www.particle.io/products/hardware/photon-wifi-dev-kit) | CVD method | analogRead() method | 16 | |
| [ESP32Dev Board](https://github.com/espressif/arduino-esp32) | touchRead() method | analogRead() method | 32 | |

Notes:

[1] TouchLib is large and the Atmel ATmega328 / ATmega32u4 processors only have
limited RAM and flash; TouchLib is therefore limited to only 6 sensors maximum
and consumes up to 77% of RAM and 86% of flash.

[2] ATmega328, ATmega32u4 and ATmega1280 / ATmega2560 are slow processors and
therefore update rate and noise performance are not as good as with some other
processors.

## Untested boards
Boards that are untested but might work:
* Any Arduino compatible board based on an ATmega328 processor.
* Any Arduino compatible board based on an ATmega1280 or ATmega2560 processor.
* Any Teensy compatible board based on MKL26Z64 (Teensy LC), MK20DX128 (Teensy
  3.0), MK20DX256 (Teensy 3.1 / 3.2), or MK66FX1M0 (Teensy 3.6). Note that
  MK64FX512 (Teensy 3.5) is __not__ supported currently since it lacks the
  touchRead() function.
* Any Particle compatible board based on an STM32F2xx processor.

When using an untested board, please check that the pin mappings from GPIO port
/ pin to Arduino pin number and from ADC input to Arduino analog pin number
(pin numbers that are prefixed with __A__) are the same as in the original
supported board.

## Unsupported boards
TouchLib does not work on ATtiny processors as these processors
simply have way to small RAM and flash for TouchLib.

