# TouchLib
A generic touch library for Arduino and Arduino-like boards, usable for both resistive and capacitive touch.

TouchLib aims to distinguish itself from other capacitive sensing libraries in 3 ways:
* Robust capacitive sensing: TouchLib contains many filters and an advanced state machine to make capacitive sensing more robust
* Multi-platform: TouchLib works on a range of different boards and processors
* Easy to use: TouchLib contains a code generator that will guide you through the tuning process and serves as a decent initial setting for your sensors

# Supported hardware
TouchLib works on boards with the following processors:
* Atmel ATmega32 (Arduino UNO or compatible)
* Atmel ATmega2560 (Arduino Mega or compatible)
* Atmel ATmega32u4 (Arduino Lilypad USB or compatible)
* Freescale MK20DX256 (Teensy 3.x or compatible)
* ST STM32F205 (Particle Photon or compatible)
* Espressif ESP32 (ESP32Dev Board or compatible)

For a complete overview, see [here](https://admarschoonen.github.io/TouchLib/getting-started/supported-boards/).

# Documentation
Well, there isn't much yet. For general capacitive sensing information, you can have a look [here](https://admarschoonen.github.io/TouchLib/). This is a work in (slow) progress and will eventually also contain stuff like API documentation and a howto.

For now, if you want to get started the easiest way is to follow these steps:
* Attach some capacitive sensors to your Arduino
* Download the latest [release](https://github.com/admarschoonen/TouchLib/releases)
* Install as a library under Arduino (Sketch -> Include Library -> Add .ZIP Library...)
* Select the right Arduino board (Tools -> Board)
* Select the right port (Tools -> Port)
* Open Example00 (File -> Examples -> TouchLib -> Example00SemiAutoTuning)
* Do not modify the code or even look at it; just upload it to your board
* Open the serial monitor (Tools -> Serial Monitor)
* Answer the questions and touch the buttons
* The Arduino will now generate an example program on the serial monitor. Select the generate code and copy to clipboard
* Create a new sketch (File -> New)
* Replace the contents of the new sketch with the code from the clipboard
* Upload the code and play with the first sensor
* Modify the code to see the output of the other sensors or use it as a starting point for your next project
