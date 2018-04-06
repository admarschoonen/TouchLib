/*
 * BoardID.h - Capacitive sensing library based on TL method for Arduino
 * https://github.com/AdmarSchoonen/TLSensor
 * Copyright (c) 2016, 2017 Admar Schoonen
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef BoardID_h
#define BoardID_h

#if defined(SPARK)
        #include "application.h"
        #include "Particle.h"
        #include <math.h>
#else
	#if ARDUINO >= 100
		#include "Arduino.h"
	#else
		#include "WProgram.h"
		#include "WConstants.h"
	#endif

	#if defined(AVR)
	        #if ARDUINO >= 100
	                #include <avr/io.h>
	        #else
	                #include "pins_arduino.h"
	                #include <avr/io.h>
	        #endif
	#elif defined(ARDUINO_ARCH_ESP32)
	        #include <math.h>
	        #include <cstddef>
	#endif
#endif

#define IS_AVR			(defined(SIGNATURE_0))

#define IS_ATMEGA32X		(defined(__AVR_ATmega328P__) || \
		defined(__AVR_ATmega328__))

#define IS_ATMEGA16X		(defined(__AVR_ATmega168__))

#define IS_ATMEGA32U4		(defined(__AVR_ATmega32U4__))

#define IS_ATMEGA16X_32X_32U4	(IS_ATMEGA16X || IS_ATMEGA32X || IS_ATMEGA32U4)

/* 
 * From
 * http://electronics.stackexchange.com/questions/31048/can-an-atmega-or-
 * attiny-device-signature-be-read-while-running
 * and http://www.avrfreaks.net/forum/device-signatures.
 */
#define IS_ATMEGA128X		(defined(SIGNATURE_0) && \
		defined(SIGNATURE_1) && (SIGNATURE_0 == 0x1E) && \
		(SIGNATURE_1 == 0X97))

#define IS_ATMEGA256X		(defined(SIGNATURE_0) && \
		defined(SIGNATURE_1) && (SIGNATURE_0 == 0x1E) && \
		(SIGNATURE_1 == 0X98))

#define IS_ATMEGA128X_256X	(IS_ATMEGA128X || IS_ATMEGA256X)

#define IS_ATMEGA		(IS_ATMEGA16X_32X_32U4 || \
		IS_ATMEGA128X_256X)

/*
 * From http://forum.arduino.cc/index.php?topic=199571.0
 */
#define IS_ATTINY_24		(defined(SIGNATURE_0) && \
		defined(SIGNATURE_1) && defined(SIGNATURE_2) && \
		(SIGNATURE_0 == 0x1E) && (SIGNATURE_1 == 0x91) && \
		(SIGNATURE_2 == 0x0B))

#define IS_ATTINY_44		(defined(SIGNATURE_0) && \
		defined(SIGNATURE_1) && defined(SIGNATURE_2) && \
		(SIGNATURE_0 == 0x1E) && (SIGNATURE_1 == 0x92) && \
		(SIGNATURE_2 == 0x07))

#define IS_ATTINY_84		(defined(SIGNATURE_0) && \
		defined(SIGNATURE_1) && defined(SIGNATURE_2) && \
		(SIGNATURE_0 == 0x1E) && (SIGNATURE_1 == 0x93) && \
		(SIGNATURE_2 == 0x0C))

#define IS_ATTINY_25		(defined(SIGNATURE_0) && \
		defined(SIGNATURE_1) && defined(SIGNATURE_2) && \
		(SIGNATURE_0 == 0x1E) && (SIGNATURE_1 == 0x91) && \
		(SIGNATURE_2 == 0x08))

#define IS_ATTINY_45		(defined(SIGNATURE_0) && \
		defined(SIGNATURE_1) && defined(SIGNATURE_2) && \
		(SIGNATURE_0 == 0x1E) && (SIGNATURE_1 == 0x92) && \
		(SIGNATURE_2 == 0x06))

#define IS_ATTINY_85		(defined(SIGNATURE_0) && \
		defined(SIGNATURE_1) && defined(SIGNATURE_2) && \
		(SIGNATURE_0 == 0x1E) && (SIGNATURE_1 == 0x93) && \
		(SIGNATURE_2 == 0x0B))

#define IS_ATTINY_2X		(IS_ATTINY_24 || IS_ATTINY_25)

#define IS_ATTINY_4X		(IS_ATTINY_44 || IS_ATTINY_45)

#define IS_ATTINY_8X		(IS_ATTINY_84 || IS_ATTINY_85)

#define IS_ATTINY_X4		(IS_ATTINY_24 || IS_ATTINY_44 || IS_ATTINY_84)

#define IS_ATTINY_X5		(IS_ATTINY_25 || IS_ATTINY_45 || IS_ATTINY_85)

#define IS_ATTINY		(IS_ATTINY_X4 || IS_ATTINY_X5)

/*
 * Teensy 3.0: MK20DX128
 * Teensy 3.1: MK20DX256
 * Teensy LC:  MKL26Z64
 * Teensy 3.2: MK20DX256
 * Teensy 3.5: MK64FX512 (does not have touchRead()!)
 * Teensy 3.6: MK64FX1M0
*/

#define IS_TEENSYLC			(defined(__MKL26Z64__))
#define IS_TEENSY30			(defined(__MK20DX128__))
#define IS_TEENSY31			(defined(__MK20DX256__))
#define IS_TEENSY32			(defined(__MK20DX256__))
#define IS_TEENSY35			(defined(__MK64FX512__))
#define IS_TEENSY36			(defined(__MK66FX1M0__))
#define IS_TEENSY3X			(IS_TEENSY30 || IS_TEENSY31 || \
		IS_TEENSY32 || IS_TEENSY35 || IS_TEENSY36)

#define IS_TEENSY3X_WITH_TOUCHREAD	(IS_TEENSY30 || IS_TEENSY31 || \
		IS_TEENSY32 || IS_TEENSY36)

#define IS_TEENSY_WITH_TOUCHREAD	(IS_TEENSYLC || \
		IS_TEENSY3X_WITH_TOUCHREAD)

#define IS_TEENSY32_WITH_ADC1		(IS_TEENSY31 || IS_TEENSY32 || \
		IS_TEENSY35 || IS_TEENSY36)

/*
 * From
 * https://community.particle.io/t/preprocessor-ifdef-to-detect-platform-type-core-photon-at-compile-time/13085
 */
#define IS_PARTICLE			(defined(SPARK) || defined(PLATFORM_ID))

/* For ESP32 */
#define IS_ESP32			(defined(ARDUINO_ARCH_ESP32))

#endif
