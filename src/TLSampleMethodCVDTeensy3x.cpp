/*
 * TLSampleMethodCVD.cpp - Capacitive sensing implementation using CVD method
 * for TouchLibrary for Arduino
 * 
 * https://github.com/AdmarSchoonen/TLSensor
 * Copyright (c) 2016 - 2017 Admar Schoonen
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

#include <stdint.h>
#include "TouchLib.h"
#include "TLSampleMethodCVDTeensy3x.h"

#include "BoardID.h"

#if IS_TEENSY3X

/*
 * Section below maps pin number to ADC channels and is copied from
 * arduino-1.8.5/hardware/teensy/avr/cores/teensy3/analog.c
 */
// The SC1A register is used for both software and hardware trigger modes of operation.

#if defined(__MK20DX128__)
static const uint8_t pin2sc1a[] = {
	5, 14, 8, 9, 13, 12, 6, 7, 15, 4, 0, 19, 3, 21, // 0-13 -> A0-A13
	5, 14, 8, 9, 13, 12, 6, 7, 15, 4, // 14-23 are A0-A9
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // 24-33 are digital only
	0, 19, 3, 21, // 34-37 are A10-A13
	26,	   // 38 is temp sensor
	22,	   // 39 is vref
	23	    // 40 is unused analog pin
};
#elif defined(__MK20DX256__)
static const uint8_t pin2sc1a[] = {
	5, 14, 8, 9, 13, 12, 6, 7, 15, 4, 0, 19, 3, 19+128, // 0-13 -> A0-A13
	5, 14, 8, 9, 13, 12, 6, 7, 15, 4, // 14-23 are A0-A9
	255, 255, // 24-25 are digital only
	5+192, 5+128, 4+128, 6+128, 7+128, 4+192, // 26-31 are A15-A20
	255, 255, // 32-33 are digital only
	0, 19, 3, 19+128, // 34-37 are A10-A13
	26,     // 38 is temp sensor,
	18+128, // 39 is vref
	23      // 40 is A14
};
#elif defined(__MKL26Z64__)
static const uint8_t pin2sc1a[] = {
	5, 14, 8, 9, 13, 12, 6, 7, 15, 11, 0, 4+64, 23, // 0-12 -> A0-A12
	255, // 13 is digital only (no A13 alias)
	5, 14, 8, 9, 13, 12, 6, 7, 15, 11, 0, 4+64, 23, // 14-26 are A0-A12
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // 27-37 unused
	26, // 38=temperature
	27  // 39=bandgap ref (PMC_REGSC |= PMC_REGSC_BGBE)
};
#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
static const uint8_t pin2sc1a[] = {
	5, 14, 8, 9, 13, 12, 6, 7, 15, 4, 3, 19+128, 14+128, 15+128, // 0-13 -> A0-A13
	5, 14, 8, 9, 13, 12, 6, 7, 15, 4, // 14-23 are A0-A9
	255, 255, 255, 255, 255, 255, 255, // 24-30 are digital only
	14+128, 15+128, 17, 18, 4+128, 5+128, 6+128, 7+128, 17+128,  // 31-39 are A12-A20
	255, 255, 255, 255, 255, 255, 255, 255, 255,  // 40-48 are digital only
	10+128, 11+128, // 49-50 are A23-A24
	255, 255, 255, 255, 255, 255, 255, // 51-57 are digital only
	255, 255, 255, 255, 255, 255, // 58-63 (sd card pins) are digital only
	3, 19+128, // 64-65 are A10-A11
	23, 23+128,// 66-67 are A21-A22 (DAC pins)
	1, 1+128,  // 68-69 are A25-A26 (unused USB host port on Teensy 3.5)
	26,	// 70 is Temperature Sensor
	18+128     // 71 is Vref
};
#endif
/* End of copied code */

void TLSetAdcReferencePin(int pin)
{
	volatile uint32_t * ADCx_CFG2 = &ADC0_CFG2;
	volatile uint32_t * ADCx_SC1A = &ADC0_SC1A;
	uint8_t channel;

	if (pin >= sizeof(pin2sc1a))
		return;

	channel = pin2sc1a[pin];

	if (channel == 255)
		return;

	#if IS_TEENSY32_WITH_ADC1
	if ((channel) & 0x80) {
		*ADCx_CFG2 = ADC1_CFG2;
		*ADCx_SC1A = ADC1_SC1A;
	}
	#endif

	#if IS_TEENSYLC
	if ((pin - A0) & 0x40) {
		*ADCx_CFG2 &= ~ADC_CFG2_MUXSEL;
	} else {
		*ADCx_CFG2 |= ADC_CFG2_MUXSEL;
	}
	#endif
	*ADCx_SC1A = (channel) & 0x1F;
}

int TLAnalogRead(int pin)
{
	// Teensy accepts both Axx and xx notation; use Axx here
	return analogRead(pin);
}
#endif
