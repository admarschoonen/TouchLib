/*
 * TLSampleMethodTouchRead.cpp - Capacitive sensing implementation using
 * TouchRead method for TouchLibrary for Arduino Teensy 3.x
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

#include "TouchLib.h"
#include "TLSampleMethodTouchRead.h"

#define TL_REFERENCE_VALUE_DEFAULT			((float) 2) /* 2 pF */
#define TL_SCALE_FACTOR_DEFAULT				((float) 1)
#define TL_OFFSET_VALUE_DEFAULT				((float) 0) /* pF */

#define TL_SET_OFFSET_VALUE_MANUALLY_DEFAULT	    	false

#define TL_RELEASED_TO_APPROACHED_THRESHOLD_DEFAULT	50.0
#define TL_APPROACHED_TO_RELEASED_THRESHOLD_DEFAULT	40.0
#define TL_APPROACHED_TO_PRESSED_THRESHOLD_DEFAULT	150.0
#define TL_PRESSED_TO_APPROACHED_THRESHOLD_DEFAULT	120.0

int TLSampleMethodTouchReadPreSample(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch)
{
	return 0;
}

int TLSampleMethodTouchReadSample(struct TLStruct * data, uint8_t nSensors,
	uint8_t ch, bool inv)
{
	int sample;
	
	/*
	 * touchRead() is only available on Teensy 3.x:
	 *
	 * Teensy 3.0: MK20DX128
	 * Teensy 3.1: MK20DX256
	 * Teensy LC:  MKL26Z64
	 * Teensy 3.2: MK20DX256
	 * Teensy 3.5: MK64FX512
	 * Teensy 3.6: MK64FX1M0
	*/

	#if !(defined(__MK20DX128__) || defined(__MK20DX256__) || \
		defined(__MKL26Z64__) || defined(__MK64FX512) || \
		defined(__MK64FX1M0__))

	/* Error! Not a Teensy! */
	sample = 0;

	#else
	struct TLStruct * dCh;
	int ch_pin;

	if (inv) {
		/* Pseudo differential measurements are not supported */
		sample = 0;
	} else {
		dCh = &(data[ch]);
		ch_pin = dCh->tlStructSampleMethod.touchRead.pin;
		sample = touchRead(ch_pin);
	}

	#endif

	return sample;
}

int TLSampleMethodTouchReadPostSample(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch)
{
        TLStruct * d;
        float tmp, scale;

        d = &(data[ch]);

        if (d->enableSlewrateLimiter) {
                scale = (float) ((TL_ADC_MAX + 1) << 2);
        } else {
                scale = ((float) (d->nMeasurementsPerSensor << 1)) *
                        ((float) (TL_ADC_MAX + 1));
        }

        tmp = d->raw / scale;
        tmp = d->scaleFactor * d->referenceValue * tmp;
        d->value = tmp;
        /* Capacitance can be negative due to noise! */

        return 0;
}

int TLSampleMethodTouchRead(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch)
{
	struct TLStruct * d;

	d = &(data[ch]);

	d->sampleMethodPreSample = TLSampleMethodTouchReadPreSample;
	d->sampleMethodSample = TLSampleMethodTouchReadSample;
	d->sampleMethodPostSample = TLSampleMethodTouchReadPostSample;

	d->tlStructSampleMethod.touchRead.pin = A0 + ch;


	d->referenceValue = TL_REFERENCE_VALUE_DEFAULT;
	d->offsetValue = TL_OFFSET_VALUE_DEFAULT;
	d->scaleFactor = TL_SCALE_FACTOR_DEFAULT;
	d->setOffsetValueManually = TL_SET_OFFSET_VALUE_MANUALLY_DEFAULT;

	d->releasedToApproachedThreshold =
		TL_RELEASED_TO_APPROACHED_THRESHOLD_DEFAULT;
	d->approachedToReleasedThreshold =
		TL_APPROACHED_TO_RELEASED_THRESHOLD_DEFAULT;
	d->approachedToPressedThreshold =
		TL_APPROACHED_TO_PRESSED_THRESHOLD_DEFAULT;
	d->pressedToApproachedThreshold =
		TL_PRESSED_TO_APPROACHED_THRESHOLD_DEFAULT;

	d->direction = TLStruct::directionPositive;
	d->sampleType = TLStruct::sampleTypeNormal;

	d->pin = &(d->tlStructSampleMethod.touchRead.pin);

	return 0;
}
