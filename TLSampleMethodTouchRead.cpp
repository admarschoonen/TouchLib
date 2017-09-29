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

#define TL_REFERENCE_VALUE_DEFAULT			((float) 20000) /* 0.02 pF */
#define TL_SCALE_FACTOR_DEFAULT				((float) 1)
#define TL_OFFSET_VALUE_DEFAULT				((float) 0) /* pF */

#define TL_SET_OFFSET_VALUE_MANUALLY_DEFAULT	    	false

#define TL_RELEASED_TO_APPROACHED_THRESHOLD_DEFAULT	2.5
#define TL_APPROACHED_TO_RELEASED_THRESHOLD_DEFAULT	2.0
#define TL_APPROACHED_TO_PRESSED_THRESHOLD_DEFAULT	7.5
#define TL_PRESSED_TO_APPROACHED_THRESHOLD_DEFAULT	6.0
#define TL_TOUCHREAD_MAX				((1 << 16) - 1)

int TLSampleMethodTouchReadPreSample(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch)
{
	return 0;
}

int TLSampleMethodTouchReadSample(struct TLStruct * data, uint8_t nSensors,
	uint8_t ch, bool inv)
{
	int sample = 0;
	
	/* touchRead() is only available on Teensy 3.x and not Teensy 3.5 */

	#if IS_TEENSY_WITH_TOUCHREAD
	struct TLStruct * dCh;
	int ch_pin;

	dCh = &(data[ch]);
	ch_pin = dCh->tlStructSampleMethod.touchRead.pin;
	if (inv) {
		/* Pseudo differential measurements are not supported */
		sample = 0;
	} else {

		if (ch_pin >= 0) {
			sample = touchRead(ch_pin);
		}
	}
	/*Serial.print("ch: ");
	Serial.print(ch);
	Serial.print("; ch_pin: ");
	Serial.print(ch_pin);
	Serial.print("; sample: ");
	Serial.println(sample);
	#else
	Serial.println("Error! touchRead() not available!");*/

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
                scale = (float) ((TL_TOUCHREAD_MAX + 1) << 2);
        } else {
                scale = ((float) (d->nMeasurementsPerSensor << 1)) *
                        ((float) (TL_TOUCHREAD_MAX + 1));
        }

        tmp = d->raw / scale;
        tmp = d->scaleFactor * d->referenceValue * tmp;
        d->value = tmp;
        /* Capacitance can be negative due to noise! */

        return 0;
}


int TLSampleMethodTouchReadMapDelta(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch, int length)
{
	int n = -1;
	struct TLStruct * d;
	float delta;

	d = &(data[ch]);

	delta = d->delta;

	n = map(100 * log(delta), 0, 80 * log(d->calibratedMaxDelta), 0,
		length);

	n = (n < 0) ? 0 : n;
	n = (n > length) ? length : n;

	return n;
}

int TLSampleMethodTouchRead(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch)
{
	struct TLStruct * d;

	d = &(data[ch]);

	d->sampleMethodPreSample = TLSampleMethodTouchReadPreSample;
	d->sampleMethodSample = TLSampleMethodTouchReadSample;
	d->sampleMethodPostSample = TLSampleMethodTouchReadPostSample;
	d->sampleMethodMapDelta = TLSampleMethodTouchReadMapDelta;

	if (ch < 2) {
		d->tlStructSampleMethod.touchRead.pin = (ch + 0);
	} else if (ch < 7) {
		d->tlStructSampleMethod.touchRead.pin = (ch - 2 + 15);
	} else if (ch < 9) {
		d->tlStructSampleMethod.touchRead.pin = (ch - 7 + 22);
	} else {
		d->tlStructSampleMethod.touchRead.pin = (ch - 9 + 29);
	}

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
