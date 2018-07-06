/*
 * TLSampleMethodResistive.cpp - Resistive sensing implementation for
 * TouchLibrary for Arduino Teensy 3.x
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
#include "TLSampleMethodResistive.h"

#define USE_CORRECT_TRANSFER_FUNCTION			0

#define TL_SAMPLE_METHOD_RESISTIVE_GND_PIN		2
#define TL_SAMPLE_METHOD_RESISTIVE_USE_INTERNAL_PULLUP	true

/* ATmega2560 has internal pull-ups of 20 - 50 kOhm. Assume it is 35 kOhm */
#define TL_REFERENCE_VALUE_DEFAULT			((int32_t) 35000) /* 32 kOhm */
#define TL_SCALE_FACTOR_DEFAULT				((int32_t) 1)
#define TL_VALUE_MAX_DEFAULT				((int32_t) 8000000) /* 8 MOhm */
#define TL_OFFSET_VALUE_DEFAULT				((int32_t) 0) /* Ohm */

#define TL_SET_OFFSET_VALUE_MANUALLY_DEFAULT		false

#if IS_PARTICLE
#define TL_ADC_RESOLUTION_BIT                                   12
#elif IS_ESP32
#define TL_ADC_RESOLUTION_BIT                                   12
#elif IS_ATMEGA
#define TL_ADC_RESOLUTION_BIT                                   10
#else
#define TL_ADC_RESOLUTION_BIT                                   10
#endif

#define TL_ADC_MAX                                              ((1 << TL_ADC_RESOLUTION_BIT) - 1)


#if (USE_CORRECT_TRANSFER_FUNCTION == 1)

#define TL_RELEASED_TO_APPROACHED_THRESHOLD_DEFAULT	\
	(TL_VALUE_MAX_DEFAULT - 4500)
#define TL_APPROACHED_TO_RELEASED_THRESHOLD_DEFAULT	\
	(TL_VALUE_MAX_DEFAULT - 5500)
#define TL_APPROACHED_TO_PRESSED_THRESHOLD_DEFAULT	\
	(TL_VALUE_MAX_DEFAULT - 800)
#define TL_PRESSED_TO_APPROACHED_THRESHOLD_DEFAULT	\
	(TL_VALUE_MAX_DEFAULT - 1000)

#else

#define TL_RELEASED_TO_APPROACHED_THRESHOLD_DEFAULT	0.5
#define TL_APPROACHED_TO_RELEASED_THRESHOLD_DEFAULT	0.25
#define TL_APPROACHED_TO_PRESSED_THRESHOLD_DEFAULT	5
#define TL_PRESSED_TO_APPROACHED_THRESHOLD_DEFAULT	4

#endif

int TLSampleMethodResistivePreSample(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch)
{
	return 0;
}

int32_t TLSampleMethodResistiveSample(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch, bool inv)
{
	struct TLStruct * dCh;
	int ch_pin, gnd_pin;
	int32_t sample;
	bool useInternalPullup;
	
	if (inv) {
		/* Pseudo differential measurements are not supported */
		sample = 0;
	} else {
		dCh = &(data[ch]);
		ch_pin = dCh->tlStructSampleMethod.resistive.pin;
		gnd_pin = dCh->tlStructSampleMethod.resistive.gndPin;
		useInternalPullup =
			dCh->tlStructSampleMethod.resistive.useInternalPullup;

		if (ch_pin < 0) {
			return 0; /* An error occurred */
		}

		if (useInternalPullup) {
			/* Enable internal pull-up on analog input */
			pinMode(ch_pin, INPUT_PULLUP);
		} else {
			/* Disable internal pull-up on analog input */
			pinMode(ch_pin, INPUT);
		}
		
		if (gnd_pin >= 0) {
			/* Configure gnd_pin as digital output, low (gnd) */
			pinMode(gnd_pin, OUTPUT);
			digitalWrite(gnd_pin, LOW);
		}

		/* Read */
		sample = analogRead(ch_pin);

		/* Disable internal pull-up on analog input */
		pinMode(ch_pin, INPUT);
		
		if (gnd_pin >= 0) {
			/* Leave gnd_pin floating */
			pinMode(gnd_pin, INPUT);
			digitalWrite(gnd_pin, LOW);
		}
	}

	return sample;
}

int TLSampleMethodResistivePostSample(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch)
{
	TLStruct * d;
	int32_t tmp, scale;

	d = &(data[ch]);

	switch (d->filterType) {
	case TLStruct::filterTypeAverage:
		scale = (((int32_t) d->nMeasurementsPerSensor) << 1) * 
			(TL_ADC_MAX + 1);
		break;
	case TLStruct::filterTypeSlewrateLimiter:
		scale = ((TL_ADC_MAX + 1) << 2);
		break;
	case TLStruct::filterTypeMedian:
		scale = ((TL_ADC_MAX + 1) << 2);
		break;
	default:
		/* Error! */
		scale = ((TL_ADC_MAX + 1) << 2);
	}

	#if (USE_CORRECT_TRANSFER_FUNCTION == 1)

	/*
	 * Actual transfer function is
	 * (d->raw / scale) / (1 - (d->raw / scale)), but this is very
	 * sensitive to noise when sensor is not pressed (since tmp will then
	 * be very close to 1). Instead, clip the value to a predefined
	 * maximum.
	 */
	if (d->raw > scale) {
		d->raw = scale;
	}
	denum = scale - d->raw;
	tmp = (d->scaleFactor * d->referenceValue * d->raw + (denum >> 1)) / 
		denum;

	if (tmp > d->tlStructSampleMethod.resistive.valueMax) {
		tmp = d->tlStructSampleMethod.resistive.valueMax;
	}

	#else

	tmp = d->scaleFactor * d->referenceValue * d->raw / scale;

	#endif

	d->value = tmp;
	/* Resistance can be negative due to noise! */

	return 0;
}

int32_t TLSampleMethodResistiveMapDelta(struct TLStruct * data, 
		uint8_t nSensors, uint8_t ch, int length)
{
	int32_t n = -1;
	struct TLStruct * d;
	int32_t delta;

	d = &(data[ch]);
	delta = d->delta - d->releasedToApproachedThreshold / 2;

	n = map(100 * delta, 0, 100 * d->calibratedMaxDelta, 0, length);

	n = (n < 0) ? 0 : n;
	n = (n > length) ? length : n;

	return n;
}

int TLSampleMethodResistive(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch)
{
	struct TLStruct * d;

	d = &(data[ch]);

	d->sampleMethodPreSample = TLSampleMethodResistivePreSample;
	d->sampleMethodSample = TLSampleMethodResistiveSample;
	d->sampleMethodPostSample = TLSampleMethodResistivePostSample;
	d->sampleMethodMapDelta = TLSampleMethodResistiveMapDelta;

	d->tlStructSampleMethod.resistive.pin = A0 + ch;
	d->tlStructSampleMethod.resistive.gndPin = ch +
		TL_SAMPLE_METHOD_RESISTIVE_GND_PIN;
	d->tlStructSampleMethod.resistive.useInternalPullup =
		TL_SAMPLE_METHOD_RESISTIVE_USE_INTERNAL_PULLUP;

	d->referenceValue = TL_REFERENCE_VALUE_DEFAULT;
	d->offsetValue = TL_OFFSET_VALUE_DEFAULT;
	d->scaleFactor = TL_SCALE_FACTOR_DEFAULT;
	d->setOffsetValueManually = TL_SET_OFFSET_VALUE_MANUALLY_DEFAULT;
	d->tlStructSampleMethod.resistive.valueMax = TL_VALUE_MAX_DEFAULT;

	d->releasedToApproachedThreshold = 
		TL_RELEASED_TO_APPROACHED_THRESHOLD_DEFAULT;
	d->approachedToReleasedThreshold =
		TL_APPROACHED_TO_RELEASED_THRESHOLD_DEFAULT;
	d->approachedToPressedThreshold =
		TL_APPROACHED_TO_PRESSED_THRESHOLD_DEFAULT;     
	d->pressedToApproachedThreshold =
		TL_PRESSED_TO_APPROACHED_THRESHOLD_DEFAULT; 

	d->direction = TLStruct::directionNegative;
	d->sampleType = TLStruct::sampleTypeNormal;
	d->filterType = TLStruct::filterTypeAverage;
	d->waterRejectPin = -1;
	d->waterRejectMode = TLStruct::waterRejectModeFloat;

	d->pin = &(d->tlStructSampleMethod.resistive.pin);

	return 0;
}
