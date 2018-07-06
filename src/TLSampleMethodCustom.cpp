/*
 * TLSampleMethodCustom.cpp - Custom sensing implementation for
 * TouchLibrary for Arduino
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
#include "TLSampleMethodCustom.h"

#define TL_RELEASED_TO_APPROACHED_THRESHOLD_DEFAULT	50.0
#define TL_APPROACHED_TO_RELEASED_THRESHOLD_DEFAULT	40.0
#define TL_APPROACHED_TO_PRESSED_THRESHOLD_DEFAULT	150.0
#define TL_PRESSED_TO_APPROACHED_THRESHOLD_DEFAULT	120.0

int TLSampleMethodCustomPreSample(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch)
{
	return 0;
}

int32_t TLSampleMethodCustomSample(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch, bool inv)
{
	return 0;
}

int TLSampleMethodCustomPostSample(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch)
{
	struct TLStruct * d;

	d = &(data[ch]);

	d->value = d->raw;

	return 0;
}

int32_t TLSampleMethodCustomMapDelta(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch, int length)
{
	return 0;
}

int TLSampleMethodCustom(struct TLStruct * data, uint8_t nSensors, uint8_t ch)
{
	struct TLStruct * d;

	d = &(data[ch]);

	d->sampleMethodPreSample = TLSampleMethodCustomPreSample;
	d->sampleMethodSample = TLSampleMethodCustomSample;
	d->sampleMethodPostSample = TLSampleMethodCustomPostSample;
	d->sampleMethodMapDelta = TLSampleMethodCustomMapDelta;

	d->tlStructSampleMethod.custom.pin = A0 + ch;

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
	d->waterRejectPin = -1;
	d->waterRejectMode = TLStruct::waterRejectModeFloat;

	d->pin = &(d->tlStructSampleMethod.custom.pin);

	return 0;
}
