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
#include "TLSampleMethodCVD.h"
#include "BoardID.h"

#if IS_ATMEGA
#include "TLSampleMethodCVDATMega.h"
#elif IS_TEENSY3X
#include "TLSampleMethodCVDTeensy3x.h"
#elif IS_PARTICLE
#include "TLSampleMethodCVDParticle.h"
#elif IS_ESP32
#include "TLSampleMethodCVDEsp32.h"
#endif

#ifndef TL_METHOD_CVD_SUPPORTED
#include "TLSampleMethodCVDUnsupported.h"
#endif

#define TL_USE_N_CHARGES_PADDING_DEFAULT		true

#define TL_REFERENCE_VALUE_DEFAULT			((int32_t) 15000) /* 15 pF */
#define TL_SCALE_FACTOR_DEFAULT				((int32_t) 1)
#define TL_OFFSET_VALUE_DEFAULT				((int32_t) 1000000) /* fF */

#define TL_SET_OFFSET_VALUE_MANUALLY_DEFAULT		false

#define TL_RELEASED_TO_APPROACHED_THRESHOLD_DEFAULT	5.0
#define TL_APPROACHED_TO_RELEASED_THRESHOLD_DEFAULT	4.0
#define TL_APPROACHED_TO_PRESSED_THRESHOLD_DEFAULT	10.0
#define TL_PRESSED_TO_APPROACHED_THRESHOLD_DEFAULT	80.0

static uint8_t TLChannelToReference(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch)
{
	uint8_t ref;

	ref = ch;
	do {
		if (ref == nSensors - 1) {
			ref = 0;
		} else {
			ref = ref + 1;
		}
		
		if (ref == ch) {
			/*
			 * Error! Could not find another pin that uses
			 * TLSampleMethodCVD.
			 */
			ref = 0xFF;
		}
	} while ((ref != 0xFF) && (data[ref].sampleMethod !=
		TLSampleMethodCVD));

	return ref;
}

static void TLSetSensorAndReferencePins(int ch_pin, int ref_pin, bool inv)
{
	/* Set reference pin as output and high. */

	pinMode(ref_pin, OUTPUT);
	if (inv) {
		digitalWrite(ref_pin, LOW);
	} else {
		digitalWrite(ref_pin, HIGH);
	}

	/* Set sensor pin as output and low (discharge sensor). */
	pinMode(ch_pin, OUTPUT);
	if (inv) {
		digitalWrite(ch_pin, HIGH);
	} else {
		digitalWrite(ch_pin, LOW);
	}
}

static void TLChargeADC(struct TLStruct * data, uint8_t nSensors, uint8_t ch, 
		int ref_pin, bool delay)
{
	/* Set ADC to reference pin (charge Chold). */
	TLSetAdcReferencePin(ref_pin);

	if ((delay) && (data[ch].tlStructSampleMethod.CVD.chargeDelayADC)) {
		delayMicroseconds(data[ch].tlStructSampleMethod.CVD.chargeDelayADC);
	}
}

static void TLChargeSensor(struct TLStruct * data, uint8_t nSensors, uint8_t ch,
		int ch_pin, bool delay)
{
	/*
	 * Set ADC to sensor pin (transfer charge from Chold to Csense).
	 */
	TLSetAdcReferencePin(ch_pin);

	if ((delay) && (data[ch].tlStructSampleMethod.CVD.chargeDelaySensor)) {
		delayMicroseconds(data[ch].tlStructSampleMethod.CVD.chargeDelaySensor);
	}
}

static void TLDischargeSensor(struct TLStruct * data, uint8_t nSensors, uint8_t ch,
		bool delay)
{
	pinMode(data[ch].tlStructSampleMethod.CVD.pin, OUTPUT);
	digitalWrite(data[ch].tlStructSampleMethod.CVD.pin, LOW);

	if ((delay) && (data[ch].tlStructSampleMethod.CVD.chargeDelaySensor)) {
		delayMicroseconds(data[ch].tlStructSampleMethod.CVD.chargeDelaySensor);
	}
}

static void TLCharge(struct TLStruct * data, uint8_t nSensors, uint8_t ch,
		int ch_pin, int ref_pin)
{
	unsigned int d;

	TLChargeADC(data, nSensors, ch, ref_pin, false);
	TLChargeSensor(data, nSensors, ch, ch_pin, false);

	d = (data[ch].tlStructSampleMethod.CVD.chargeDelayADC >
			data[ch].tlStructSampleMethod.CVD.chargeDelaySensor) ?
		data[ch].tlStructSampleMethod.CVD.chargeDelayADC :
			data[ch].tlStructSampleMethod.CVD.chargeDelaySensor;

	if (d) {
		delayMicroseconds(d);
	}
}

static void correctSample(struct TLStruct * data, uint8_t nSensors, uint8_t ch)
{
	TLStruct * d;
	int32_t tmp, scale;
	float tmp_f, scale_f;

	d = &(data[ch]);

	if (d->tlStructSampleMethod.CVD.nCharges > 1) {
		/* floating point math */
		switch (d->filterType) {
		case TLStruct::filterTypeAverage:
			scale_f = ((float) (d->nMeasurementsPerSensor << 1)) *
				((float) (TL_ADC_MAX + 1));
			break;
		case TLStruct::filterTypeSlewrateLimiter:
			scale_f = (float) ((TL_ADC_MAX + 1) << 2);
			break;
		case TLStruct::filterTypeMedian:
			scale_f = (float) ((TL_ADC_MAX + 1) << 2);
			break;
		default:
			/* Error! */
			scale_f = (float) ((TL_ADC_MAX + 1) << 2);
		}
	
		tmp = 1 - ((float) d->raw) / scale_f;

		tmp_f = pow((float) tmp, -((float) 1) /
			((float) d->tlStructSampleMethod.CVD.nCharges)) - 
			((float) 1);
		tmp_f = (((float) 1) / tmp_f);
		tmp = (int32_t) tmp_f;

		d->tlStructSampleMethod.CVD.nChargesNext = (int32_t)
			(ceilf(tmp));
	
		tmp = (int32_t) (scale_f * tmp * d->scaleFactor / 
				d->referenceValue);
	} else {
		switch (d->filterType) {
		case TLStruct::filterTypeAverage:
			scale = (((int32_t) d->nMeasurementsPerSensor) << 1) *
				(((int32_t) TL_ADC_MAX) + 1);
			break;
		case TLStruct::filterTypeSlewrateLimiter:
			scale = (((int32_t) TL_ADC_MAX) + 1) << 2;
			break;
		case TLStruct::filterTypeMedian:
			scale = (((int32_t) TL_ADC_MAX) + 1) << 2;
			break;
		default:
			/* Error! */
			scale = (((int32_t) TL_ADC_MAX) + 1) << 2;
		}
		tmp = ((d->referenceValue * d->scaleFactor * (scale - d->raw))
				+ (scale >> 1)) / scale;
	}
	d->value = (int32_t) tmp;

	if (d->tlStructSampleMethod.CVD.nChargesNext < 
			TL_N_CHARGES_MIN_DEFAULT) {
		d->tlStructSampleMethod.CVD.nChargesNext =
			TL_N_CHARGES_MIN_DEFAULT;
	}
	if (d->tlStructSampleMethod.CVD.nChargesNext > 
			TL_N_CHARGES_MAX_DEFAULT) {
		d->tlStructSampleMethod.CVD.nChargesNext =
			TL_N_CHARGES_MAX_DEFAULT;
	}
	/* Capacitance can be negative due to noise! */
}

static void updateNCharges(struct TLStruct * data, uint8_t nSensors, uint8_t ch)
{
	TLStruct * d;

	d = &(data[ch]);
	d->tlStructSampleMethod.CVD.nCharges = d->tlStructSampleMethod.CVD.nChargesNext;
}


int TLSampleMethodCVDPreSample(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch)
{
	return 0;
}

int32_t TLSampleMethodCVDSample(struct TLStruct * data, uint8_t nSensors, 
		uint8_t ch, bool inv)
{
	struct TLStruct * dCh;
	struct TLStruct * dRef;
	uint8_t ref;
	int ch_pin, ref_pin;
	int32_t sample;
	uint8_t i;

	ref = TLChannelToReference(data, nSensors, ch);
	if (ref == 0xFF) {
		/* An error occurred! */
		return 0;
	}

	dCh = &(data[ch]);
	dRef = &(data[ref]);
	ch_pin = dCh->tlStructSampleMethod.CVD.pin;
	ref_pin = dRef->tlStructSampleMethod.CVD.pin;

	if (ch_pin < 0) {
		/* An error occurred! */
		return 0;
	}

	if (ref_pin < 0) {
		/* An error occurred! */
		return 0;
	}

	TLSetSensorAndReferencePins(ch_pin, ref_pin, inv);

	/* Set sensor pin as analog input. */
	pinMode(ch_pin, INPUT);

	/*
	 * Charge nCharges - 1 times to account for the charge during the
	 * TLAnalogRead() below.
	 */
	for (i = 0; i < dCh->tlStructSampleMethod.CVD.nCharges - 1; i++) {
		TLCharge(data, nSensors, ch, ch_pin, ref_pin);
	}

	/* Set ADC to reference pin (charge internal capacitor). */
	TLChargeADC(data, nSensors, ch, ref_pin, true);

	/* Read sensor. */
	sample = TLAnalogRead(ch_pin);

	if (inv) {
		sample = TL_ADC_MAX - sample;
	}

	if (dCh->tlStructSampleMethod.CVD.useNChargesPadding) {
		/*
		 * Increment i before starting the loop to account for the
		 * charge during the TLAnalogRead() above.
		 */
		for (++i; i < dCh->tlStructSampleMethod.CVD.nChargesMax; i++) {
			TLCharge(data, nSensors, ch, ch_pin, ref_pin);
		}
	}

	TLDischargeSensor(data, nSensors, ch, true);

	return sample;
}

int TLSampleMethodCVDPostSample(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch)
{
	correctSample(data, nSensors, ch);
	updateNCharges(data, nSensors, ch);

	return 0;
}

int32_t TLSampleMethodCVDMapDelta(struct TLStruct * data, uint8_t nSensors,
                uint8_t ch, int length)
{
	int32_t n = -1;
	struct TLStruct * d;
	int32_t delta;

	d = &(data[ch]);

	delta = d->delta;

	/*
	 * Ignore everything below TL_BAR_LOWER_PCT of log(maxDelta); it's
	 * mostly noise
	 */
	n = map(100 * log(delta), TL_BAR_LOWER_PCT * 
		log(d->calibratedMaxDelta),
		TL_BAR_UPPER_PCT * log(d->calibratedMaxDelta), (float) 0,
		(float) length);

	n = (n < 0) ? 0 : n;
	n = (n > length) ? length : n;

	return n;
}

int TLSampleMethodCVD(struct TLStruct * data, uint8_t nSensors, uint8_t ch)
{
	struct TLStruct * d;

	#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
	/* Perform analogRead() to ensure ADC has finished calibration */
	analogRead(A0);
	#endif

	d = &(data[ch]);

	d->sampleMethodPreSample = TLSampleMethodCVDPreSample;
	d->sampleMethodSample = TLSampleMethodCVDSample;
	d->sampleMethodPostSample = TLSampleMethodCVDPostSample;
	d->sampleMethodMapDelta = TLSampleMethodCVDMapDelta;

	d->tlStructSampleMethod.CVD.pin = A0 + ch;
	d->tlStructSampleMethod.CVD.useNChargesPadding =
		TL_USE_N_CHARGES_PADDING_DEFAULT;
	d->tlStructSampleMethod.CVD.nChargesMin = TL_N_CHARGES_MIN_DEFAULT;
	d->tlStructSampleMethod.CVD.nChargesMax = TL_N_CHARGES_MAX_DEFAULT;
	d->tlStructSampleMethod.CVD.nCharges = TL_N_CHARGES_MIN_DEFAULT;
	d->tlStructSampleMethod.CVD.nChargesNext = TL_N_CHARGES_MIN_DEFAULT;

	d->tlStructSampleMethod.CVD.chargeDelaySensor =
		TL_CHARGE_DELAY_SENSOR_DEFAULT;

	d->tlStructSampleMethod.CVD.chargeDelayADC = TL_CHARGE_DELAY_ADC_DEFAULT;

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
	d->sampleType = TLStruct::sampleTypeDifferential;
	d->filterType = TLStruct::filterTypeAverage;
	d->waterRejectPin = -1;
	d->waterRejectMode = TLStruct::waterRejectModeFloat;

	d->pin = &(d->tlStructSampleMethod.CVD.pin);

	return 0;
}

