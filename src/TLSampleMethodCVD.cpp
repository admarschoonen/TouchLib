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

#include "TouchLib.h"
#include "TLSampleMethodCVD.h"
#include "BoardID.h"

#define TL_N_CHARGES_MIN_DEFAULT			1
#define TL_N_CHARGES_MAX_DEFAULT			1

#define TL_USE_N_CHARGES_PADDING_DEFAULT		true

#define TL_CHARGE_DELAY_SENSOR_DEFAULT			0
#define TL_CHARGE_DELAY_ADC_DEFAULT			0

#define TL_REFERENCE_VALUE_DEFAULT			((float) 15) /* 15 pF */
#define TL_SCALE_FACTOR_DEFAULT				((float) 1)
#define TL_OFFSET_VALUE_DEFAULT				((float) 1000) /* pF */

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

bool TLHasMux5(void)
{
	#if IS_ATMEGA128X_256X
	return true;
	#else
	return false;
	#endif
}

#if IS_AVR
void TLSetAdcReferencePin(int pin)
{
	unsigned char mux;

	if (TLHasMux5()) {
		if (pin - A0 < 8) {
			mux = pin - A0;
		} else {
			mux = 0x20 + (pin - A0 - 8);
		}

		ADMUX &= ~0x1F;
		ADMUX |= (mux & 0x1F);
		if (mux > 0x1F) {
			ADCSRB |= 0x08;
		} else {
			ADCSRB &= ~0x08;
		}
	} else {
		mux = pin - A0;
		ADMUX &= ~0x0F;
		ADMUX |= (mux & 0x0F);
	}
}
#elif IS_TEENSY3X
void TLSetAdcReferencePin(int pin)
{
	volatile uint32_t * ADCx_CFG2 = &ADC0_CFG2;
	volatile uint32_t * ADCx_SC1A = &ADC0_SC1A;

	#if IS_TEENSY32_WITH_ADC1
	if ((pin - A0) & 0x80) {
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
	*ADCx_SC1A = (pin - A0) & 0x1F;
}
#else
#warning CVD sensors has not yet been ported to this architecture. Please \
	inform the author.
void TLSetAdcReferencePin(int pin)
{
}
#endif

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
	float tmp, scale;

	d = &(data[ch]);

	if (d->enableSlewrateLimiter) {
		scale = (float) ((TL_ADC_MAX + 1) << 2);
	} else {
		scale = ((float) (d->nMeasurementsPerSensor << 1)) *
			((float) (TL_ADC_MAX + 1));
	}

	tmp = 1 - ((float) d->raw) / scale;
	tmp = pow(tmp, -((float) 1) /
		((float) d->tlStructSampleMethod.CVD.nCharges)) - ((float) 1);
	tmp = ((float) 1) / tmp;

	d->tlStructSampleMethod.CVD.nChargesNext = (int32_t) (ceilf(tmp));
	if (d->tlStructSampleMethod.CVD.nChargesNext < TL_N_CHARGES_MIN_DEFAULT) {
		d->tlStructSampleMethod.CVD.nChargesNext =
			TL_N_CHARGES_MIN_DEFAULT;
	}
	if (d->tlStructSampleMethod.CVD.nChargesNext > TL_N_CHARGES_MAX_DEFAULT) {
		d->tlStructSampleMethod.CVD.nChargesNext =
			TL_N_CHARGES_MAX_DEFAULT;
	}

	tmp = scale * tmp * d->scaleFactor / d->referenceValue;
	d->value = tmp;
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

int TLSampleMethodCVDSample(struct TLStruct * data, uint8_t nSensors, 
		uint8_t ch, bool inv)
{
	struct TLStruct * dCh;
	struct TLStruct * dRef;
	uint8_t ref;
	int ch_pin, ref_pin, sample;
	uint8_t i;

	ref = TLChannelToReference(data, nSensors, ch);
	if (ref == 0xFF) {
		/* An error occurred! */
		return 0;
	}

	dCh = &(data[ch]);
	dRef = &(data[ref]);
	ch_pin = dCh->tlStructSampleMethod.CVD.pin;

	if (ch_pin < 0) {
		/* An error occurred! */
		return 0;
	}

	ref_pin = dRef->tlStructSampleMethod.CVD.pin;

	TLSetSensorAndReferencePins(ch_pin, ref_pin, inv);

	/* Set sensor pin as analog input. */
	pinMode(ch_pin, INPUT);

	/*
	 * Charge nCharges - 1 times to account for the charge during the
	 * analogRead() below.
	 */
	for (i = 0; i < dCh->tlStructSampleMethod.CVD.nCharges - 1; i++) {
		TLCharge(data, nSensors, ch, ch_pin, ref_pin);
	}

	/* Set ADC to reference pin (charge internal capacitor). */
	TLChargeADC(data, nSensors, ch, ref_pin, true);

	/* Read sensor. */
	sample = analogRead(ch_pin - A0);

	if (dCh->tlStructSampleMethod.CVD.useNChargesPadding) {
		/*
		 * Increment i before starting the loop to account for the
		 * charge during the analogRead() above.
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

int TLSampleMethodCVDMapDelta(struct TLStruct * data, uint8_t nSensors,
                uint8_t ch, int length)
{
	int n = -1;
	struct TLStruct * d;
	float delta;

	d = &(data[ch]);

	delta = d->delta;

	/* Ignore everything below 40% of log(maxDelta); it's mostly noise */
	n = map(100 * log(delta),  40 * log(d->calibratedMaxDelta), 
		80 * log(d->calibratedMaxDelta), 0,
		length);

	n = (n < 0) ? 0 : n;
	n = (n > length) ? length : n;

	return n;
}

int TLSampleMethodCVD(struct TLStruct * data, uint8_t nSensors, uint8_t ch)
{
	struct TLStruct * d;

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

	d->pin = &(d->tlStructSampleMethod.CVD.pin);

	return 0;
}

