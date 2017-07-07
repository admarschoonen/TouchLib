/*
 * TLSampleMethodCVD.cpp - Capacitive sensing implementation using CVD method
 * for TouchLibrary for Arduino
 * 
 * https://github.com/AdmarSchoonen/CVDSensor
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

#include "CVDSensor.h"
#include "TLSampleMethodCVD.h"

static uint8_t TLChannelToReference(struct CvdStruct * data, uint8_t nSensors,
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
	bool result = false;

	/* 
	 * From
	 * http://electronics.stackexchange.com/questions/31048/can-an-atmega-
	 * or-attiny-device-signature-be-read-while-running
	 * and http://www.avrfreaks.net/forum/device-signatures.
	 */
	if ((SIGNATURE_0 == 0x1E) && (SIGNATURE_1 == 0x97)) {
		/* ATmega128x */
		result = true;
	}
	if ((SIGNATURE_0 == 0x1E) && (SIGNATURE_1 == 0x98)) {
		/* ATmega256x */
		result = true;
	}

	return result;
}

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

static void TLChargeADC(struct CvdStruct * data, uint8_t nSensors, uint8_t ch, 
		int ref_pin, bool delay)
{
	/* Set ADC to reference pin (charge Chold). */
	TLSetAdcReferencePin(ref_pin);

	if ((delay) && (data[ch].chargeDelayADC)) {
		delayMicroseconds(data[ch].chargeDelayADC);
	}
}

static void TLChargeSensor(struct CvdStruct * data, uint8_t nSensors, uint8_t ch,
		int ch_pin, bool delay)
{
	/*
	 * Set ADC to sensor pin (transfer charge from Chold to Csense).
	 */
	TLSetAdcReferencePin(ch_pin);

	if ((delay) && (data[ch].chargeDelaySensor)) {
		delayMicroseconds(data[ch].chargeDelaySensor);
	}
}

static void TLCharge(struct CvdStruct * data, uint8_t nSensors, uint8_t ch,
		int ch_pin, int ref_pin)
{
	unsigned int d;

	TLChargeADC(data, nSensors, ch, ref_pin, false);
	TLChargeSensor(data, nSensors, ch, ch_pin, false);

	d = (data[ch].chargeDelayADC > data[ch].chargeDelaySensor) ?
		data[ch].chargeDelayADC : data[ch].chargeDelaySensor;

	if (d) {
		delayMicroseconds(d);
	}
}

int TLSampleMethodCVD(struct CvdStruct * data, uint8_t nSensors, uint8_t ch,
		bool inv)
{
	struct CvdStruct * dCh;
	struct CvdStruct * dRef;
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
	ch_pin = dCh->pin;
	ref_pin = dRef->pin;

	TLSetSensorAndReferencePins(ch_pin, ref_pin, inv);

	/* Set sensor pin as analog input. */
	pinMode(ch_pin, INPUT);

	/*
	 * Charge nCharges - 1 times to account for the charge during the
	 * analogRead() below.
	 */
	for (i = 0; i < dCh->nCharges - 1; i++) {
		TLCharge(data, nSensors, ch, ch_pin, ref_pin);
	}

	/* Set ADC to reference pin (charge internal capacitor). */
	TLChargeADC(data, nSensors, ch, ref_pin, true);

	/* Read sensor. */
	sample = analogRead(ch_pin - A0);

	if (dCh->useNChargesPadding) {
		/*
		 * Increment i before starting the loop to account for the
		 * charge during the analogRead() above.
		 */
		for (++i; i < dCh->nChargesMax; i++) {
			TLCharge(data, nSensors, ch, ch_pin, ref_pin);
		}
	}

	/* Discharge sensor. */
	pinMode(ch_pin, OUTPUT);
	digitalWrite(ch_pin, LOW);

	return sample;
}
