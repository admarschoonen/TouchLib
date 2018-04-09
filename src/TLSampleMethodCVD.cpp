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

#if IS_PARTICLE
#include "pinmap_hal.h"
#include "stm32f2xx.h"
#include "stm32f2xx_rcc.h"
#endif

#define TL_N_CHARGES_MIN_DEFAULT			1
#define TL_N_CHARGES_MAX_DEFAULT			1

#define TL_USE_N_CHARGES_PADDING_DEFAULT		true

#define TL_CHARGE_DELAY_SENSOR_DEFAULT			0
#define TL_CHARGE_DELAY_ADC_DEFAULT			0

#define TL_REFERENCE_VALUE_DEFAULT			((int32_t) 15000) /* 15 pF */
#define TL_SCALE_FACTOR_DEFAULT				((int32_t) 1)
#define TL_OFFSET_VALUE_DEFAULT				((int32_t) 1000000) /* fF */

#define TL_SET_OFFSET_VALUE_MANUALLY_DEFAULT		false

#define TL_RELEASED_TO_APPROACHED_THRESHOLD_DEFAULT	5.0
#define TL_APPROACHED_TO_RELEASED_THRESHOLD_DEFAULT	4.0
#define TL_APPROACHED_TO_PRESSED_THRESHOLD_DEFAULT	10.0
#define TL_PRESSED_TO_APPROACHED_THRESHOLD_DEFAULT	80.0

#if IS_PARTICLE
static const unsigned char pin_to_adc_channel[8] = {15, 13, 12, 5, 6, 7, 4, 0};
#endif

#if IS_PARTICLE
#define TL_ADC_RESOLUTION_BIT					12
#define TL_BAR_LOWER_PCT					50
#define TL_BAR_UPPER_PCT					95
#elif IS_ATMEGA
#define TL_ADC_RESOLUTION_BIT					10
#define TL_BAR_LOWER_PCT					40
#define TL_BAR_UPPER_PCT					80
#else
#define TL_ADC_RESOLUTION_BIT					10
#define TL_BAR_LOWER_PCT					40
#define TL_BAR_UPPER_PCT					80
#endif

#define TL_ADC_MAX                                              ((1 << TL_ADC_RESOLUTION_BIT) - 1)

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

int TLAnalogRead(int pin)
{
	return analogRead(pin - A0);
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

int TLAnalogRead(int pin)
{
	return analogRead(pin - A0);
}
#elif IS_PARTICLE
void TLSetAdcReferencePin(int pin)
{
	long unsigned int reg;
	long unsigned int *p;

	p = (long unsigned int *) ADC1_BASE + offsetof(ADC_TypeDef, CR1);
	reg = *p;
	reg &= ~(0x1F);
	//reg |= pin_to_adc_channel[pin - A0];
	reg |= PIN_MAP[pin].adc_channel;
	*p = reg;
}

int ADC_Init(void)
{
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2,
		ENABLE);
	ADC_DeInit();

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = 0;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;

	ADC_Init(ADC1, &ADC_InitStructure);

	//ADC_Init(ADC2, &ADC_InitStructure);

	ADC_Cmd(ADC1, ENABLE);

	return 0;
}

int ADC_ReInit(void)
{
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2,
		ENABLE);
	ADC_DeInit();

	/* ADC Common Init */
	ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_RegSimult;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	// ADC1 configuration
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	// ADC2 configuration
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC2, &ADC_InitStructure);

	return 0;
}

int TLAnalogRead(int pin)
{
	/*
	 * Cannot use analogRead() here since Photons analogRead() reads 10 ADC
	 * samples at once. This influences the capacitive signal too much.
	 */
	uint8_t adc_ch;
	uint8_t ADC_Sample_Time = ADC_SampleTime_3Cycles;
	uint8_t ADC_Sample_Time_Default = ADC_SampleTime_480Cycles;
	int value;

	adc_ch = PIN_MAP[pin].adc_channel;

	ADC_Init();

	HAL_Pin_Mode(pin, AN_INPUT);
	ADC_RegularChannelConfig(ADC1, adc_ch, 1, ADC_Sample_Time);
	// ADC_RegularChannelConfig(ADC2, adc_ch, 1, ADC_Sample_Time);

	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

	value = ADC_GetConversionValue(ADC1);

	ADC_ReInit();

	ADC_RegularChannelConfig(ADC1, adc_ch, 1, ADC_Sample_Time_Default);
	ADC_RegularChannelConfig(ADC2, adc_ch, 1, ADC_Sample_Time_Default);

	return value;
}
#elif IS_ESP32
/* ESP32 does not yet have CVD support */
void TLSetAdcReferencePin(int pin)
{
}

int TLAnalogRead(int pin)
{
	return 0;
}
#else
#warning CVD sensors has not yet been ported to this architecture. Please inform the author.
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
	int32_t tmp, scale;
	float tmp_f, scale_f;

	d = &(data[ch]);

	if (d->tlStructSampleMethod.CVD.nCharges > 1) {
		/* floating point math */
		if (d->enableSlewrateLimiter) {
			scale_f = (float) ((TL_ADC_MAX + 1) << 2);
		} else {
			scale_f = ((float) (d->nMeasurementsPerSensor << 1)) *
				((float) (TL_ADC_MAX + 1));
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
		if (d->enableSlewrateLimiter) {
			scale = (((int32_t) TL_ADC_MAX) + 1) << 2;
		} else {
			scale = (((int32_t) d->nMeasurementsPerSensor) << 1) *
				(((int32_t) TL_ADC_MAX) + 1);
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

