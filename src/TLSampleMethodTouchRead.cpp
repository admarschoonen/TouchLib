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

#if (IS_ESP32)
/*
 * Arduino port for ESP32 configures touch peripheral for very large
 * capacitances and provides no means of reconfiguration so we'll have to
 * implement our own configuration.
 */
#include <soc/sens_reg.h>
#include <soc/rtc_cntl_reg.h>
#endif

/* 
 * Teensy has 1/50 pF (== 20 fF) per lsb --> set default reference value to 20
 * https://www.kickstarter.com/projects/paulstoffregen/teensy-30-32-bit-arm-cortex-m4-usable-in-arduino-a/posts
 */
#define TL_REFERENCE_VALUE_DEFAULT			((int32_t) 20)

#if IS_ESP32
#define TL_SCALE_FACTOR_DEFAULT				(16)
#else
#define TL_SCALE_FACTOR_DEFAULT				((int32_t) 1)
#endif

#define TL_OFFSET_VALUE_DEFAULT				((int32_t) 0) /* fF */

#define TL_SET_OFFSET_VALUE_MANUALLY_DEFAULT	    	false

#define TL_RELEASED_TO_APPROACHED_THRESHOLD_DEFAULT	2.5
#define TL_APPROACHED_TO_RELEASED_THRESHOLD_DEFAULT	2.0
#define TL_APPROACHED_TO_PRESSED_THRESHOLD_DEFAULT	7.5
#define TL_PRESSED_TO_APPROACHED_THRESHOLD_DEFAULT	6.0
#define TL_TOUCHREAD_MAX				((1 << 16) - 1)

#define TL_BAR_LOWER_PCT				40
#define TL_BAR_UPPER_PCT				80

#if (IS_ESP32)
static uint16_t __touchSleepCycles = 0x1000;

/*
 * Increase __touchMeasureCycles to get more counts per measurement (but of
 * course this will increase the measurement time as well).
 * The value is the number of counts of an 8 MHz clock, so 0x2000 is 1.024 ms.
 * */
static uint16_t __touchMeasureCycles = 0x2000;

typedef void (*voidFuncPtr)(void);
static voidFuncPtr __touchInterruptHandlers[10] = {0,};
static intr_handle_t touch_intr_handle = NULL;

void IRAM_ATTR __touchISR(void * arg)
{
	uint32_t pad_intr = READ_PERI_REG(SENS_SAR_TOUCH_CTRL2_REG) & 0x3ff;
	uint32_t rtc_intr = READ_PERI_REG(RTC_CNTL_INT_ST_REG);
	uint8_t i = 0;
	//clear interrupt
	WRITE_PERI_REG(RTC_CNTL_INT_CLR_REG, rtc_intr);
	SET_PERI_REG_MASK(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_MEAS_EN_CLR);

	if (rtc_intr & RTC_CNTL_TOUCH_INT_ST) {
		for (i = 0; i < 10; ++i) {
			if ((pad_intr >> i) & 0x01) {
				if(__touchInterruptHandlers[i]){
					__touchInterruptHandlers[i]();
				}
			}
		}
	}
}

void __touchSetCycles(uint16_t measure, uint16_t sleep)
{
	__touchSleepCycles = sleep;
	__touchMeasureCycles = measure;
	//Touch pad SleepCycle Time
	SET_PERI_REG_BITS(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_SLEEP_CYCLES, __touchSleepCycles, SENS_TOUCH_SLEEP_CYCLES_S);
	//Touch Pad Measure Time
	SET_PERI_REG_BITS(SENS_SAR_TOUCH_CTRL1_REG, SENS_TOUCH_MEAS_DELAY, __touchMeasureCycles, SENS_TOUCH_MEAS_DELAY_S);
}

void __touchInit()
{
	SET_PERI_REG_BITS(RTC_IO_TOUCH_CFG_REG, RTC_IO_TOUCH_XPD_BIAS, 1, RTC_IO_TOUCH_XPD_BIAS_S);

	/* Configure for lowest upper level and highest lower level. This
	 * results in a higher oscillation frequency and therefore more counts
	 * per measurement.
	 */
	SET_PERI_REG_BITS(RTC_IO_TOUCH_CFG_REG, RTC_IO_TOUCH_DREFH, 0, RTC_IO_TOUCH_DREFH_S);
	SET_PERI_REG_BITS(RTC_IO_TOUCH_CFG_REG, RTC_IO_TOUCH_DREFL, 3, RTC_IO_TOUCH_DREFL_S);
	SET_PERI_REG_MASK(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_MEAS_EN_CLR);
	//clear touch enable
	WRITE_PERI_REG(SENS_SAR_TOUCH_ENABLE_REG, 0x0);
	SET_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_TOUCH_SLP_TIMER_EN);

	__touchSetCycles(__touchMeasureCycles, __touchSleepCycles);

	esp_intr_alloc(ETS_RTC_CORE_INTR_SOURCE, (int)ESP_INTR_FLAG_IRAM, __touchISR, NULL, &touch_intr_handle);
}

uint16_t esp32TouchRead(uint8_t pin)
{
	int8_t pad = digitalPinToTouchChannel(pin);

	if(pad < 0){
		return 0;
	}

	pinMode(pin, ANALOG);

	__touchInit();

	uint32_t v0 = READ_PERI_REG(SENS_SAR_TOUCH_ENABLE_REG);
	//Disable Intr & enable touch pad
	WRITE_PERI_REG(SENS_SAR_TOUCH_ENABLE_REG,
			(v0 & ~((1 << (pad + SENS_TOUCH_PAD_OUTEN2_S)) | (1 << (pad + SENS_TOUCH_PAD_OUTEN1_S))))
			| (1 << (pad + SENS_TOUCH_PAD_WORKEN_S)));

	SET_PERI_REG_MASK(SENS_SAR_TOUCH_ENABLE_REG, (1 << (pad + SENS_TOUCH_PAD_WORKEN_S)));

	uint32_t rtc_tio_reg = RTC_IO_TOUCH_PAD0_REG + pad * 4;
	WRITE_PERI_REG(rtc_tio_reg, (READ_PERI_REG(rtc_tio_reg)
					  & ~(RTC_IO_TOUCH_PAD0_DAC_M))
					  | (7 << RTC_IO_TOUCH_PAD0_DAC_S)//Touch Set Slope
					  | RTC_IO_TOUCH_PAD0_TIE_OPT_M   //Enable Tie,Init Level
					  | RTC_IO_TOUCH_PAD0_START_M	 //Enable Touch Pad IO
					  | RTC_IO_TOUCH_PAD0_XPD_M);	 //Enable Touch Pad Power on

	//force oneTime test start
	SET_PERI_REG_MASK(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_START_EN_M|SENS_TOUCH_START_FORCE_M);

	SET_PERI_REG_BITS(SENS_SAR_TOUCH_CTRL1_REG, SENS_TOUCH_XPD_WAIT, 10, SENS_TOUCH_XPD_WAIT_S);

	while (GET_PERI_REG_MASK(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_MEAS_DONE) == 0) {};

	uint16_t touch_value = READ_PERI_REG(SENS_SAR_TOUCH_OUT1_REG + (pad / 2) * 4) >> ((pad & 1) ? SENS_TOUCH_MEAS_OUT1_S : SENS_TOUCH_MEAS_OUT0_S);

	//clear touch force ,select the Touch mode is Timer
	CLEAR_PERI_REG_MASK(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_START_EN_M|SENS_TOUCH_START_FORCE_M);

	//restore previous value
	WRITE_PERI_REG(SENS_SAR_TOUCH_ENABLE_REG, v0);
	return touch_value;
}
#endif

int TLSampleMethodTouchReadPreSample(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch)
{
	return 0;
}


int32_t TLSampleMethodTouchReadSample(struct TLStruct * data, uint8_t nSensors,
	uint8_t ch, bool inv)
{
	int32_t sample = 0;
	
	/* touchRead() is only available on ESP32 and Teensy 3.x and not Teensy 3.5 */

	#if ((IS_TEENSY_WITH_TOUCHREAD) || (IS_ESP32))
	struct TLStruct * dCh;
	int ch_pin;

	dCh = &(data[ch]);
	ch_pin = dCh->tlStructSampleMethod.touchRead.pin;
	if (inv) {
		/* Pseudo differential measurements are not supported */
		sample = 0;
	} else {

		if (ch_pin >= 0) {
			#if (IS_ESP32)
			/* discharge pin */
			pinMode(ch_pin, OUTPUT);
			digitalWrite(ch_pin, LOW);
			pinMode(ch_pin, INPUT);
			sample = esp32TouchRead(ch_pin);
			#else
			sample = touchRead(ch_pin);
			#endif

			#if (IS_ESP32)
			if (sample == 0) {
				/* 
				 * Workaround for ESP32 which sometimes returns
				 * 0 
				 */
				/* discharge pin */
				pinMode(ch_pin, OUTPUT);
				digitalWrite(ch_pin, LOW);
				pinMode(ch_pin, INPUT);
				sample = esp32TouchRead(ch_pin);
			}
			#endif
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
        int32_t tmp, scale;

        d = &(data[ch]);

	switch (d->filterType) {
	case TLStruct::filterTypeAverage:
                scale = (((int32_t) d->nMeasurementsPerSensor) << 1);
		break;
	case TLStruct::filterTypeSlewrateLimiter:
                scale = 2;
		break;
	case TLStruct::filterTypeMedian:
                scale = 2;
		break;
	default:
		/* Error! */
		scale = 2;
	}

        tmp = (d->scaleFactor * d->referenceValue * d->raw + (scale >> 1)) / 
		scale;
        d->value = tmp;
        /* Capacitance can be negative due to noise! */

        return 0;
}


int32_t TLSampleMethodTouchReadMapDelta(struct TLStruct * data, uint8_t nSensors,
		uint8_t ch, int length)
{
	int32_t n = -1;
	struct TLStruct * d;
	int32_t delta;

	d = &(data[ch]);

	delta = d->delta;

	n = map(100 * log(delta), TL_BAR_LOWER_PCT *
			log(d->calibratedMaxDelta), TL_BAR_UPPER_PCT *
			log(d->calibratedMaxDelta), 0.0f, (float) length);

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

	#if (IS_ESP32)
	d->direction = TLStruct::directionNegative;
	#else
	d->direction = TLStruct::directionPositive;
	#endif

	d->sampleType = TLStruct::sampleTypeNormal;
	d->filterType = TLStruct::filterTypeAverage;
	d->waterRejectPin = -1;
	d->waterRejectMode = TLStruct::waterRejectModeFloat;

	d->pin = &(d->tlStructSampleMethod.touchRead.pin);

	return 0;
}
