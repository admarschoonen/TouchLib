/*
 * CVDSensor.h - Capacitive sensing library based on CVD method for Arduino
 * https://github.com/AdmarSchoonen/CVDSensor
 * Copyright (c) 2016 Admar Schoonen
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

#ifndef CVDSensor_h
#define CVDSensor_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#include "WConstants.h"
#endif
#include <avr/io.h>
#include <math.h>

/* These strings are for human readability */
const char * const CVDButtonStateLabels[] = {
	"PreCalibrating", "Calibrating", "Released", "ReleasedToApproached",
	"Approached", "ApproachedToPressed", "ApproachedToReleased", "Pressed",
	"PressedToApproached"
};

struct CvdStruct {
	/* enum definitions */
	enum ButtonState {
		/*
		 * Button states.
		 * buttonStatePreCalibrating, buttonStateCalibrating,
		 * buttonStateReleased, buttonStateApproached and
		 * buttonStateApproachedToPressed can be regarded as "released"
		 * or "not touched". buttonStatePressed and
		 * buttonStatePressedToApproached can be regarded as "pressed"
		 * or "touched".
		 *
		 * In application code, this can be simplified by considering a
		 * button as "pressed" or "touched" if the button state is equal
		 * to or larger than buttonStatePressed.
		 *
		 * If desired, application can consider a button state larger
		 * than or equal to buttonStateApproached and smaller than or
		 * equal to buttonStateApproachedToReleased as "approached".
		 */
		buttonStatePreCalibrating = 0,
		buttonStateCalibrating,
		buttonStateReleased,
		buttonStateReleasedToApproached,
		buttonStateApproached,
		buttonStateApproachedToPressed,
		buttonStateApproachedToReleased,
		buttonStatePressed,
		buttonStatePressedToApproached
	};

	enum Direction {
		/*
		 * directionPositive means the capacitance is increased when a
		 * user touches the button (this is the default behaviour).
		 * directionNegative means the capacitance is decreased when a
		 * user touches the button (not very common).
		 */
		directionNegative,
		directionPositive
	};

	enum SampleType {
		/*
		 * SampleType specifies the way samples are taken.
		 * sampleTypeNormal means the sensor is first discharged and
		 * reference capacitor is charged. sampleTypeInverted means the
		 * sensor is first charged and the reference capacitor is
		 * discharged. sampleTypeDifferential means that both normal and
		 * inverted samples must be taken and the difference between
		 * those must be computed.
		 *
		 * sampleTypeDifferential is slower but more robust against
		 * interference. sampleTypeDifferential is default.
		 *
		 * Note: do not change values associated with these types!
		 *
 		 * sampleTypeNormal must be binary 01 and sampletypeInverted
 		 * must be binary 10 so that sampleTypeDifferential is equal to
 		 * sampleTypeNormal | sampleTypeInverted.
		 */
		sampleTypeNormal = 1,
		sampleTypeInverted = 2,
		sampleTypeDifferential = 3
	};

	/*
	 * These members are set to defaults upon initialization but can be
	 * overruled by the user.
	 */
	int pin;
	enum Direction direction;
	enum SampleType sampleType;
	float releasedToApproachedThreshold;
	float approachedToReleasedThreshold;
	float approachedToPressedThreshold;
	float pressedToApproachedThreshold;
	uint32_t releasedToApproachedTime;
	uint32_t approachedToReleasedTime;
	uint32_t approachedToPressedTime;
	uint32_t pressedToApproachedTime;
	bool enableSlewrateLimiter;
	unsigned long preCalibrationTime;
	unsigned long calibrationTime;
	unsigned long approachedTimeout;
	unsigned long pressedTimeout;
	uint16_t filterCoeff;
	bool forceCalibrationAfterApproached;
	bool forceCalibrationAfterPressed;
	bool setParallelCapacitanceManually;
	bool disableUpdateIfAnyButtonIsApproached;
	bool disableUpdateIfAnyButtonIsPressed;
	float referenceCapacitance; /* in pico Farad (pF) */
	float parallelCapacitance; /* in pico Farad (pF) */
	float capacitanceScaleFactor;
	float distanceOffset;
	float distanceScaleFactor;
	float relativePermittivity;
	float area; /* in mm^2 */
	uint32_t nChargesMin;
	uint32_t nChargesMax;
	bool useNChargesPadding;
	uint16_t chargeDelaySensor; /* delay to charge sensor in microseconds (us) */
	uint16_t chargeDelayADC; /* delay to charge ADC in microseconds (us) */

	/*
	 * Set enableTouchStateMachine to false to only use a sensor for
	 * capacitive sensing or during tuning. After startup, sensor will be in
	 * state buttonStatePreCalibrating followed by buttonStateCalibrating
	 * and after calibration the state will switch to buttonStateReleased
	 * and stay there.
	 */
	bool enableTouchStateMachine;

	/*
	 * Set enableNoisePowerMeasurement to true to measure noise power.
	 * This is useful during tuning or debugging but adds processing time.
	 */
	bool enableNoisePowerMeasurement;

	/* These members will be set by the init / sample methods. */
	uint8_t nSensors;
	uint8_t nMeasurementsPerSensor;
	int32_t raw;
	float capacitance; /* in pico Farad (pF) */
	float distance;
	float avg;
	float delta;
	float noisePower;
	enum ButtonState buttonState;
	const char * buttonStateLabel; /* human readable label */
	bool buttonIsCalibrating; /* use this to see if button is calibrating */
	bool buttonIsReleased; /* use this to see if button is released */
	bool buttonIsApproached; /* use this to see if button is approached */
	bool buttonIsPressed; /* use this to see if button is pressed */
	uint32_t counter;
	uint32_t recalCounter;
	uint32_t nCharges;
	uint32_t nChargesNext;
	unsigned long lastSampledAtTime;
	unsigned long stateChangedAtTime;
	bool slewrateFirstSample;
};

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
class CvdSensors
{
	public:
		struct CvdStruct data[N_SENSORS];
		uint8_t nSensors;

		/*
		 * Ideally scanOrder would be a static const uint8_t array the
		 * size of nSensors * nMeasurementPerSensor with a pseudo random
		 * sequence generated by the C preprocessor. I just don't know
		 * how to do this (or if it is possible at all). For now, we'll
		 * just use RAM and initialize it at start up, wasting some
		 * precious RAM space.
		 */
		uint8_t scanOrder[N_SENSORS * N_MEASUREMENTS_PER_SENSOR];
		uint8_t	nMeasurementsPerSensor;
		int8_t error;

		int8_t setDefaults(void);
		int8_t sample(void);
		void printScanOrder(void);
		CvdSensors(void);
		~CvdSensors(void);

	private:
		bool useCustomScanOrder;
		bool anyButtonIsApproached;
		bool anyButtonIsPressed;

		bool hasMux5(void);
		uint8_t channelToReference(uint8_t ch);
		void setAdcReferencePin(int pin);
		int8_t addChannel(uint8_t ch);
		void addSample(uint8_t ch, int32_t sample);
		void chargeADC(uint8_t ch, int ref_pin);
		void chargeSensor(uint8_t ch, int ch_pin);
		void charge(uint8_t ch, int ch_pin, int ref_pin);
		void setSensorAndReferencePins(int ch_pin, int ref_pin, bool
			inv);
		int sampleHalf(uint16_t pos, bool inv);
		bool isPressed(CvdStruct * d);
		bool isApproached(CvdStruct * d);
		bool isReleased(CvdStruct * d);
		void updateAvg(CvdStruct * d);
		void processStatePreCalibrating(CvdStruct * d);
		void processStateCalibrating(CvdStruct * d);
		void processStateReleased(CvdStruct * d);
		void processStateReleasedToApproached(CvdStruct * d);
		void processStateApproached(CvdStruct * d);
		void processStateApproachedToPressed(CvdStruct * d);
		void processStatePressed(CvdStruct * d);
		void processStatePressedToApproached(CvdStruct * d);
		void processStateApproachedToReleased(CvdStruct * d);
		void correctSample(uint8_t ch);
		void processSample(uint8_t ch);
		void updateNCharges(uint8_t ch);
		void resetButtonStateSummaries(uint8_t ch);
		void initScanOrder(void);
};

/* Actual implementation */
#define CVD_RELEASED_TO_APPROACHED_THRESHOLD_DEFAULT		50
#define CVD_APPROACHED_TO_RELEASED_THRESHOLD_DEFAULT		40
#define CVD_APPROACHED_TO_PRESSED_THRESHOLD_DEFAULT		150
#define CVD_PRESSED_TO_APPROACHED_THRESHOLD_DEFAULT		120
#define CVD_RELEASED_TO_APPROACHED_TIME_DEFAULT			10
#define CVD_APPROACHED_TO_RELEASED_TIME_DEFAULT			10
#define CVD_APPROACHED_TO_PRESSED_TIME_DEFAULT			10
#define CVD_PRESSED_TO_APPROACHED_TIME_DEFAULT			10
#define CVD_ENABLE_SLEW_RATE_LIMITER_DEFAULT			true
#define CVD_PRE_CALIBRATION_TIME_DEFAULT			100
#define CVD_CALIBRATION_TIME_DEFAULT				500
#define CVD_FILTER_COEFF_DEFAULT				128
#define CVD_APPROACHED_TIMEOUT_DEFAULT				300000
#define CVD_PRESSED_TIMEOUT_DEFAULT				CVD_APPROACHED_TIMEOUT_DEFAULT
#define CVD_FORCE_CALIBRATION_AFTER_APPROACHED_DEFAULT		false
#define CVD_FORCE_CALIBRATION_AFTER_PRESSED_DEFAULT		false
#define CVD_USE_CUSTOM_SCAN_ORDER_DEFAULT			false
#define CVD_ADC_RESOLUTION_BIT					10
#define CVD_ADC_MAX						((1 << CVD_ADC_RESOLUTION_BIT) - 1)
#define CVD_N_CHARGES_MIN_DEFAULT				1
#define CVD_N_CHARGES_MAX_DEFAULT				5
#define CVD_USE_N_CHARGES_PADDING_DEFAULT			true
#define CVD_CHARGE_DELAY_SENSOR_DEFAULT				0
#define CVD_CHARGE_DELAY_ADC_DEFAULT				0
#define CVD_ENABLE_TOUCH_STATE_MACHINE_DEFAULT			true
#define CVD_ENABLE_NOISE_POWER_MEASUREMENT_DEFAULT		false

#define CVD_REFERENCE_CAPACITANCE_DEFAULT			((float) 15) /* 15 pF */
#define CVD_CAPACITANCE_SCALE_FACTOR_DEFAULT			((float) 1)
#define CVD_PARALLEL_CAPACITANCE_DEFAULT			((float) 1000) /* pF */

#define CVD_DISTANCE_SCALE_FACTOR_DEFAULT			((float) 1)
#define CVD_DISTANCE_OFFSET_DEFAULT				((float) 0)

#define CVD_PERMITTIVITY_VACUUM					((float) (8.85E-12*1E12/1E3))
#define CVD_RELATIVE_PERMITTIVITY_DEFAULT			((float) 1)
#define CVD_AREA_DEFAULT					(10*10) /* 100 mm^2 */

#define CVD_SET_PARALLEL_CAPACITANCE_MANUALLY_DEFAULT		false
#define DISABLE_UPDATE_IF_ANY_BUTTON_IS_APPROACHED_DEFAULT	false
#define DISABLE_UPDATE_IF_ANY_BUTTON_IS_PRESSED_DEFAULT		false

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
int8_t CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::addChannel(uint8_t ch)
{
	long r;
	uint16_t n, pos, length;
	int8_t err = -1;

	length = ((uint16_t) nSensors) * ((uint16_t) nMeasurementsPerSensor);
	
	r = random(0, length);

	for (n = 0; n < length; n++) {
		pos = (n + r) % length;
		if (scanOrder[pos] == 255) {
			scanOrder[pos] = ch;
			err = 0;
			break;
		}
	}

	error = err;
	return err;
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::initScanOrder(void)
{
	uint16_t pos, length;
	uint8_t n, k;

	length = ((uint16_t) nSensors) * ((uint16_t) nMeasurementsPerSensor);

	for (pos = 0; pos < length; pos++) {
		scanOrder[pos] = 255;
	}
	
	/*
	 * Use a fixed seed so that scan order is pseudo random but always the
	 * same.
	 */
	randomSeed(nMeasurementsPerSensor);

	for (k = 0; k < nMeasurementsPerSensor; k++) {
		for (n = 0; n < nSensors; n++) {
			addChannel(n);
		}
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
int8_t CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::setDefaults(void)
{
	uint8_t n;
	
	error = 0;

	if (nSensors < 2) {
		error = -1;
	}

	if (error == 0) {
		this->anyButtonIsApproached = false;
		this->anyButtonIsPressed = false;
	}

#ifdef NUM_ANALOG_PINS
	if (error == 0) {
		for (n = 0; n < nSensors; n++) {
			if (cvdSensors[n].pin >= NUM_ANALOG_PINS) {
				error = -1;
			}
		}
	}
#endif

	if (error == 0) {
		this->useCustomScanOrder = CVD_USE_CUSTOM_SCAN_ORDER_DEFAULT;
	
		if (useCustomScanOrder == false) {
			initScanOrder();
		}
	}

	if (error == 0) {
		for (n = 0; n < nSensors; n++) {
			data[n].pin = n;
			data[n].direction = CvdStruct::directionPositive;
			data[n].sampleType = CvdStruct::sampleTypeDifferential;
			data[n].releasedToApproachedThreshold =
				CVD_RELEASED_TO_APPROACHED_THRESHOLD_DEFAULT;
			data[n].approachedToReleasedThreshold =
				CVD_APPROACHED_TO_RELEASED_THRESHOLD_DEFAULT;
			data[n].approachedToPressedThreshold =
				CVD_APPROACHED_TO_PRESSED_THRESHOLD_DEFAULT;
			data[n].pressedToApproachedThreshold =
				CVD_PRESSED_TO_APPROACHED_THRESHOLD_DEFAULT;
			data[n].releasedToApproachedTime =
				CVD_RELEASED_TO_APPROACHED_TIME_DEFAULT;
			data[n].approachedToReleasedTime =
				CVD_APPROACHED_TO_RELEASED_TIME_DEFAULT;
			data[n].approachedToPressedTime =
				CVD_APPROACHED_TO_PRESSED_TIME_DEFAULT;
			data[n].pressedToApproachedTime =
				CVD_PRESSED_TO_APPROACHED_TIME_DEFAULT;
			data[n].enableSlewrateLimiter = 
				CVD_ENABLE_SLEW_RATE_LIMITER_DEFAULT;
			data[n].preCalibrationTime =
				CVD_PRE_CALIBRATION_TIME_DEFAULT;
			data[n].calibrationTime =
				CVD_CALIBRATION_TIME_DEFAULT;
			data[n].filterCoeff =
				CVD_FILTER_COEFF_DEFAULT;
			data[n].approachedTimeout =
				CVD_APPROACHED_TIMEOUT_DEFAULT;
			data[n].pressedTimeout =
				CVD_PRESSED_TIMEOUT_DEFAULT;
			data[n].forceCalibrationAfterApproached =
				CVD_FORCE_CALIBRATION_AFTER_APPROACHED_DEFAULT;
			data[n].forceCalibrationAfterPressed =
				CVD_FORCE_CALIBRATION_AFTER_PRESSED_DEFAULT;
			data[n].enableTouchStateMachine = 
				CVD_ENABLE_TOUCH_STATE_MACHINE_DEFAULT;
			data[n].enableNoisePowerMeasurement =
				CVD_ENABLE_NOISE_POWER_MEASUREMENT_DEFAULT;
			data[n].parallelCapacitance =
				CVD_PARALLEL_CAPACITANCE_DEFAULT;
			data[n].referenceCapacitance =
				CVD_REFERENCE_CAPACITANCE_DEFAULT;
			data[n].capacitanceScaleFactor =
				CVD_CAPACITANCE_SCALE_FACTOR_DEFAULT;
			data[n].distanceScaleFactor =
				CVD_DISTANCE_SCALE_FACTOR_DEFAULT;
			data[n].relativePermittivity =
				CVD_RELATIVE_PERMITTIVITY_DEFAULT;
			data[n].distanceOffset = CVD_DISTANCE_OFFSET_DEFAULT;
			data[n].area = CVD_AREA_DEFAULT;
			data[n].setParallelCapacitanceManually =
				CVD_SET_PARALLEL_CAPACITANCE_MANUALLY_DEFAULT;
			data[n].disableUpdateIfAnyButtonIsApproached =
				DISABLE_UPDATE_IF_ANY_BUTTON_IS_APPROACHED_DEFAULT;
			data[n].disableUpdateIfAnyButtonIsPressed =
				DISABLE_UPDATE_IF_ANY_BUTTON_IS_PRESSED_DEFAULT;

			if (!data[n].setParallelCapacitanceManually) {
				/*
				 * Set parallelCapacitance to 0; will be updated
				 * after calibration.
				 */
				data[n].parallelCapacitance = 0;
			}
		}
	}

	return error;
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::~CvdSensors(void)
{
	/* Nothing to destroy */
}

#warning overload constructor with uint8_t customScanOrder[]
template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::CvdSensors(void)
{
	uint8_t n;
	unsigned long now;
	
	error = 0;

	if (N_SENSORS < 2) {
		error = -1;
	} else {
		nSensors = N_SENSORS;
	}

	if ((error == 0) && (N_MEASUREMENTS_PER_SENSOR >= 1)) {
		nMeasurementsPerSensor = N_MEASUREMENTS_PER_SENSOR;
	} else {
		error = -1;
	}

#ifdef NUM_ANALOG_PINS
	if (error == 0) {
		for (n = 0; n < nSensors; n++) {
			if (cvdSensors[n].pin >= NUM_ANALOG_PINS) {
				error = -1;
			}
		}
	}
#endif

	if (error == 0) {
		initScanOrder();
	}

	if (error == 0) {
		setDefaults();
	}

	if (error == 0) {
		now = millis();

		for (n = 0; n < nSensors; n++) {
			resetButtonStateSummaries(n);
			data[n].buttonState = 
				CvdStruct::buttonStatePreCalibrating;
			data[n].buttonStateLabel =
				CVDButtonStateLabels[data[n].buttonState];
			data[n].counter = 0;
			data[n].raw = 0;
			data[n].capacitance = 0;
			data[n].avg = 0;
			data[n].noisePower = 0;
			data[n].delta = 0;
			data[n].nCharges = CVD_N_CHARGES_MIN_DEFAULT;
			data[n].nChargesNext = CVD_N_CHARGES_MIN_DEFAULT;
			data[n].nChargesMin = CVD_N_CHARGES_MIN_DEFAULT;
			data[n].nChargesMax = CVD_N_CHARGES_MAX_DEFAULT;
			data[n].useNChargesPadding =
				CVD_USE_N_CHARGES_PADDING_DEFAULT;
			data[n].chargeDelaySensor = CVD_CHARGE_DELAY_SENSOR_DEFAULT;
			data[n].chargeDelayADC = CVD_CHARGE_DELAY_ADC_DEFAULT;
			data[n].stateChangedAtTime = now;
			data[n].lastSampledAtTime = 0;
		}
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
uint8_t CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::channelToReference(uint8_t ch)
{
	uint8_t ref;

	if (ch == nSensors - 1)
		ref = 0;
	else
		ref = ch + 1;

	return ref;
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
bool CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::hasMux5(void)
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

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::setAdcReferencePin(int pin)
{
	unsigned char mux;

	if (hasMux5()) {
		if (pin < 8) {
			mux = pin;
		} else {
			mux = 0x20 + (pin - 8);
		}

		ADMUX &= ~0x1F;
		ADMUX |= (mux & 0x1F);
		if (mux > 0x1F) {
			ADCSRB |= 0x08;
		} else {
			ADCSRB &= ~0x08;
		}
	} else {
		mux = pin;
		ADMUX &= ~0x0F;
		ADMUX |= (mux & 0x0F);
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::setSensorAndReferencePins(
		int ch_pin, int ref_pin, bool inv)
{
	/* Set reference pin as output and high. */
	pinMode(A0 + ref_pin, OUTPUT);
	if (inv) {
		digitalWrite(A0 + ref_pin, LOW);
	} else {
		digitalWrite(A0 + ref_pin, HIGH);
	}

	/* Set sensor pin as output and low (discharge sensor). */
	pinMode(A0 + ch_pin, OUTPUT);
	if (inv) {
		digitalWrite(A0 + ch_pin, HIGH);
	} else {
		digitalWrite(A0 + ch_pin, LOW);
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::chargeADC(uint8_t ch,
		int ref_pin)
{
	/* Set ADC to reference pin (charge Chold). */
	setAdcReferencePin(ref_pin);

	if (data[ch].chargeDelayADC) {
		delayMicroseconds(data[ch].chargeDelayADC);
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::chargeSensor(uint8_t ch,
		int ch_pin)
{
	/*
	 * Set ADC to sensor pin (transfer charge from Chold to Csense).
	 */
	setAdcReferencePin(ch_pin);

	if (data[ch].chargeDelaySensor) {
		delayMicroseconds(data[ch].chargeDelaySensor);
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::charge(uint8_t ch,
		int ch_pin, int ref_pin)
{
	chargeADC(ch, ref_pin);
	chargeSensor(ch, ch_pin);
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
int CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::sampleHalf(uint16_t pos,
		bool inv)
{
	uint8_t ch, ref;
	int ch_pin, ref_pin, sample;
	uint8_t i;

	ch = scanOrder[pos];
	ref = channelToReference(ch);
	ch_pin = data[ch].pin;
	ref_pin = data[ref].pin;

	setSensorAndReferencePins(ch_pin, ref_pin, inv);

	/* Set sensor pin as analog input. */
	pinMode(A0 + ch_pin, INPUT);

	/*
	 * Charge nCharges - 1 times to account for the charge during the
	 * analogRead() below.
	 */
	for (i = 0; i < data[ch].nCharges - 1; i++) {
		charge(ch, ch_pin, ref_pin);
	}

	/* Set ADC to reference pin (charge internal capacitor). */
	chargeADC(ch, ref_pin);

	/* Read sensor. */
	sample = analogRead(ch_pin);

	if (data[ch].useNChargesPadding) {
		/*
		 * Increment i before starting the loop to account for the
		 * charge during the analogRead() above.
		 */
		for (++i; i < data[ch].nChargesMax; i++) {
			charge(ch, ch_pin, ref_pin);
		}
	}

	/* Discharge sensor. */
	pinMode(A0 + ch_pin, OUTPUT);
	digitalWrite(A0 + ch_pin, LOW);

	return sample;
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::addSample(uint8_t ch, int32_t sample)
{
	if (data[ch].enableSlewrateLimiter) {
		if (data[ch].slewrateFirstSample) {
			data[ch].raw = sample;
			data[ch].slewrateFirstSample = false;
		} else {
			if (sample > data[ch].raw) {
				data[ch].raw++;
			} 
			if (sample < data[ch].raw) {
				data[ch].raw--;
			}
		}
	} else {
		data[ch].raw += sample;
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
bool CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::isReleased(CvdStruct * d)
{
	bool isReleased = false;

	if ((d->direction == CvdStruct::directionPositive) &&
			(d->delta <= d->approachedToReleasedThreshold)) {
		isReleased = true;
	}
	if ((d->direction == CvdStruct::directionNegative) &&
			(-d->delta <= d->approachedToReleasedThreshold)) {
		isReleased = true;
	}

	return isReleased;
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
bool CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::isApproached(CvdStruct * d)
{
	bool isApproached = false;

	if ((d->direction == CvdStruct::directionPositive) &&
			(d->delta >= d->releasedToApproachedThreshold)) {
		isApproached = true;
	}
	if ((d->direction == CvdStruct::directionNegative) &&
			(-d->delta >= d->releasedToApproachedThreshold)) {
		isApproached = true;
	}

	return isApproached;
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
bool CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::isPressed(CvdStruct * d)
{
	bool isPressed = false;

	if ((d->direction == CvdStruct::directionPositive) &&
			(d->delta >= d->approachedToPressedThreshold)) {
		isPressed = true;
	}
	if ((d->direction == CvdStruct::directionNegative) &&
			(-d->delta >= d->approachedToPressedThreshold)) {
		isPressed = true;
	}

	return isPressed;
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::updateAvg(CvdStruct * d)
{
	float s;

	if (d->disableUpdateIfAnyButtonIsApproached &&
			this->anyButtonIsApproached) {
		return;
	}
	if (d->disableUpdateIfAnyButtonIsPressed &&
			this->anyButtonIsPressed) {
		return;
	}

	d->avg = (d->counter * d->avg + d->capacitance) / (d->counter + 1);

	if (d->enableNoisePowerMeasurement) {
		s = d->delta * d->delta;
		d->noisePower = (d->counter * d->noisePower + s) / 
			(d->counter + 1);
	}

	if (d->counter < d->filterCoeff - 1) {
		d->counter++;
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::processStatePreCalibrating(CvdStruct * d)
{
	if (d->lastSampledAtTime - d->stateChangedAtTime >= d->preCalibrationTime) {
		d->stateChangedAtTime = d->lastSampledAtTime;
		d->buttonState = CvdStruct::buttonStateCalibrating;
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::processStateCalibrating(CvdStruct * d)
{
	if (d->lastSampledAtTime - d->stateChangedAtTime < d->calibrationTime) {
		updateAvg(d);
	} else {
		d->stateChangedAtTime = d->lastSampledAtTime;
		d->buttonState = CvdStruct::buttonStateReleased;

		if (!d->setParallelCapacitanceManually) {
			d->parallelCapacitance = d->avg;
		}
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::processStateReleased(CvdStruct * d)
{
	if ((d->enableTouchStateMachine) && (isApproached(d))) {
		d->stateChangedAtTime = d->lastSampledAtTime;
		d->buttonState = CvdStruct::buttonStateReleasedToApproached;
	} else {
		updateAvg(d);
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::processStateReleasedToApproached(CvdStruct * d)
{
	/* Do not update average in this state. */

	if (!d->enableTouchStateMachine)
		return;

	if (isApproached(d)) {
		if (d->lastSampledAtTime - d->stateChangedAtTime >=
				d->releasedToApproachedTime) {
			d->stateChangedAtTime = d->lastSampledAtTime;
			d->buttonState = CvdStruct::buttonStateApproached;
		}
	} else {
		d->stateChangedAtTime = d->lastSampledAtTime;
		d->buttonState = CvdStruct::buttonStateReleased;
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::processStateApproached(CvdStruct * d)
{
	if (!d->enableTouchStateMachine)
		return;

	if (isReleased(d)) {
		d->stateChangedAtTime = d->lastSampledAtTime;
		d->buttonState = CvdStruct::buttonStateApproachedToReleased;
	} else if (isPressed(d)) {
		d->stateChangedAtTime = d->lastSampledAtTime;
		d->buttonState = CvdStruct::buttonStateApproachedToPressed;
	} else if ((d->approachedTimeout > 0) && (d->lastSampledAtTime - 
			d->stateChangedAtTime > d->approachedTimeout)) {
		d->counter = 0;
		d->avg = 0;
		d->noisePower = 0;
		d->stateChangedAtTime = d->lastSampledAtTime;
		d->buttonState = CvdStruct::buttonStateCalibrating;

		if (!d->setParallelCapacitanceManually) {
			/*
			 * Set parallelCapacitance to 0; will be updated
			 * after calibration.
			 */
			d->parallelCapacitance = 0;
		}
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::processStateApproachedToPressed(CvdStruct * d)
{
	/* Do not update average in this state. */

	if (!d->enableTouchStateMachine)
		return;

	if (isPressed(d)) {
		if (d->lastSampledAtTime - d->stateChangedAtTime >=
				d->approachedToPressedTime) {
			d->stateChangedAtTime = d->lastSampledAtTime;
			d->buttonState = CvdStruct::buttonStatePressed;
		}
	} else {
		d->stateChangedAtTime = d->lastSampledAtTime;
		d->buttonState = CvdStruct::buttonStateApproached;
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::processStateApproachedToReleased(CvdStruct * d)
{
	if (!d->enableTouchStateMachine)
		return;

	if (isReleased(d)) {
		if (d->lastSampledAtTime - d->stateChangedAtTime >=
				d->approachedToReleasedTime) {
			d->stateChangedAtTime = d->lastSampledAtTime;
			if (d->forceCalibrationAfterApproached) {
				d->counter = 0;
				d->avg = 0;
				d->noisePower = 0;
				d->buttonState =
					CvdStruct::buttonStateCalibrating;
				if (!d->setParallelCapacitanceManually) {
					/*
					 * Set parallelCapacitance to 0; will be
					 * updated after calibration.
					 */
					d->parallelCapacitance = 0;
				}
			} else {
				d->buttonState =
					CvdStruct::buttonStateReleased;
			}
		}
	} else {
		/* Do not set stateChangedAtTime here. */

		d->buttonState = CvdStruct::buttonStateApproached;
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::processStatePressed(CvdStruct * d)
{
	if (!d->enableTouchStateMachine)
		return;

	if (isPressed(d)) {
		if ((d->pressedTimeout > 0) && (d->lastSampledAtTime - 
				d->stateChangedAtTime > d->pressedTimeout)) {
			d->counter = 0;
			d->avg = 0;
			d->noisePower = 0;
			d->stateChangedAtTime = d->lastSampledAtTime;
			d->buttonState = CvdStruct::buttonStateCalibrating;

			if (!d->setParallelCapacitanceManually) {
				/*
				 * Set parallelCapacitance to 0; will be updated
				 * after calibration.
				 */
				d->parallelCapacitance = 0;
			}
		}
	} else {
		d->stateChangedAtTime = d->lastSampledAtTime;
		d->buttonState = CvdStruct::buttonStatePressedToApproached;
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::processStatePressedToApproached(CvdStruct * d)
{
	if (!d->enableTouchStateMachine)
		return;

	if (isPressed(d)) {
		/* Do not set stateChangedAtTime here. */

		d->buttonState = CvdStruct::buttonStatePressed;
	} else {
		if (d->lastSampledAtTime - d->stateChangedAtTime >= 
				d->pressedToApproachedTime) {
			d->stateChangedAtTime = d->lastSampledAtTime;
			if (d->forceCalibrationAfterPressed) {
				d->counter = 0;
				d->avg = 0;
				d->noisePower = 0;
				d->buttonState =
					CvdStruct::buttonStateCalibrating;
				if (!d->setParallelCapacitanceManually) {
					/*
					 * Set parallelCapacitance to 0; will be
					 * updated after calibration.
					 */
					d->parallelCapacitance = 0;
				}
			} else {
				d->buttonState =
					CvdStruct::buttonStateApproached;
			}
		}
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::processSample(uint8_t ch)
{
	CvdStruct * d;

	d = &(data[ch]);

	d->delta = d->capacitance - d->avg;

	switch (d->buttonState) {
	case CvdStruct::buttonStatePreCalibrating:
		processStatePreCalibrating(d);
		break;
	case CvdStruct::buttonStateCalibrating:
		processStateCalibrating(d);
		break;
	case CvdStruct::buttonStateReleased:
		processStateReleased(d);
		break;
	case CvdStruct::buttonStateReleasedToApproached:
		processStateReleasedToApproached(d);
		break;
	case CvdStruct::buttonStateApproached:
		processStateApproached(d);
		break;
	case CvdStruct::buttonStateApproachedToReleased:
		processStateApproachedToReleased(d);
		break;
	case CvdStruct::buttonStateApproachedToPressed:
		processStateApproachedToPressed(d);
		break;
	case CvdStruct::buttonStatePressed:
		processStatePressed(d);
		break;
	case CvdStruct::buttonStatePressedToApproached:
		processStatePressedToApproached(d);
		break;
	}

	d->buttonStateLabel = CVDButtonStateLabels[d->buttonState];
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::correctSample(uint8_t ch)
{
	CvdStruct * d;
	float tmp, scale;

	d = &(data[ch]);

	if (d->enableSlewrateLimiter) {
		scale = (float) ((CVD_ADC_MAX + 1) << 2);
	} else {
		scale = ((float) (nMeasurementsPerSensor << 1)) *
			((float) (CVD_ADC_MAX + 1));
	}

	tmp = 1 - ((float) d->raw) / scale;
	tmp = pow(tmp, -((float) 1) / ((float) d->nCharges)) - ((float) 1);
	tmp = ((float) 1) / tmp;

	d->nChargesNext = (int32_t) (ceilf(tmp));
	if (d->nChargesNext < CVD_N_CHARGES_MIN_DEFAULT) {
		d->nChargesNext = CVD_N_CHARGES_MIN_DEFAULT;
	}
	if (d->nChargesNext > CVD_N_CHARGES_MAX_DEFAULT) {
		d->nChargesNext = CVD_N_CHARGES_MAX_DEFAULT;
	}

	tmp = scale * tmp * d->capacitanceScaleFactor /
		d->referenceCapacitance - d->parallelCapacitance;
	d->capacitance = tmp;
	/* Capacitance can be negative due to noise! */

	d->distance = (d->distanceScaleFactor * CVD_PERMITTIVITY_VACUUM *
		d->relativePermittivity * d->area / tmp) - d->distanceOffset;
	/* Distance can be negative due to noise! */
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::updateNCharges(uint8_t ch)
{
	CvdStruct * d;

	d = &(data[ch]);
	d->nCharges = d->nChargesNext;
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::resetButtonStateSummaries(uint8_t ch)
{
	data[ch].buttonIsCalibrating = false;
	data[ch].buttonIsReleased = false;
	data[ch].buttonIsApproached = false;
	data[ch].buttonIsPressed = false;
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
int8_t CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::sample(void)
{
	uint16_t length, pos;
	uint8_t ch;
	int sample1 = 0, sample2 = 0;
	int32_t sum;
	unsigned long now;

	length = ((uint16_t) nSensors) * ((uint16_t)
		nMeasurementsPerSensor);

	for (ch = 0; ch < nSensors; ch++) {
		data[ch].raw = 0;
		data[ch].slewrateFirstSample = true;
	}

	for (pos = 0; pos < length; pos++) {
		if (data[scanOrder[pos]].sampleType &
				CvdStruct::sampleTypeNormal) {
			sample1 = sampleHalf(pos, false);
		}
		if (data[scanOrder[pos]].sampleType &
				CvdStruct::sampleTypeInverted) {
			sample2 = CVD_ADC_MAX - sampleHalf(pos, true);
		}

		/*
		 * For sampleTypeNormal and sampleTypeInverted: scale by factor
		 * 2 to get same amplitude as with sampleTypeDifferential.
		 */
		if (data[scanOrder[pos]].sampleType ==
				CvdStruct::sampleTypeNormal) {
			sample1 = sample1 << 1;
		}
		if (data[scanOrder[pos]].sampleType ==
				CvdStruct::sampleTypeInverted) {
			sample2 = sample2 << 1;
		}

		sum = sample1 + sample2;
		addSample(scanOrder[pos], sum);
	}
	
	now = millis();

	for (ch = 0; ch < nSensors; ch++) {
		data[ch].lastSampledAtTime = now;
		correctSample(ch);
		processSample(ch);
	}

	this->anyButtonIsApproached = false;
	this->anyButtonIsPressed = false;
	for (ch = 0; ch < nSensors; ch++) {
		resetButtonStateSummaries(ch);
		if (data[ch].buttonState <= CvdStruct::buttonStateCalibrating) {
			data[ch].buttonIsCalibrating = true;
		}
		if ((data[ch].buttonState >= CvdStruct::buttonStateReleased) &&
				(data[ch].buttonState <=
				CvdStruct::buttonStateReleasedToApproached)) {
			data[ch].buttonIsReleased = true;
		}
		if (data[ch].buttonState >= CvdStruct::buttonStateApproached) {
			data[ch].buttonIsApproached = true;
			this->anyButtonIsApproached = true;
		}
		if (data[ch].buttonState >= CvdStruct::buttonStatePressed) {
			data[ch].buttonIsPressed = true;
			this->anyButtonIsPressed = true;
		}
		updateNCharges(ch);
	}

	return error;
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::printScanOrder(void)
{
	uint16_t n;

	for (n = 0; n < ((uint16_t) nSensors) * ((uint16_t)
			nMeasurementsPerSensor); n++) {
		Serial.print(scanOrder[n]);
		Serial.print(" ");
	}
	Serial.println();
}

#endif
