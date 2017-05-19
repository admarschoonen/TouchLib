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

struct CvdStruct {
	/* enum definitions */
	enum ButtonState {
		/*
		 * Button states.
		 * buttonStateReleased and buttonStateReleasedToPressed can be
		 * regarded as "released" or "not touched".
		 * buttonStatePressed and buttonStatePressedToReleased can be
		 * regarded as "pressed" or "touched".
		 *
		 * In application code, this can be simplified by considering a
		 * button as "pressed" or "touched" if the button state is equal
		 * to or larger than buttonStatePressed.
		 */
		buttonStateCalibrating = 0,
		buttonStateReleased,
		buttonStateReleasedToPressed,
		buttonStatePressed,
		buttonStatePressedToReleased
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

	/* These members should be set by the user. */
	int pin;

	/*
	 * These members are set to defaults upon initialization but can be
	 * overruled by the user.
	 */
	enum Direction direction;
	enum SampleType sampleType;
	int32_t releasedToPressedThreshold;
	int32_t pressedToReleasedThreshold;
	uint32_t releasedToPressedTime;
	uint32_t pressedToReleasedTime;
	bool enableSlewrateLimiter;
	unsigned long calibrationTime;
	unsigned long pressedTimeout;
	bool forceCalibrationAfterRelease;
	bool setParallelCapacitanceManually;
	float referenceCapacitance; /* in femto Farad (pF) */
	float parallelCapacitance; /* in femto Farad (pF) */
	float capacitanceScaleFactor;
	float distanceOffset;
	float distanceScaleFactor;
	float relativePermittivity;
	float area; /* in mm^2 */

	/*
	 * Set enableTouchStateMachine to false to only use a sensor for
	 * capacitive sensing. After startup, sensor will be in state
	 * buttonStateCalibrating and after calibration switch to
	 * buttonStateReleased and stay there.
	 */
	bool enableTouchStateMachine;

	/* These members will be set by the init / sample methods. */
	uint8_t nSensors;
	uint8_t nMeasurementsPerSensor;
	int32_t raw;
	float capacitance; /* in femto Farad (pF) */
	float distance;
	float avg;
	float delta;
	enum ButtonState buttonState;
	uint32_t counter;
	uint32_t recalCounter;
	uint8_t nCharges;
	uint8_t nChargesNext;
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
		 * how to do this (or if it is possible at all).  For now, we'll
		 * just ask the user to allocate the space and we'll initialize
		 * it at start up, wasting some precious RAM space. If the user
		 * needs to preserve RAM, he should create his own pseudo random
		 * sequence and make the scanOrder array a const uint8_t.
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

		bool hasMux5(void);
		uint8_t channelToReference(uint8_t ch);
		void setAdcReferencePin(int pin);
		int8_t addChannel(uint8_t ch);
		void addSample(uint8_t ch, int32_t sample);
		int sampleHalf(uint16_t pos, bool inv);
		bool isPressed(CvdStruct * d);
		bool isReleased(CvdStruct * d);
		void updateAvg(CvdStruct * d);
		void processStateCalibrating(CvdStruct * d);
		void processStateReleased(CvdStruct * d);
		void processStateReleasedToPressed(CvdStruct * d);
		void processStatePressed(CvdStruct * d);
		void processStatePressedToReleased(CvdStruct * d);
		void correctSample(uint8_t ch);
		void processSample(uint8_t ch);
		void updateNCharges(uint8_t ch);
		void initScanOrder(void);
};

/* Actual implementation */
#define CVD_SENSOR_DEFAULT_N_CHARGES			2
#define CVD_RELEASED_TO_PRESSED_THRESHOLD_DEFAULT	50
#define CVD_PRESSED_TO_RELEASED_THRESHOLD_DEFAULT	50
#define CVD_RELEASED_TO_PRESSED_TIME_DEFAULT		20
#define CVD_PRESSED_TO_RELEASED_TIME_DEFAULT		20
#define CVD_ENABLE_SLEW_RATE_LIMITER_DEFAULT		false
#define CVD_CALIBRATION_TIME_DEFAULT			500
#define CVD_PRESSED_TIMEOUT_DEFAULT			300000
#define CVD_FORCE_CALIBRATION_AFTER_RELEASE_DEFAULT	false
#define CVD_USE_CUSTOM_SCAN_ORDER_DEFAULT		false
#define CVD_ADC_RESOLUTION_BIT				10
#define CVD_ADC_MAX					((1 << CVD_ADC_RESOLUTION_BIT) - 1)
#define CVD_N_CHARGES_MAX				5
#define CVD_ENABLE_TOUCH_STATE_MACHINE_DEFAULT		true

#define CVD_REFERENCE_CAPACITANCE_DEFAULT		((float) 15) /* 15 pF */
#define CVD_CAPACITANCE_SCALE_FACTOR_DEFAULT		((float) 1)
#define CVD_PARALLEL_CAPACITANCE_DEFAULT		((float) 1000) /* pF */

#define CVD_DISTANCE_SCALE_FACTOR_DEFAULT		((float) 1)
#define CVD_DISTANCE_OFFSET_DEFAULT			((float) 0)

#define CVD_PERMITTIVITY_VACUUM				((float) (8.85E-12*1E12/1E3))
#define CVD_RELATIVE_PERMITTIVITY_DEFAULT		((float) 1)
#define CVD_AREA_DEFAULT				(10*10) /* 100 mm^2 */

#define CVD_SET_PARALLEL_CAPACITANCE_MANUALLY_DEFAULT	true

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
	unsigned long now;
	
	error = 0;

	if (nSensors < 2) {
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
		this->useCustomScanOrder = CVD_USE_CUSTOM_SCAN_ORDER_DEFAULT;
	
		if (useCustomScanOrder == false) {
			initScanOrder();
		}
	}

	if (error == 0) {
		now = millis();

		for (n = 0; n < nSensors; n++) {
			data[n].pin = n;
			data[n].direction = CvdStruct::directionPositive;
			data[n].sampleType = CvdStruct::sampleTypeDifferential;
			data[n].releasedToPressedThreshold =
				CVD_RELEASED_TO_PRESSED_THRESHOLD_DEFAULT;
			data[n].pressedToReleasedThreshold =
				CVD_PRESSED_TO_RELEASED_THRESHOLD_DEFAULT;
			data[n].releasedToPressedTime = 
				CVD_RELEASED_TO_PRESSED_TIME_DEFAULT;
			data[n].pressedToReleasedTime = 
				CVD_PRESSED_TO_RELEASED_TIME_DEFAULT;
			data[n].enableSlewrateLimiter = 
				CVD_ENABLE_SLEW_RATE_LIMITER_DEFAULT;
			data[n].calibrationTime =
				CVD_CALIBRATION_TIME_DEFAULT;
			data[n].pressedTimeout =
				CVD_PRESSED_TIMEOUT_DEFAULT;
			data[n].forceCalibrationAfterRelease =
				CVD_FORCE_CALIBRATION_AFTER_RELEASE_DEFAULT;
			data[n].enableTouchStateMachine = 
				CVD_ENABLE_TOUCH_STATE_MACHINE_DEFAULT;
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

			if (data[n].setParallelCapacitanceManually) {
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
			data[n].buttonState = 
				CvdStruct::buttonStateCalibrating;
			data[n].counter = 0;
			data[n].raw = 0;
			data[n].capacitance = 0;
			data[n].avg = 0;
			data[n].delta = 0;
			data[n].nCharges = CVD_SENSOR_DEFAULT_N_CHARGES;
			data[n].nChargesNext = CVD_SENSOR_DEFAULT_N_CHARGES;
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
int CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::sampleHalf(uint16_t pos, bool inv)
{
	uint8_t ch, ref;
	int ch_pin, ref_pin, sample;
	uint8_t i;

	ch = scanOrder[pos];
	ref = channelToReference(ch);
	ch_pin = data[ch].pin;
	ref_pin = data[ref].pin;

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

	/* Set sensor pin as analog input. */
	pinMode(A0 + ch_pin, INPUT);

	for (i = 0; i < data[ch].nCharges - 1; i++) {
		/* Set ADC to reference pin (charge internal capacitor). */
		setAdcReferencePin(ref_pin);
		/*
		 * Set ADC to sensor pin (transfer charge from Chold to 
		 * Csense).
		 */
		setAdcReferencePin(ch_pin);
	}

	/* Set ADC to reference pin (charge internal capacitor). */
	setAdcReferencePin(ref_pin);

	/* Read sensor. */
	sample = analogRead(ch_pin);

	/* Discharge sensor. */
	pinMode(A0 + ch_pin, OUTPUT);
	digitalWrite(A0 + ch_pin, LOW);

	return sample;
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::addSample(uint8_t ch, int32_t sample)
{
	int32_t inc;

	if (data[ch].enableSlewrateLimiter) {
		inc = 0;
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
bool CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::isPressed(CvdStruct * d)
{
	bool isPressed = false;

	if ((d->direction == CvdStruct::directionPositive) &&
			(d->delta >= d->releasedToPressedThreshold)) {
		isPressed = true;
	}
	if ((d->direction == CvdStruct::directionNegative) &&
			(-d->delta >= d->releasedToPressedThreshold)) {
		isPressed = true;
	}

	return isPressed;
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
bool CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::isReleased(CvdStruct * d)
{
	bool isReleased = false;

	if ((d->direction == CvdStruct::directionPositive) &&
			(d->delta <= d->pressedToReleasedThreshold)) {
		isReleased = true;
	}
	if ((d->direction == CvdStruct::directionNegative) &&
			(-d->delta <= d->pressedToReleasedThreshold)) {
		isReleased = true;
	}

	return isReleased;
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::updateAvg(CvdStruct * d)
{
	d->avg = ((d->counter - 1) * d->avg + d->capacitance) / d->counter;
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::processStateCalibrating(CvdStruct * d)
{
	if (d->lastSampledAtTime - d->stateChangedAtTime < d->calibrationTime) {
		d->avg = (d->counter * d->avg + d->capacitance) /
			(d->counter + 1);
		d->counter++;
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
	if (d->enableTouchStateMachine) {
		if (isPressed(d)) {
			d->stateChangedAtTime = d->lastSampledAtTime;
			d->buttonState = CvdStruct::buttonStateReleasedToPressed;
		} else {
			updateAvg(d);
		}
	} else {
		updateAvg(d);
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::processStateReleasedToPressed(CvdStruct * d)
{
	/* Do not update average in this state. */

	if (isPressed(d)) {
		if (d->lastSampledAtTime - d->stateChangedAtTime >=
				d->releasedToPressedTime) {
			d->stateChangedAtTime = d->lastSampledAtTime;
			d->buttonState = CvdStruct::buttonStatePressed;
		}
	} else {
		d->stateChangedAtTime = d->lastSampledAtTime;
		d->buttonState = CvdStruct::buttonStateReleased;
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::processStatePressed(CvdStruct * d)
{
	if (isReleased(d)) {
		d->stateChangedAtTime = d->lastSampledAtTime;
		d->buttonState = CvdStruct::buttonStatePressedToReleased;
	} else {
		if ((d->pressedTimeout > 0) && (d->lastSampledAtTime - 
				d->stateChangedAtTime > d->pressedTimeout)) {
			d->counter = 0;
			d->avg = 0;
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
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::processStatePressedToReleased(CvdStruct * d)
{
	if (isReleased(d)) {
		if (d->lastSampledAtTime - d->stateChangedAtTime >= 
				d->pressedToReleasedTime) {
			d->stateChangedAtTime = d->lastSampledAtTime;
			if (d->forceCalibrationAfterRelease) {
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
				d->buttonState = CvdStruct::buttonStateReleased;
			}
		}
	} else {
		/* Do not set stateChangedAtTime here. */

		d->buttonState = CvdStruct::buttonStatePressed;
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::processSample(uint8_t ch)
{
	CvdStruct * d;

	d = &(data[ch]);

	d->delta = d->capacitance - d->avg;

	switch (d->buttonState) {
	case CvdStruct::buttonStateCalibrating:
		processStateCalibrating(d);
		break;
	case CvdStruct::buttonStateReleased:
		processStateReleased(d);
		break;
	case CvdStruct::buttonStateReleasedToPressed:
		processStateReleasedToPressed(d);
		break;
	case CvdStruct::buttonStatePressed:
		processStatePressed(d);
		break;
	case CvdStruct::buttonStatePressedToReleased:
		processStatePressedToReleased(d);
		break;
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::correctSample(uint8_t ch)
{
	CvdStruct * d;
	float tmp, scale;

	d = &(data[ch]);

	/*
	 * Multiply nMeasurementsPerSensor by 2 since sample() scaled it by 2 as
	 * well (except for sampleTypeDifferential, since that type has factor 2
	 * built in).
	 */
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
	if (d->nChargesNext < 1) {
		d->nChargesNext = 1;
	}
	if (d->nChargesNext > CVD_N_CHARGES_MAX) {
		d->nChargesNext = CVD_N_CHARGES_MAX;
	}

	tmp = scale * tmp * d->capacitanceScaleFactor /
		d->referenceCapacitance - d->parallelCapacitance;
	d->capacitance = tmp;
	if (d->capacitance < 0) {
		d->capacitance = 0;
	}

	d->distance = (d->distanceScaleFactor * CVD_PERMITTIVITY_VACUUM *
		d->relativePermittivity * d->area / tmp) - d->distanceOffset;
	if (d->distance < 0) {
		d->distance = 0;
	}
}

template <int N_SENSORS, int N_MEASUREMENTS_PER_SENSOR>
void CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR>::updateNCharges(uint8_t ch)
{
	CvdStruct * d;

	d = &(data[ch]);
	d->nCharges = d->nChargesNext;
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
			sample2 = sampleHalf(pos, true);
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

		sum = sample1 - sample2;
		addSample(scanOrder[pos], sum);
	}
	
	now = millis();

	for (ch = 0; ch < nSensors; ch++) {
		data[ch].lastSampledAtTime = now;
		correctSample(ch);
		processSample(ch);
	}

	for (ch = 0; ch < nSensors; ch++) {
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
