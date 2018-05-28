#include <TouchLib.h>
#include <stdlib.h>

/*
 * Touch Library tuning program
 *
 * Admar Schoonen 2016-2017
 *
 * Two types of sensors are supported: capacitive sensors and resistive sensors.
 * The library even supports to use a resistive sensor as both resistive
 * (pressure) sensor and capacitive (presence / proximity) sensor.
 *
 * To use capacitive only sensors: connect up to 16 electrodes (piece of metal
 * sheet / foil or conductive fabric) to analog pins A0 - A15 to use as
 * capacitive touch sensors or capacitive distance sensors.
 *
 * Note that if you want to use capacitive sensors, the program needs a minimum
 * of 2 capacitive sensors (only 1 will not work for technical reasons). If you
 * really only want 1 capacitive sensor, just tell the program you have 2
 * capacitive sensors and use an unused analog input for the 2nd sensor. Do not
 * connect an electrode to that input.
 *
 * To use resistive only sensors: connect up to 16 resistive pressure sensors to
 * analog pins A0 - A15. Connect the other electrode of the resistive sensors to
 * digital pins 2 - 17. You can make these sensors yourself by sandwiching a
 * piece of ESD foam or Velostat between two electrodes (metal foil or
 * conductive fabric). Connect the top electrode to an analog pin and the bottom
 * electrode to a digital pin.
 *
 * To use the resistive sensors as both resistive and capacitive sensors:
 * connect them just like you would do for resistive only sensors. In the
 * tuning program, just configure twice the number of actual sensors you have,
 * with the first half being capacitive sensors and the second half the
 * resistive sensors. The library will rapidly switch the sensor back and forth
 * between resistive sensing and capacitive sensing mode.
 *
 * Stackup of a sensor that can be used as resistive pressure sensor and
 * capacitive proximity sensor:
 *
 *   +---------------------+
 *   |  conductive fabric  *----------> to Arduino pin A0
 *  ,+=====================+.
 *  |       ESD foam        |
 *  `+=====================+'
 *   |  conductive fabric  *----------> to Arduino pin 2
 *   +---------------------+
 *
 * A second sensor would be connected to pin A1 and pin 3, a third to A2 and
 * pin 4 etc.
 *
 * The capacitive sensor part is connected to A0 (respectively A1, A2 etc). The
 * resistive sensor part is connected to A1 and pin 2 (respectively A2 / pin 3,
 * A3 / pin 4 etc).
 *
 * For ESD foam, MULTICOMP 039-0050 (sold at http://nl.farnell.com with order
 * code 1687866) is known to work very well.
 */

/* Maximum number of sensors. Depends on processor type. */
#if IS_ATMEGA16X_32X_32U4
/*
 * ATmega328P only has enough memory (RAM) for 6 sensors. ATmega168 is untested.
 * ATmega32u4 is tested up to 6 sensors.
 */
#define N_SENSORS			6
#define KNOWN_BOARD			1
#endif

#if IS_ATMEGA128X_256X
/*
 * ATmega2560 is known to be able to support up to 32 sensors. ATmega1280 is
 * untested.
 */
#define N_SENSORS			32
#define KNOWN_BOARD			1
#endif

#if IS_ATTINY
#error "TouchLib is too fat for ATtiny."
#endif

#if IS_ESP32
/*
 * ESP32 are not yet tested to how many sensors they can support. Probably a
 * lot.
 */
#define N_SENSORS			32
#define KNOWN_BOARD			1
#endif

#if IS_TEENSY3X_WITH_TOUCHREAD
/*
 * Teensies are not yet tested to how many sensors they can support. Probably a
 * lot.
 */
#define N_SENSORS			32
#define KNOWN_BOARD			1
#endif

#if IS_PARTICLE
/*
 * Particle boards all use STM32F205.
 */
#define N_SENSORS			16
#define KNOWN_BOARD			1
#endif

#ifndef KNOWN_BOARD
/* Unknown processor; assume it is capable of at least 4 sensors */
#warning "Unknown processor. Limiting number of sensors to 6."
#define N_SENSORS			6
#endif
/*
 * Number of measurements per sensor to take in one cycle. More measurements
 * means more noise reduction / spreading, but is also slower.
 */
#define N_MEASUREMENTS_PER_SENSOR	16

/* tlSensors is the actual object that contains all the sensors */
TLSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR> tlSensors;

/* Some defines */
#define PIN_TYPE_ANY		0
#define PIN_TYPE_ANALOG		1
#define PIN_TYPE_DIGITAL	2
#define PIN_TYPE_TOUCH		3

static int nSensors = N_SENSORS;

void setup()
{
	int n;

	/* Delay to make sure serial monitor receives first message */
	Serial.begin(9600);

	#if IS_ATMEGA32U4
	while(!Serial); /* Required for ATmega32u4 processors */
	#endif

	delay(500);
	Serial.println();
	Serial.println();
	Serial.println(F("Switching baudrate to 115200. Make sure to adjust "
		"baudrate in serial monitor as well!"));
	Serial.println();
	Serial.println();
	Serial.end();

	/*
	 * Switch baudrate to highest baudrate available. With higher baudrate,
	 * CPU has more time left to do capacitive sensing and thus get better
	 * signal quality.
	 */
	Serial.begin(115200);
	delay(500);
	Serial.println();
	Serial.println();

	for (n = 0; n < nSensors; n++) {
		/* By default, assume all sensors are capacitive */
		tlSensors.initialize(n, TLSampleMethodCVD);

		/* Set pin to an invalid setting */
		tlSensors.data[n].tlStructSampleMethod.CVD.pin = -1;
		tlSensors.data[n].pin =
			&(tlSensors.data[n].tlStructSampleMethod.CVD.pin);

		/*
		 * Disable state machine so we can control updating average in
		 * the main loop.
		 */
		tlSensors.data[n].enableTouchStateMachine = false;

		tlSensors.data[n].disableUpdateIfAnyButtonIsApproached = false;
		tlSensors.data[n].disableUpdateIfAnyButtonIsPressed = false;
		/* Enable noise power measurement */
		tlSensors.data[n].enableNoisePowerMeasurement = true;

		tlSensors.data[n].forceCalibrationWhenApproachingFromPressed = 0UL;
	}

	if (tlSensors.error) {
		Serial.println(F("Error detected during initialization of "
			"TouchLib. This is \nprobably a bug; please notify the"
			" author."));
		while (1);
	}

	Serial.println(F("Welcome to the TouchLib tuning program."));
	Serial.println("");
}

bool Serial_waitForFirstChar(int timeout)
{
	int t_start = millis();
	int now;
	bool charAvailable = true;

	/* Wait for first character */

	while (!Serial.available()) {
		if (timeout > 0) {
			now = millis();
			if ((now - t_start) > timeout) {
				charAvailable = false;
				break;
			}
		}
		#if IS_PARTICLE
		Particle.process();
		#endif
	}

	return charAvailable;
}

bool Serial_waitForFirstChar()
{
	return Serial_waitForFirstChar(-1);
}

char Serial_getChar()
{
	char c = '\0', tmp;

	Serial.setTimeout(10);

	Serial_waitForFirstChar();

	while (Serial.available()) {
		tmp = Serial.read();

		/* Eat CR and LF */
		if ((tmp != '\r') && (tmp != '\n')) {
			c = tmp;
		}
		#if IS_PARTICLE
		Particle.process();
		#endif
	}

	return c;
}

int Serial_getString(char * s, int length, int timeout)
{
	int pos = 0;

	if (length < 1) {
		return 0;
	}

	Serial.setTimeout(10);

	if (Serial_waitForFirstChar(timeout)) {
		/* Read 1 character less than length to reserve space for \0. */
		#if IS_PARTICLE
		while ((pos < length - 1) && (Serial.available())) {
			s[pos++] = Serial.read();
			Particle.process();
		}
		#else
		pos = Serial.readBytes(s, length - 1);
		#endif
	
		/* Force string termination */
		s[pos] = '\0';
	
		/* Strip CR and LF */
		while ((pos >= 0) && ((s[pos - 1] == '\r') || (s[pos - 1] == '\n'))) {
			s[--pos] = '\0';
		}
	}

	return pos;
}

int Serial_getString(char * s, int length)
{
	return Serial_getString(s, length, -1);
}

bool isNum(char c)
{
	return ((c >= '0') && (c <= '9'));
}

char lower(char c)
{
	if ((c >= 'A') && (c <= 'Z')) {
		c -= 32;
	}

	return c;
}

void printIntMsg(int nMin, int nMax)
{
	Serial.print(F("Invalid number. Must be between "));
	Serial.print(nMin);
	Serial.print(F(" and "));
	Serial.print(nMax);
	Serial.print(F(". "));
}

void printPinMsg(int type)
{
	switch(type) {
	case PIN_TYPE_ANY:
		Serial.print(F("Illegal pin. Enter A0 for analog 0, A1 for "
			"analog 1 etc or 0 for digital pin 0, 1 for digital pin"
			"1 etc. "));
		break;
	case PIN_TYPE_ANALOG:
		Serial.print(F("Illegal analog pin. Enter A0 for analog 0, A1 "
			"for analog 1 etc. "));
		break;
	case PIN_TYPE_DIGITAL:
		Serial.print(F("Illegal digital pin. Enter 0 for digital pin 0,"
			" 1" " for digital pin 1 etc. "));
		break;
	case PIN_TYPE_TOUCH:
		Serial.print(F("Illegal touch pin. Enter T0 for touch pin 0,"
			" T1" " for touch pin 1 etc. "));
		break;
	default:
		Serial.println(F("Error! This should never happen!"));
	}
}

int processSerialDataForNumber(int nMin, int nMax)
{
	int ret;
	char s[20] = {'\0'};
	int n, k;
	bool illegalChar;
	bool kIsNum;

	ret = nMin - 1;
	do {
		n = Serial_getString(s, sizeof(s));
		Serial.println(s);

		if (n == 0) {
		} else {
			k = 0;
			illegalChar = false;
			while (s[k] != '\0') {
				kIsNum = isNum(s[k]);
				if (k == 0) {
					if ((s[k] != '-') && (!kIsNum)) {
						illegalChar = true;
						break;
					}
				}
				if ((k > 0) && (!kIsNum)) {
					illegalChar = true;
					break;
				}
				k++;
			}

			if (illegalChar) {
				printIntMsg(nMin, nMax);
			} else {
				ret = atoi(s);
			}
			if ((ret >= nMin) && (ret <= nMax)) {
				break;
			} else {
				printIntMsg(nMin, nMax);
			}
		}
	} while (true);

	return ret;
}

int processSerialDataForPin(int type)
{
	int pin = -1;
	char s[20] = {'\0'};
	int n, k;
	bool isAnalog = false, isTouch = false, illegalChar, kIsNum;
	#if IS_ESP32
	const int pinMap[] = {T0, T1, T2, T3, T4, T5, T6, T7, T8, T9};
	#endif

	do {
		n = Serial_getString(s, sizeof(s));
		Serial.println(s);

		isAnalog = false;
		if (n == 0) {
		} else {
			k = 0;
			illegalChar = false;
			while (s[k] != '\0') {
				kIsNum = isNum(s[k]);
				if (k == 0) {
					if ((s[k] == 'A') || (s[k] == 'a')) {
						isAnalog = true;
					} else {
						if ((s[k] == 'T') || 
								(s[k] == 't')) {
							isTouch = true;
						} else if (!kIsNum) {
							illegalChar = true;
							break;
						}
					}
				}
				if ((k > 0) && (!kIsNum)) {
					illegalChar = true;
					break;
				}
				k++;
			}

			if ((illegalChar) || (isAnalog && (n == 1)) || 
					(isTouch && (n == 1))) {
				printPinMsg(type);
			} else {
				pin = -1;
				if (isAnalog && ((type == PIN_TYPE_ANALOG) ||
						(type == PIN_TYPE_ANY))) {
					pin = atoi(&s[1]) + A0;
				}
				#if IS_ESP32
				if (isTouch && (type == PIN_TYPE_TOUCH)) {
					pin = pinMap[atoi(&s[1])];
				}
				#endif
				if (!(isAnalog || isTouch) && ((type == PIN_TYPE_DIGITAL) ||
						(type == PIN_TYPE_ANY))) {
					pin = atoi(s);
				}
				if (pin >= 0) {
					break;
				} else {
					printPinMsg(type);
				}
			}
		}
	} while (true);

	return pin;
}

void floatToIntFrac(float f, int scale, int precision, int * iInt, int * iFrac)
{
	float fFrac;

	if (f > 0) {
		*iInt = scale * f;
		fFrac = scale * f - *iInt;
	} else {
		*iInt = scale * f + 1;
		fFrac = 1 - (scale * f - *iInt);
	}
	*iFrac = floor(pow(10, precision - 1) * fFrac);
}

void askNSensors(void)
{
	Serial.print(F("How many sensors do you want to tune? (1 - "));
	Serial.print(N_SENSORS);
	Serial.print(F(") "));
	nSensors = processSerialDataForNumber(1, N_SENSORS);
	Serial.println("");
	Serial.println("");

	//tlSensors.nSensors = nSensors;
}

bool askPower(int timeout)
{
	char s[20];
	bool ret = false, do_reset = false;
	int n;

	Serial.println(F("How is your system powered:"));
	Serial.println(F("a. Battery only"));
	Serial.println(F("b. Power supply WITH earth connection CONNECTED TO "
		"GND."));
	Serial.println(F("c. Power supply WITHOUT earth connection (floating "
		"ground)."));
	Serial.println(F("d. Mixed: sometimes system is powered by battery, "
		"sometimes by a suppy with floating ground."));
	Serial.println("");
	Serial.println(F("Options a and b allow to use capacitive distance "
		"sensing. Options c and d use extra filtering for more robust "
		"touch / no touch\ndetection but will not work well for "
		"distance sensing."));
	Serial.println("");

	do {
		Serial.print(F("Enter your choice (a, b, c or d): "));
		n = Serial_getString(s, sizeof(s), timeout);
		if (n > 0) {
			Serial.println(s);
			if ((n != 1) || ((s[0] != 'a') && (s[0] != 'b') &&
					(s[0] != 'c') && (s[0] != 'd') &&
					(s[0] != '~'))) {
				Serial.print(F("Illegal choice. "));
			} 
			if ((n == 1) && (s[0] == '~')) {
				do_reset = true;
				break;
			}
			if ((n == 1) && ((s[0] == 'a') || (s[0] == 'b'))) {
				ret = false; /* no need for slewrate limiter */
				break;
			}
			if ((n == 1) && ((s[0] == 'c') || (s[0] == 'd'))) {
				ret = true; /* enable slewrate limiter */
				break;
			}
		} else {
			/* Timeout occurred */
			do_reset = true;
			Serial.println("");
			break;
		}
	} while (true);

	Serial.println("");

	for (n = 0; n < nSensors; n++) {
		if (ret) {
			tlSensors.data[n].filterType =
				TLStruct::filterTypeSlewrateLimiter;
		}
	}
	
	return do_reset;
}

void askPinning(int n)
{
	int pin = 0;

	if ((tlSensors.data[n].sampleMethod == TLSampleMethodCVD) ||
		(tlSensors.data[n].sampleMethod == TLSampleMethodResistive)) {
		do {
			Serial.print(F("Which analog pin is sensor "));
			Serial.print(n);
			Serial.print(F(" connected to? "));
			pin = processSerialDataForPin(PIN_TYPE_ANALOG);
			if (pin == -1) {
				Serial.print(F("Illegal analog pin. Enter A0 "
					"for analog pin 0, A1 for analog pin 1"
					" etc. "));
			}
		} while (pin == -1);
	}
	if (tlSensors.data[n].sampleMethod == TLSampleMethodTouchRead) {
		#if IS_TEENSY3X_WITH_TOUCHREAD
		do {
			Serial.print(F("Which touch pin is sensor "));
			Serial.print(n);
			Serial.print(F(" connected to? "));
			pin = processSerialDataForPin(PIN_TYPE_DIGITAL);
			if (pin == -1) {
				Serial.print(F("Illegal pin. Enter 0 for "
				"digital pin 0, 1 for digital pin 1 etc. "));
			}
		} while (pin == -1);
		#endif
		#if IS_ESP32
		do {
			Serial.print(F("Which touch pin is sensor "));
			Serial.print(n);
			Serial.print(F(" connected to? "));
			pin = processSerialDataForPin(PIN_TYPE_TOUCH);
			if (pin == -1) {
				Serial.print(F("Illegal pin. Enter T0 for "
				"touch pin 0, T1 for touch pin 1 etc. "));
			}
		} while (pin == -1);
		#endif
	}

	if (tlSensors.data[n].sampleMethod == TLSampleMethodCVD) {
		tlSensors.data[n].tlStructSampleMethod.CVD.pin = pin;
	}

	if (tlSensors.data[n].sampleMethod == TLSampleMethodTouchRead) {
		tlSensors.data[n].tlStructSampleMethod.touchRead.pin = pin;
	}

	if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
		tlSensors.data[n].tlStructSampleMethod.resistive.pin = pin;
	}

	if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
		do {
			Serial.print(F("Which digital pin is the resistive "
				" sensor also connected to (used as ground "
				"pin)? "));
			pin = processSerialDataForPin(PIN_TYPE_DIGITAL);
			if (pin == -1) {
				Serial.print(F("Illegal pin. Enter 0 for "
				"digital pin 0, 1 for digital pin 1 etc. "));
			}
		} while (pin == -1);

		tlSensors.data[n].tlStructSampleMethod.resistive.gndPin = pin;
	}
}

#if ((IS_TEENSY3X) || (IS_ESP32))
#define ASK_SAMPLE_METHOD_DEFINED 1
bool askSampleMethod(int n)
{
	/* Disabled CVD for Teensy as it is not yet very reliable */
	char c;

	Serial.println("");
	Serial.print(F("Is sensor "));
	Serial.print(n);
	Serial.println(F(" a capacitive sensor using CVD method, a capacitive "
		"sensor using touchRead() method or a resistive sensor using\n"
		"analogRead() method? "));
	//Serial.println(F(" a capacitive sensor using touchRead() method or a "
	//	"resistive sensor using analogRead() method? "));
	do {
		Serial.print(F("Enter c for capacitive using CVD, t for "
		"capacitive using touchRead() or r for resistive: "));
		//Serial.print(F("Enter t for capacitive using touchRead() or r "
		//	"for resistive: "));
		c = Serial_getChar();
		Serial.println(c);
		if ((lower(c) != 'c') && (lower(c) != 't') && (lower(c) != 'r')) {
		//if ((lower(c) != 't') && (lower(c) != 'r')) {
			Serial.print(F("Illegal sensor type. "));
		} else {
			if (lower(c) == 'c') {
				tlSensors.initialize(n, TLSampleMethodCVD);
			}
			if (lower(c) == 't') {
				tlSensors.initialize(n, TLSampleMethodTouchRead);
			}
			if (lower(c) == 'r') {
				tlSensors.initialize(n, TLSampleMethodResistive);
				tlSensors.data[n].filterType =
					TLStruct::filterTypeAverage;
			}
			break;
		}
	} while (true);

	return false;
}
#endif

#if ((IS_AVR) || (IS_PARTICLE))
#define ASK_SAMPLE_METHOD_DEFINED 1
bool askSampleMethod(int n)
{
	char c;

	Serial.println("");
	Serial.print(F("Is sensor "));
	Serial.print(n);
	Serial.print(F(" a capacitive sensor using CVD method or a resistive "
		"sensor using analogRead() method?\n"));
	do {
		Serial.print(F("Enter c for capacitive using CVD or r for "
			"resistive: "));
		c = Serial_getChar();
		Serial.println(c);
		if ((lower(c) != 'c') && (lower(c) != 'r')) {
			Serial.print(F("Illegal sensor type. "));
		} else {
			if (lower(c) == 'c') {
				tlSensors.initialize(n, TLSampleMethodCVD);
			}
			if (lower(c) == 'r') {
				tlSensors.initialize(n, TLSampleMethodResistive);
				tlSensors.data[n].filterType =
					TLStruct::filterTypeAverage;
			}
			break;
		}
	} while (true);

	return false;
}
#endif

#ifndef ASK_SAMPLE_METHOD_DEFINED
#error askSampleMethod() is not defined. This is a bug. Please report to the author.
#endif

void noiseTuning(void)
{
	char c;
	bool highNoise = false;
	uint8_t n;

	Serial.print(F("Performing noise measurement for all sensors. "));
	do {
		Serial.print(F("Make sure to not touch any sensor. Enter y to "
			"start the noise measurement. "));
		c = Serial_getChar();
		Serial.println(c);
		if (lower(c) != 'y') {
			Serial.print(F("Illegal choice. Only y is allowed. "));
		}
	} while (lower(c) != 'y');

	Serial.println(F("Noise measurement started... "));
	for (n = 0; n < nSensors; n++) {
		tlSensors.data[n].enableTouchStateMachine = false;
		tlSensors.data[n].enableNoisePowerMeasurement = true;
		tlSensors.setState(n, TLStruct::buttonStatePreCalibrating);
	}
	while (tlSensors.getState(0) != TLStruct::buttonStateReleased) {
		tlSensors.sample();
	}
	Serial.println(F("Noise measurement finished. Found the following "
		"thresholds:"));

	for (n = 0; n < nSensors; n++) {
		highNoise = false;
	
		/*
		 * Disabled adjusting thresholds based on noise. Seems to be not
		 * very reliable.
		 */
		#if (1)
		if (tlSensors.data[n].releasedToApproachedThreshold <
				3 * sqrt(tlSensors.data[n].noisePower)) {
			tlSensors.data[n].releasedToApproachedThreshold =
				3 * sqrt(tlSensors.data[n].noisePower);
			highNoise = true;
		}
	
		if (tlSensors.data[n].approachedToReleasedThreshold <
				(9 * 3 * sqrt(tlSensors.data[n].noisePower) +
				5) / 10) {
			tlSensors.data[n].approachedToReleasedThreshold =
				(9 * 3 * sqrt(tlSensors.data[n].noisePower) +
				5) / 10;
			highNoise = true;
		}
		#endif
	
		Serial.print(F("  Sensor "));
		Serial.print(n);
		Serial.println(F(":"));
		/*if (highNoise) {
			Serial.print(F("    Warning! High noise levels detected! "
				"Increased released to approached and approached to "
				"released thresholds for sensor "));
			Serial.print(n);
			Serial.println("!");
		}*/
	
		Serial.print(F("    released -> approached: "));
		Serial.println(tlSensors.data[n].releasedToApproachedThreshold);
		Serial.print(F("    approached -> released: "));
		Serial.println(tlSensors.data[n].approachedToReleasedThreshold);
		Serial.println("");
	}

	Serial.println("");
}

#define N_MEASUREMENTS 10
bool touchTuning(int n)
{
	char c;
	int32_t delta[N_MEASUREMENTS];
	int32_t avg = 0;
	int32_t min_val;
	int k;
	bool done = false;
	bool isTuned = false;

	Serial.print(F("Performing touch measurement for "));
	if (tlSensors.data[n].sampleMethod == TLSampleMethodCVD) {
		Serial.print(F("capacitive (CVD method)"));
	}
	if (tlSensors.data[n].sampleMethod == TLSampleMethodTouchRead) {
		Serial.print(F("capacitive (touchRead() method)"));
	}
	if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
		Serial.print(F("resistive (analogRead() method)"));
	}
	Serial.print(F(" sensor "));
	Serial.print(n);
	Serial.println(F(". "));

	do {
		do {
			Serial.print(F("Make sure to touch and hold the sensor,"
				" then press y to start the touch\n"
				"measurement or press s to skip tuning this "
				"sensor."));
			c = Serial_getChar();
			Serial.println(c);
			if ((lower(c) != 'y') && (lower(c) != 's')) {
				Serial.println(F("Illegal choice. Only y or s "
					"are allowed. "));
			}
		} while ((lower(c) != 'y') && (lower(c) != 's'));

		if (lower(c) == 's') {
			done = true;
			tlSensors.data[n].disableSensor = true;
			break;
		}
	
		tlSensors.data[n].enableTouchStateMachine = false;
		tlSensors.setState(n, TLStruct::buttonStatePressed);
	
		for (k = 0; k < N_MEASUREMENTS; k++) {
			tlSensors.sample();
			delta[k] = tlSensors.data[n].delta;
			avg += delta[k];
			if (k == 0) {
				min_val = delta[k];
			} else {
				min_val = (delta[k] < min_val) ? delta[k] :
					min_val;
			}
		}
		avg = avg / N_MEASUREMENTS;
		
		tlSensors.data[n].approachedToPressedThreshold = (avg >> 1);
		tlSensors.data[n].pressedToApproachedThreshold = (9 *
			tlSensors.data[n].approachedToPressedThreshold + 5) / 10;

		if (tlSensors.data[n].pressedToApproachedThreshold < ((11 *
				tlSensors.data[n].releasedToApproachedThreshold
				+ 5) / 10)) {
			Serial.print(F("Error! Detected signal is too low: "));
			Serial.print(tlSensors.data[n].pressedToApproachedThreshold);
			Serial.print(F(" < "));
			Serial.print((11 *
				tlSensors.data[n].releasedToApproachedThreshold
				+ 5) / 10);
			Serial.print(". ");
		} else {
			done = true;
			isTuned = true;
			tlSensors.data[n].disableSensor = false;
		}
	} while (!done);

	if (isTuned) {
		Serial.println(F("Found the following thresholds:"));
		Serial.print(F("  approached -> pressed: "));
		Serial.println(tlSensors.data[n].approachedToPressedThreshold);
		Serial.print(F("  pressed -> approached: "));
		Serial.println(tlSensors.data[n].pressedToApproachedThreshold);
	} else {
		/* 
		 * Skip tuning for this sensor; instead set some sensible
		 * defaults
		 */
		tlSensors.data[n].approachedToPressedThreshold =
			tlSensors.data[n].releasedToApproachedThreshold * 10;
		tlSensors.data[n].pressedToApproachedThreshold = (9 *
			tlSensors.data[n].approachedToPressedThreshold + 5) / 10;
		tlSensors.data[n].calibratedMaxDelta = (11 *
			tlSensors.data[n].approachedToPressedThreshold + 5) / 10;
		Serial.print(F("\nSkipped tuning sensor "));
		Serial.print(n);
	}
	Serial.println("");

	return isTuned;
}

void maxTouchTuning(int n)
{
	char c;
	int32_t maxDelta[N_MEASUREMENTS];
	int32_t min_val;
	int k;
	bool done = false;

	Serial.print(F("Performing maximum range measurement for "));
	if (tlSensors.data[n].sampleMethod == TLSampleMethodCVD) {
		Serial.print(F("capacitive (CVD method)"));
	}
	if (tlSensors.data[n].sampleMethod == TLSampleMethodTouchRead) {
		Serial.print(F("capacitive (touchRead() method)"));
	}
	if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
		Serial.print(F("resistive (analogRead() method)"));
	}
	Serial.print(F(" sensor "));
	Serial.print(n);
	Serial.print(". ");

	do {
		do {
			if (tlSensors.data[n].sampleMethod == 
					TLSampleMethodCVD) {
				Serial.print(F("Make sure to cover and hold "
					"the sensor with as many fingers as "
					"will\nfit or with your whole hand, then"
					" press y to start the touch "
					"measurement. "));
			}
			if (tlSensors.data[n].sampleMethod == 
					TLSampleMethodResistive) {
				Serial.print(F("Make sure to press and hold the"
					"sensor as firmly as possible, then\n"
					"press y to start the touch "
					"measurement. "));
			}
			c = Serial_getChar();
			Serial.println(c);
			if (lower(c) != 'y') {
				Serial.print(F("Illegal choice. Only y is "
					"allowed. "));
			}
		} while (lower(c) != 'y');
	
		tlSensors.data[n].enableTouchStateMachine = false;
		tlSensors.setState(n, TLStruct::buttonStatePressed);
	
		for (k = 0; k < N_MEASUREMENTS; k++) {
			tlSensors.sample();
			maxDelta[k] = tlSensors.data[n].maxDelta;
			if (k == 0) {
				min_val = maxDelta[k];
			} else {
				min_val = (maxDelta[k] < min_val) ? maxDelta[k] :
					min_val;
			}
		}
		
		/*
		 * Allow maxDelta to be equal or even smaller than approached to
		 * pressed threshold. We don't really care much for maxDelta
		 * anyway.
		 */

		if (maxDelta[N_MEASUREMENTS - 1] < 
				tlSensors.data[n].approachedToPressedThreshold) {
			tlSensors.data[n].calibratedMaxDelta = 
				tlSensors.data[n].approachedToPressedThreshold;
		} else {
			tlSensors.data[n].calibratedMaxDelta =
				maxDelta[N_MEASUREMENTS - 1];
		}

		done = true;
	} while (!done);

	Serial.print(F("Found the following maximum: "));
	Serial.println(maxDelta[N_MEASUREMENTS - 1]);
	Serial.println("");
}

int countNSensors(int (*sampleMethod)(struct TLStruct * d, uint8_t nSensors,
		uint8_t ch))
{
	int k, n;

	k = 0;

	for (n = 0; n < nSensors; n++) {
		if (tlSensors.data[n].sampleMethod == sampleMethod) {
			k++;
		}
	}

	return k;
}

void checkResistiveCapacitiveSensor(int n)
{
	int k;
	int cap_sensor = -1, res_sensor = -1;
	int res_pin = -1, cap_pin = -1;

	k = tlSensors.findSensorPair(n, (n + 1) % nSensors);

	if (k >= 0) {
		if (tlSensors.data[k].sampleMethod == TLSampleMethodResistive) {
			res_sensor = k;
			res_pin = tlSensors.data[k].tlStructSampleMethod.resistive.pin;
		}

		if (tlSensors.data[k].sampleMethod == TLSampleMethodCVD) {
			cap_sensor = k;
			cap_pin = tlSensors.data[k].tlStructSampleMethod.CVD.pin;
		}
	}

	if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
		res_sensor = n;
		res_pin = tlSensors.data[n].tlStructSampleMethod.resistive.pin;
	}

	if (tlSensors.data[n].sampleMethod == TLSampleMethodCVD) {
		cap_sensor = n;
		cap_pin = tlSensors.data[n].tlStructSampleMethod.CVD.pin;
	}

	if (res_pin == cap_pin) {
		Serial.print(F("Capacitive sensor "));
		Serial.print(cap_sensor);
		Serial.print(F(" and resistive sensor "));
		Serial.print(res_sensor);
		Serial.println(F(" use the same analog pin. I will enforce "
			"that the resistive sensor is automatically "
			"recalibrated when the capacitive sensor is released."));

		tlSensors.data[cap_sensor].forceCalibrationWhenApproachingFromPressed |=
	                (1UL << res_sensor);
	}
}

void printCode(void)
{
	/*
	 * Yo dawg, I heard you like code, so I put code in your code so you can
	 * program while you program.
	 */

	char c;
	int n, pin;

	do {
		Serial.print(F("Tuning has finished. Press y to get copy/paste "
			"code to use in your project. "));
		c = Serial_getChar();
		Serial.println(c);
		if (lower(c) != 'y') {
			Serial.print(F("Illegal choice. Only y is "
				"allowed. "));
		}
	} while (lower(c) != 'y');

	Serial.print(F("\n"));
	Serial.print(F("********************************************************************************\n"));
	Serial.print(F("\n"));

	Serial.print(F("#include <TouchLib.h>\n"));
	Serial.print(F("\n"));
	Serial.print(F("/*\n"));
	Serial.print(F(" * Code generated by TouchLib SemiAutoTuning.\n"));
	Serial.print(F(" *\n"));
	Serial.print(F(" * Hardware configuration:\n"));
	for (n = 0; n < nSensors; n++) {
		Serial.print(F(" *   sensor "));
		Serial.print(n);
		Serial.print(F(": type: "));
		if (tlSensors.data[n].sampleMethod == TLSampleMethodCVD) {
			Serial.print(F("capacitive (CVD method), analog pin A"));
			pin = tlSensors.data[n].tlStructSampleMethod.CVD.pin;
			Serial.print(pin - A0);
		}
		if (tlSensors.data[n].sampleMethod == TLSampleMethodTouchRead) {
			Serial.print(F("capacitive (touchRead()) method), pin "));
			pin = tlSensors.data[n].tlStructSampleMethod.touchRead.pin;
			Serial.print(pin);
		}
		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
			Serial.print(F("resistive (analogRead() method),  "
				"analog pin A"));
			pin = tlSensors.data[n].tlStructSampleMethod.resistive.pin;
			Serial.print(pin - A0);
			Serial.print(F(", ground pin: "));
			Serial.print(tlSensors.data[n].tlStructSampleMethod.resistive.gndPin);
		}
		Serial.print(F("\n"));
	}
	Serial.print(F(" */\n"));
	Serial.print(F("\n"));

	Serial.print(F("/*\n"));
	Serial.print(F(" *  Number of sensors. For capacitive sensors: needs to"
		" be a minimum of 2. When\n"));
	Serial.print(F(" * using only one sensor, set N_SENSORS to 2 and use an"
		" unused analog input pin for the second\n"));
	Serial.print(F(" * sensor. For 2 or more sensors you don't need to add"
		" an unused analog input.\n"));
 	Serial.print(F(" */\n"));
	Serial.print(F("#define N_SENSORS                       "));
	Serial.print(nSensors);
	Serial.print(F("\n"));
	Serial.print(F("\n"));

	Serial.print(F("/*\n"));
	Serial.print(F(" * Number of measurements per sensor to take in one "
		"cycle. More measurements\n"));
	Serial.print(F(" * means more noise reduction / spreading, but is also "
		"slower.\n"));
	Serial.print(F(" */\n"));
	Serial.print(F("#define N_MEASUREMENTS_PER_SENSOR       16\n"));
	Serial.print(F("\n"));

	Serial.print(F("/* tlSensors is the actual object that contains all the"
		" sensors */\n"));
	Serial.print(F("TLSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR> "
		"tlSensors;\n"));
	Serial.print(F("\n"));

	#if IS_PARTICLE
	Serial.print(F("/*\n"));
	Serial.print(F(" * Declaration of all the web variables. \n"));
	Serial.print(F(" *\n"));
	Serial.print(F(" * For every sensor there are 2 variables:\n"));
	Serial.print(F(" * double snsrX_delta:      difference between "
		"current value and average\n"));
	Serial.print(F(" * int    snsrX_press:      0 if not pressed, 1 if "
		"pressed\n"));
	Serial.print(F(" *\n"));
	Serial.print(F(" * The variable state can have the following values:\n"));
	Serial.print(F(" *   0: PreCalibrating\n"));
	Serial.print(F(" *   1: Calibrating\n"));
	Serial.print(F(" *   2: NoisePowerMeasurement\n"));
	Serial.print(F(" *   3: Released\n"));
	Serial.print(F(" *   4: ReleasedToApproached\n"));
	Serial.print(F(" *   5: Approached\n"));
	Serial.print(F(" *   6: ApproachedToPressed\n"));
	Serial.print(F(" *   7: ApproachedToReleased\n"));
	Serial.print(F(" *   8: Pressed\n"));
	Serial.print(F(" *   9: PressedToApproached\n"));
	Serial.print(F(" */\n\n"));
	
        for (n = 0; n < nSensors; n++) {
		if (tlSensors.data[n].disableSensor == false) {
			Serial.print(F("double snsr"));
			Serial.print(n);
			Serial.print(F("_delta = 0;\n"));
			Serial.print(F("int    snsr"));
			Serial.print(n);
			Serial.print(F("_press = 0;\n"));
		}
	}
	#endif

	Serial.print(F("void setup()\n"));
	Serial.print(F("{\n"));

	#if IS_PARTICLE
        for (n = 0; n < nSensors; n++) {
		if (tlSensors.data[n].disableSensor == false) {
			Serial.print(F("        Particle.variable(\"snsr"));
			Serial.print(n);
			Serial.print(F("_delta\", snsr"));
			Serial.print(n);
			Serial.print(F("_delta);\n"));
		}
	}
	#endif

        Serial.print(F("        Serial.begin(9600);\n"));
	Serial.print(F("\n"));
	Serial.print(F("        #if IS_ATMEGA32U4\n"));
	Serial.print(F("        while(!Serial); /* Required for ATmega32u4 "
		"processors */\n"));
	Serial.print(F("        #endif\n"));
	Serial.print(F("\n"));
	Serial.print(F("        /* Delay to make sure serial monitor receives "
		"first message */\n"));
        Serial.print(F("        delay(500);\n"));
        Serial.print(F("        Serial.println();\n"));
        Serial.print(F("        Serial.println();\n"));
	Serial.print(F("        Serial.println(\"Switching baudrate to 115200. "
		"Make sure to adjust baudrate in serial monitor as "
		"well!\");\n"));
        Serial.print(F("        Serial.println();\n"));
        Serial.print(F("        Serial.println();\n"));
        Serial.print(F("        Serial.end();\n"));
	Serial.print(F("\n"));
        Serial.print(F("        /*\n"));
        Serial.print(F("         * Switch baudrate to highest baudrate "
		"available. With higher baudrate,\n"));
	Serial.print(F("         * CPU has more time left to do capacitive "
		"sensing and thus get better\n"));
	Serial.print(F("         * signal quality.\n"));
        Serial.print(F("         */\n"));
        Serial.print(F("        Serial.begin(115200);\n"));
        Serial.print(F("        delay(500);\n"));
        Serial.print(F("        Serial.println();\n"));
        Serial.print(F("        Serial.println();\n"));

        for (n = 0; n < nSensors; n++) {
		Serial.print(F("\n"));
		Serial.print(F("        /*\n"));
		Serial.print(F("         * Configuration for sensor "));
		Serial.print(n);
		Serial.print(F(":\n"));
		if (tlSensors.data[n].sampleMethod == TLSampleMethodCVD) {
			Serial.print(F("         * Type: capacitive (CVD "
				"method)\n"));
			Serial.print(F("         * Analog pin: A"));
			pin = tlSensors.data[n].tlStructSampleMethod.CVD.pin;
			Serial.print(pin - A0);
		}
		if (tlSensors.data[n].sampleMethod == TLSampleMethodTouchRead) {
			Serial.print(F("         * Type: capacitive "
				"(touchRead() method)\n"));
			Serial.print(F("         * Pin: "));
			pin = tlSensors.data[n].tlStructSampleMethod.touchRead.pin;
			Serial.print(pin);
		}
		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
			Serial.print(F("         * Type: resistive "
				"(analogRead() method)\n"));
			Serial.print(F("         * Analog pin: A"));
			pin = tlSensors.data[n].tlStructSampleMethod.resistive.pin;
			Serial.print(pin - A0);
		}
		Serial.print(F("\n"));
		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
			Serial.print(F("         * Ground pin: "));
			Serial.print(tlSensors.data[n].tlStructSampleMethod.resistive.gndPin);
			Serial.print(F("\n"));
		}
		Serial.print(F("         */\n"));
		Serial.print(F("        tlSensors.initialize("));
		Serial.print(n);
		if (tlSensors.data[n].sampleMethod == TLSampleMethodCVD) {
			Serial.print(F(", TLSampleMethodCVD);\n"));
		}
		if (tlSensors.data[n].sampleMethod == TLSampleMethodTouchRead) {
			Serial.print(F(", TLSampleMethodTouchRead);\n"));
		}
		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
			Serial.print(F(", TLSampleMethodResistive);\n"));
		}
		Serial.print(F("        tlSensors.data["));
		Serial.print(n);
		Serial.print(F("].tlStructSampleMethod."));
		if (tlSensors.data[n].sampleMethod == TLSampleMethodCVD) {
			Serial.print(F("CVD.pin =               A"));
			pin = tlSensors.data[n].tlStructSampleMethod.CVD.pin;
			Serial.print(pin - A0);
		}
		if (tlSensors.data[n].sampleMethod == TLSampleMethodTouchRead) {
			Serial.print(F("touchRead.pin =         "));
			pin = tlSensors.data[n].tlStructSampleMethod.touchRead.pin;
			Serial.print(pin);
		}
		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
			Serial.print(F("resistive.pin =         A"));
			pin = tlSensors.data[n].tlStructSampleMethod.resistive.pin;
			Serial.print(pin - A0);
		}
		Serial.print(F(";\n"));
		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
			Serial.print(F("        tlSensors.data["));
			Serial.print(n);
			Serial.print(F("].tlStructSampleMethod.resistive.gndPin"
				" =      "));
			Serial.print(tlSensors.data[n].tlStructSampleMethod.resistive.gndPin);
			Serial.print(F(";\n"));
		}
		Serial.print(F("        tlSensors.data["));
		Serial.print(n);
		Serial.print(F("].releasedToApproachedThreshold ="
			"              "));
		Serial.print(tlSensors.data[n].releasedToApproachedThreshold);
		Serial.print(F(";\n"));
		Serial.print(F("        tlSensors.data["));
		Serial.print(n);
		Serial.print(F("].approachedToReleasedThreshold ="
			"              "));
		Serial.print(tlSensors.data[n].approachedToReleasedThreshold);
		Serial.print(F(";\n"));
		Serial.print(F("        tlSensors.data["));
		Serial.print(n);
		Serial.print(F("].approachedToPressedThreshold ="
			"               "));
		Serial.print(tlSensors.data[n].approachedToPressedThreshold);
		Serial.print(F(";\n"));
		Serial.print(F("        tlSensors.data["));
		Serial.print(n);
		Serial.print(F("].pressedToApproachedThreshold ="
			"               "));
		Serial.print(tlSensors.data[n].pressedToApproachedThreshold);
		Serial.print(F(";\n"));
		Serial.print(F("        tlSensors.data["));
		Serial.print(n);
		Serial.print(F("].calibratedMaxDelta ="
			"                         "));
		Serial.print(tlSensors.data[n].calibratedMaxDelta);
		Serial.print(F(";\n"));
		Serial.print(F("        tlSensors.data["));
		Serial.print(n);
		Serial.print(F("].filterType = TLStruct::filterType"));
		switch (tlSensors.data[n].filterType) {
		case TLStruct::filterTypeAverage:
			Serial.print(F("Average;\n"));
			break;
		case TLStruct::filterTypeSlewrateLimiter:
			Serial.print(F("SlewrateLimiter;\n"));
			break;
		case TLStruct::filterTypeMedian:
			Serial.print(F("Median;\n"));
			break;
		default:
			/* Error? */
			Serial.print(F("Average;\n"));
			break;
		}

		if (tlSensors.data[n].forceCalibrationWhenApproachingFromPressed) {
			Serial.print(F("        tlSensors.data["));
			Serial.print(n);
			Serial.print(F("].forceCalibrationWhenApproachingFromPressed"
				" = 0x"));
			Serial.print(tlSensors.data[n].forceCalibrationWhenApproachingFromPressed,
				HEX);
			Serial.print(F(";\n"));
		}
	}

	Serial.print(F("\n"));
	Serial.print(F("        if (tlSensors.error) {\n"));
	Serial.print(F("                Serial.println(\"Error detected during "
		"initialization of TouchLib. This is \"\n"));
	Serial.print(F("                       \"probably a bug; please notify the author.\");\n"));
	Serial.print(F("                while (1);\n"));
	Serial.print(F("        }\n"));
	Serial.print(F("\n"));
	Serial.print(F("        Serial.println(\"Calibrating sensors...\");\n"));
	Serial.print(F("        while(tlSensors.anyButtonIsCalibrating()) {\n"));
	Serial.print(F("                tlSensors.sample();\n"));
	Serial.print(F("        }\n"));
	Serial.print(F("        Serial.println(\"Calibration done...\");\n"));
	Serial.print(F("}\n"));

	Serial.print(F("\n"));
	Serial.print(F("void print_sensor_state(int n)\n"));
	Serial.print(F("{\n"));
	Serial.print(F("        char s[32] = {'\\0'};\n"));
	#if IS_PARTICLE
	Serial.print(F("        char sInt[12] = {'\\0'};\n"));
	#endif
	Serial.print(F("\n"));
	Serial.print(F("        Serial.print(\" #\");\n"));
	Serial.print(F("        Serial.print(n);\n"));
	Serial.print(F("        Serial.print(\": \");\n"));
	Serial.print(F("        Serial.print(tlSensors.isCalibrating(n));\n"));
	Serial.print(F("        Serial.print(\" \");\n"));
	Serial.print(F("        Serial.print(tlSensors.isReleased(n));\n"));
	Serial.print(F("        Serial.print(\" \");\n"));
	Serial.print(F("        Serial.print(tlSensors.isApproached(n));\n"));
	Serial.print(F("        Serial.print(\" \");\n"));
	Serial.print(F("        Serial.print(tlSensors.isPressed(n));\n"));
	Serial.print(F("        Serial.print(\" \");\n"));
	Serial.print(F("        Serial.print(tlSensors.getState(n));\n"));
	Serial.print(F("        Serial.print(\" \");\n"));
	Serial.print(F("        Serial.print(tlSensors.getStateLabel(n));\n"));
	Serial.print(F("        memset(s, '\\0', sizeof(s));\n"));
	Serial.print(F("        memset(s, ' ', 22 - "
			"strlen(tlSensors.getStateLabel(n)));\n"));
	Serial.print(F("        Serial.print(s);\n"));

	#if IS_PARTICLE
	Serial.println("");
        for (n = 0; n < nSensors; n++) {
		if (tlSensors.data[n].disableSensor == false) {
			Serial.print(F("        if (snsr"));
			Serial.print(n);
			Serial.print(F("_press != tlSensors.isPressed("));
			Serial.print(n);
			Serial.print(F(")) {\n"));
			
			Serial.print(F("                snsr"));
			Serial.print(n);
			Serial.print(F("_press = tlSensors.isPressed("));
			Serial.print(n);
			Serial.print(F(");\n"));
	
			Serial.print(F("                snprintf(sInt, sizeof(sInt) - 1, "
				"\"%d\", snsr"));
			Serial.print(n);
			Serial.print(F("_press);\n"));
			Serial.print(F("                Particle.publish(\"snsr"));
			Serial.print(n);
			Serial.print(F("_press\", sInt);\n"));
			Serial.print(F("        }\n"));

			Serial.print(F("                snsr"));
			Serial.print(n);
			Serial.print(F("_delta = tlSensors.getDelta("));
			Serial.print(n);
			Serial.print(F(");\n"));
		}
	}
	#endif

	Serial.print(F("}\n"));
	Serial.print(F("\n"));
	Serial.print(F("#define BAR_LENGTH                     58 /* <-- Change "
		"this to print longer or shorter visualizations */\n"));
	Serial.print(F("\n"));
	Serial.print(F("void loop(void)\n"));
	Serial.print(F("{\n"));
	Serial.print(F("        int k;\n"));
	Serial.print(F("\n"));
	Serial.print(F("        int n = 0; /* <-- Change this number to view a "
		"different sensor */\n"));
	Serial.print(F("\n"));
	Serial.print(F("        tlSensors.sample(); /* <-- Take a series of new "
		"samples for all sensors */\n"));
	Serial.print(F("\n"));
	Serial.print(F("        tlSensors.printBar(n, BAR_LENGTH); /* <-- Print "
		"the visualization */\n"));
	Serial.print(F("\n"));
	Serial.print(F("        print_sensor_state(n); /* <-- Print summary of "
		"sensor n */\n"));
	Serial.print(F("\n"));
	Serial.print(F("        /* For capacitive + resistive sensors: k is the "
		"sensor that sensor n is paired with */\n"));
	Serial.print(F("        k = tlSensors.findSensorPair(n, (n + 1) % "
		"N_SENSORS);\n"));
	Serial.print(F("        if (k > -1) {\n"));
	Serial.print(F("                print_sensor_state(k); /* <-- Print "
		"summary of sensor k */\n"));
	Serial.print(F("        }\n"));
	Serial.print(F("        Serial.println(\"\");\n"));
	Serial.print(F("}\n"));

	Serial.print(F("\n"));
	Serial.print(F("********************************************************************************\n"));
}

void loop()
{
	int n;

	/*
	 * Ugly hack for boards that do not reset upon opening serial monitor:
	 * answer '~' on first question to force displaying the messages from
	 * setup and askPower() again.
	 */
	while (askPower(10000) == true);

	askNSensors();
	Serial.println(F("First we need to know what type of sensors the system"
		" has and to which pins they are connected."));
	for (n = 0; n < nSensors; n++) {
		askSampleMethod(n);
		askPinning(n);
	}
	Serial.println("");

	Serial.println();
	for (n = 0; n < nSensors; n++) {
		Serial.print("sensor ");
		Serial.print(n);
		Serial.print(": CVD pin: ");
		Serial.println(tlSensors.data[n].tlStructSampleMethod.CVD.pin);
	}

	n = countNSensors(TLSampleMethodCVD);
	if (n == 1) {
		Serial.println(F("Error! Detected only 1 capacitive sensor "
			"using CVD method. CVD method can only work if you have"
			" at least 2 capacitive sensors with CVD method. If you"
			" only need 1 sensor, just add a 2nd one but do not "
			"connect it to a touchpad."));
		Serial.println();
		Serial.println(F("TouchLib tuning program aborted."));
		while (true);
	}

	Serial.println(F("Next step is to tune the sensors."));
	noiseTuning();
	Serial.println();
	for (n = 0; n < nSensors; n++) {
		Serial.print("sensor ");
		Serial.print(n);
		Serial.print(": CVD pin: ");
		Serial.println(tlSensors.data[n].tlStructSampleMethod.CVD.pin);
	}
	
	for (n = 0; n < nSensors; n++) {
		if (touchTuning(n)) {
			maxTouchTuning(n);
			Serial.print(F("Finished tuning sensor "));
			Serial.println(n);
			Serial.println("");
		}
		Serial.println("");
	}

	/*
	 * Disabled forced calibration of resistive sensors. Seems not to be
	 * needed.
	 */
	#if (0)
	for (n = 0; n < nSensors; n++) {
		checkResistiveCapacitiveSensor(n);
	}
	#endif
	Serial.println("");
	Serial.println("");

	printCode();
	Serial.println("");
	Serial.println(F("TouchLib tuning program finished"));
	while (true) {
		#if IS_PARTICLE
		Particle.process();
		#endif
	}
}
