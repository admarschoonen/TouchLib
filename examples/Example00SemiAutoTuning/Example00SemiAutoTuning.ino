#include <TouchLib.h>
#include <stdlib.h>

/*
 * Touch Library Demo Sketch
 * Admar Schoonen 2016
 * Connect 4 electrodes (piece of metal sheet / foil) to analog pins A0 - A3
 */

/*
 * Number of capacitive sensors. Needs a minimum of 2. When using only one
 * sensor, set N_SENSORS to 2 and use an unused analog input pin for the second
 * sensor. For 2 or more sensors you don't need to add an unused analog input.
 */
#define N_SENSORS			16

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

static int nSensors = N_SENSORS;

void setup()
{
	int n;

	/* Delay to make sure serial monitor receives first message */
	Serial.begin(9600);
	delay(500);
	Serial.println();
	Serial.println();
	Serial.println("Switching baudrate to 115200. Make sure to adjust "
		"baudrate in serial monitor as well!");
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

	Serial.println("Welcome to the TouchLib tuning program.");
	Serial.println("");

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

		/* Enable noise power measurement */
		tlSensors.data[n].enableNoisePowerMeasurement = true;

		tlSensors.data[n].forceCalibrationWhenApproachingFromPressed = 0UL;
	}
}

char Serial_getChar()
{
	char c;

	Serial.setTimeout(10);

	while (!Serial.available()); /* Wait for first character */

	c = Serial.read();

	return c;
}

int Serial_getString(char * s, int length)
{
	int pos = 0;

	if (length < 1) {
		return 0;
	}

	Serial.setTimeout(10);

	while (!Serial.available()); /* Wait for first character */

	pos = Serial.readBytes(s, length);

	s[pos] = '\0';

	return pos;
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
	Serial.print("Invalid number. Must be between ");
	Serial.print(nMin);
	Serial.print(" and ");
	Serial.print(nMax);
	Serial.print(". ");
}

void printPinMsg(int type)
{
	switch(type) {
	case PIN_TYPE_ANY:
		Serial.print("Illegal pin. Enter A0 for analog 0, A1 for analog"
			" 1 etc or 0 for digital pin 0, 1 for digital pin 1 "
			"etc. ");
		break;
	case PIN_TYPE_ANALOG:
		Serial.print("Illegal analog pin. Enter A0 for analog 0, A1 for"
			" analog 1 etc. ");
		break;
	case PIN_TYPE_DIGITAL:
		Serial.print("Illegal digital pin. Enter 0 for digital pin 0, 1"
			" for digital pin 1 etc. ");
		break;
	default:
		Serial.println("Error! This should never happen!");
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
	bool isAnalog = false, illegalChar, kIsNum;

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
					} else if (!kIsNum) {
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

			if ((illegalChar) || (isAnalog && (n == 1))) {
				printPinMsg(type);
			} else {
				pin = -1;
				if (isAnalog && ((type == PIN_TYPE_ANALOG) ||
						(type == PIN_TYPE_ANY))) {
					pin = atoi(&s[1]) + A0;
				}
				if (!isAnalog && ((type == PIN_TYPE_DIGITAL) ||
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
	Serial.print("How many sensors do you want to tune? (1 - ");
	Serial.print(N_SENSORS);
	Serial.print(") ");
	nSensors = processSerialDataForNumber(1, N_SENSORS);
	Serial.println("");
	Serial.println("");
}

bool askPower(void)
{
	char s[20];
	bool ret;
	int n;

	Serial.println("How is your system powered:");
	Serial.println("a. Battery only");
	Serial.println("b. Power supply WITH earth connection CONNECTED TO "
		"GND.");
	Serial.println("c. Power supply WITHOUT earth connection (floating "
		"ground).");
	Serial.println("d. Mixed: sometimes system is powered by battery, "
		"sometimes by a suppy with floating ground.");
	Serial.println("");

	do {
		Serial.print("Enter your choice (a, b, c or d): ");
		n = Serial_getString(s, sizeof(s));
		Serial.println(s);
		if ((n != 1) || ((s[0] != 'a') && (s[0] != 'b') &&
				(s[0] != 'c') && (s[0] != 'd'))) {
			Serial.print("Illegal choice. ");
		} 
		if ((n == 1) && ((s[0] == 'a') || (s[0] == 'b'))) {
			ret = false; /* no need for slewrate limiter */
			break;
		}
		if ((n == 1) && ((s[0] == 'c') || (s[0] == 'd'))) {
			ret = true; /* enable slewrate limiter */
			break;
		}
	} while (true);

	Serial.println("");

	for (n = 0; n < nSensors; n++) {
		tlSensors.data[n].enableSlewrateLimiter = ret;
	}
	
	return ret;
}

void askPinning(int n)
{
	int pin;

	do {
		Serial.print("Which analog pin is sensor ");
		Serial.print(n);
		Serial.print(" connected to? ");
		pin = processSerialDataForPin(PIN_TYPE_ANALOG);
		if (pin == -1) {
			Serial.print("Illegal analog pin. Enter A0 for analog "
				"pin 0, A1 for analog pin 1 etc. ");
		}
	} while (pin == -1);

	if (tlSensors.data[n].sampleMethod == TLSampleMethodCVD) {
		tlSensors.data[n].tlStructSampleMethod.CVD.pin = pin;
	}

	if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
		tlSensors.data[n].tlStructSampleMethod.resistive.pin = pin;
	}

	if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
		do {
			Serial.print("Which digital pin is the resistive "
				" sensor also connected to (used as ground "
				"pin)? ");
			pin = processSerialDataForPin(PIN_TYPE_DIGITAL);
			if (pin == -1) {
				Serial.print("Illegal pin. Enter 0 for digital "
					"pin 0, 1 for digital pin 1 etc. ");
			}
		} while (pin == -1);

		tlSensors.data[n].tlStructSampleMethod.resistive.gndPin = pin;
	}
}

bool askSampleMethod(int n)
{
	char c;

	Serial.println("");
	Serial.print("Is sensor ");
	Serial.print(n);
	Serial.print(" a capacitive or a resistive sensor? ");
	do {
		Serial.print("Enter c for capacitive or r for resistive: ");
		c = Serial_getChar();
		Serial.println(c);
		if ((lower(c) != 'c') && (lower(c) != 'r')) {
			Serial.print("Illegal sensor type. ");
		} else {
			if (lower(c) == 'c') {
				tlSensors.initialize(n, TLSampleMethodCVD);
			}
			if (lower(c) == 'r') {
				tlSensors.initialize(n, TLSampleMethodResistive);
				tlSensors.data[n].enableSlewrateLimiter = false;
			}
			break;
		}
	} while (true);

	return false;
}

void noiseTuning(int n)
{
	char c;
	bool highNoise = false;

	Serial.print("Performing noise measurement for sensor ");
	Serial.print(n);
	Serial.print(". ");
	do {
		Serial.print("Make sure to not touch the sensor. Enter y to "
			"start the noise measurement. ");
		c = Serial_getChar();
		Serial.println(c);
		if (lower(c) != 'y') {
			Serial.print("Illegal choice. Only y is allowed. ");
		}
	} while (lower(c) != 'y');

	Serial.println("Noise measurement started... ");
	tlSensors.data[n].enableTouchStateMachine = true;
	tlSensors.data[n].enableNoisePowerMeasurement = true;
	tlSensors.setState(n, TLStruct::buttonStatePreCalibrating);
	while (tlSensors.getState(n) != TLStruct::buttonStateReleased) {
		tlSensors.sample();
	}
	Serial.print("Noise measurement for sensor ");
	Serial.print(n);
	Serial.println(" is finished.");

	highNoise = false;

	if (tlSensors.data[n].releasedToApproachedThreshold <
			3 * sqrt(tlSensors.data[n].noisePower)) {
		tlSensors.data[n].releasedToApproachedThreshold =
			3 * sqrt(tlSensors.data[n].noisePower);
		highNoise = true;
	}

	if (tlSensors.data[n].approachedToReleasedThreshold <
			0.9 * 3 * sqrt(tlSensors.data[n].noisePower)) {
		tlSensors.data[n].approachedToReleasedThreshold =
			0.9 * 3 * sqrt(tlSensors.data[n].noisePower);
		highNoise = true;
	}

	if (highNoise) {
		Serial.println("");
		Serial.println("Warning! High noise levels detected! Increased"
			" released to approached and approached to released "
			" thresholds!");
		Serial.println("");
	}

	Serial.println("Found the following thresholds:");
	Serial.print("  released -> approached: ");
	Serial.println(tlSensors.data[n].releasedToApproachedThreshold);
	Serial.print("  approached -> released: ");
	Serial.println(tlSensors.data[n].approachedToReleasedThreshold);

	Serial.println("");
}

#define N_MEASUREMENTS 10
void touchTuning(int n)
{
	char c;
	float delta[N_MEASUREMENTS];
	float avg = 0;
	float min_val;
	int k;
	bool done = false;

	Serial.print("Performing touch measurement for sensor ");
	Serial.print(n);
	Serial.print(". ");

	do {
		do {
			Serial.print("Make sure to touch and hold the sensor, "
				"then press y to start the touch "
				"measurement. ");
			c = Serial_getChar();
			Serial.println(c);
			if (lower(c) != 'y') {
				Serial.println("Illegal choice. Only y is "
					"allowed. ");
			}
		} while (lower(c) != 'y');
	
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
		
		tlSensors.data[n].approachedToPressedThreshold = avg / 2;
		tlSensors.data[n].pressedToApproachedThreshold = 0.9 *
			tlSensors.data[n].approachedToPressedThreshold;

		if (tlSensors.data[n].pressedToApproachedThreshold < 1.1 *
				tlSensors.data[n].releasedToApproachedThreshold) {
			Serial.print("Error! Detected signal is too low. ");
			Serial.print("avg: ");
			Serial.print(avg);
			Serial.print(", avg/2: ");
			Serial.print(avg/2);
			Serial.print(", 0.9*avg/2: ");
			Serial.print(0.9*avg/2);
			Serial.print(", r2a threshold: ");
			Serial.print(tlSensors.data[n].releasedToApproachedThreshold);
			Serial.print(" ");
		} else {
			done = true;
		}
	} while (!done);

	Serial.println("Found the following thresholds:");
	Serial.print("  approached -> pressed: ");
	Serial.println(tlSensors.data[n].approachedToPressedThreshold);
	Serial.print("  pressed -> approached: ");
	Serial.println(tlSensors.data[n].pressedToApproachedThreshold);
	Serial.println("");
}

void maxTouchTuning(int n)
{
	char c;
	float maxDelta[N_MEASUREMENTS];
	float min_val;
	int k;
	bool done = false;

	Serial.print("Performing maximum range measurement for sensor ");
	Serial.print(n);
	Serial.print(". ");

	do {
		do {
			if (tlSensors.data[n].sampleMethod == 
					TLSampleMethodCVD) {
				Serial.print("Make sure to cover and hold the "
					"sensor with as many fingers as will "
					"fit or with your whole hand, then "
					"press y to start the touch "
					"measurement. ");
			}
			if (tlSensors.data[n].sampleMethod == 
					TLSampleMethodResistive) {
				Serial.print("Make sure to press and hold the "
					"sensor as firmly as possible, then "
					"press y to start the touch "
					"measurement. ");
			}
			c = Serial_getChar();
			Serial.println(c);
			if (lower(c) != 'y') {
				Serial.print("Illegal choice. Only y is "
					"allowed. ");
			}
		} while (lower(c) != 'y');
	
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

	Serial.print("Found the following maximum: ");
	Serial.println(maxDelta[N_MEASUREMENTS - 1]);
	Serial.println("");
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
		Serial.print("Capacitive sensor ");
		Serial.print(cap_sensor);
		Serial.print(" and resistive sensor ");
		Serial.print(res_sensor);
		Serial.println(" use the same analog pin. I will enforce that "
			"the resistive sensor is automatically recalibrated "
			"when the capacitive sensor is released.");

		tlSensors.data[cap_sensor].forceCalibrationWhenApproachingFromPressed |=
	                (1UL << res_sensor);
	}
}

void printCode(void)
{
	char c;
	int n, pin;

	do {
		Serial.print("Tuning has finished. Press y to get copy/paste "
			"code to use in your project. ");
		c = Serial_getChar();
		Serial.println(c);
		if (lower(c) != 'y') {
			Serial.print("Illegal choice. Only y is "
				"allowed. ");
		}
	} while (lower(c) != 'y');

//	Serial.print("\n");
//	Serial.print("********************************************************************************\n");
//	Serial.print("\n");
//
//	Serial.print("#include <TouchLib.h>\n");
//	Serial.print("\n");
//	Serial.print("/*\n");
//	Serial.print(" * Code generated by TouchLib SemiAutoTuning.\n");
//	Serial.print(" *\n");
//	Serial.print(" * Hardware configuration:\n");
//	for (n = 0; n < nSensors; n++) {
//		Serial.print(" *   sensor ");
//		Serial.print(n);
//		Serial.print(": type: ");
//		if (tlSensors.data[n].sampleMethod == TLSampleMethodCVD) {
//			Serial.print("capacitive, ");
//		}
//		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
//			Serial.print("resistive,  ");
//		}
//		Serial.print("analog pin A");
//		if (tlSensors.data[n].sampleMethod == TLSampleMethodCVD) {
//			pin = tlSensors.data[n].tlStructSampleMethod.CVD.pin;
//		}
//		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
//			pin = tlSensors.data[n].tlStructSampleMethod.resistive.pin;
//		}
//		Serial.print(pin - A0);
//		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
//			Serial.print(", ground pin: ");
//			Serial.print(tlSensors.data[n].tlStructSampleMethod.resistive.gndPin);
//		}
//		Serial.print("\n");
//	}
//	Serial.print(" */\n");
//	Serial.print("\n");
//
//	Serial.print("/*\n");
//	Serial.print(" *  Number of sensors. Needs a minimum of 2. When "
//		"using only one\n");
//	Serial.print(" * sensor, set N_SENSORS to 2 and use an unused analog "
//		"input pin for the second\n");
//	Serial.print(" * sensor. For 2 or more sensors you don't need to add"
//		" an unused analog input.\n");
// 	Serial.print(" */\n");
//	Serial.print("#define N_SENSORS                       ");
//	Serial.print(nSensors);
//	Serial.print("\n");
//	Serial.print("\n");
//
//	Serial.print("/*\n");
//	Serial.print(" * Number of measurements per sensor to take in one "
//		"cycle. More measurements\n");
//	Serial.print(" * means more noise reduction / spreading, but is also "
//		"slower.\n");
//	Serial.print(" */\n");
//	Serial.print("#define N_MEASUREMENTS_PER_SENSOR       16\n");
//	Serial.print("\n");
//
//	Serial.print("/* tlSensors is the actual object that contains all the"
//		" sensors */\n");
//	Serial.print("TLSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR> "
//		"tlSensors;\n");
//	Serial.print("\n");
//	Serial.print("void setup()\n");
//	Serial.print("{\n");
//        Serial.print("        Serial.begin(9600);\n");
//	Serial.print("        /* Delay to make sure serial monitor receives "
//		"first message */\n");
//        Serial.print("        delay(500);\n");
//        Serial.print("        Serial.println();\n");
//        Serial.print("        Serial.println();\n");
//	Serial.print("        Serial.println(\"Switching baudrate to 115200. "
//		"Make sure to adjust baudrate in serial monitor as well!\");\n");
//        Serial.print("        Serial.println();\n");
//        Serial.print("        Serial.println();\n");
//        Serial.print("        Serial.end();\n");
//	Serial.print("\n");
//        Serial.print("        /*\n");
//        Serial.print("         * Switch baudrate to highest baudrate "
//		"available. With higher baudrate,\n");
//	Serial.print("         * CPU has more time left to do capacitive "
//		"sensing and thus get better\n");
//	Serial.print("         * signal quality.\n");
//        Serial.print("         */\n");
//        Serial.print("        Serial.begin(115200);\n");
//        Serial.print("        delay(500);\n");
//        Serial.print("        Serial.println();\n");
//        Serial.print("        Serial.println();\n");
//
//        for (n = 0; n < nSensors; n++) {
//		Serial.print("\n");
//		Serial.print("        /*\n");
//		Serial.print("         * Configuration for sensor ");
//		Serial.print(n);
//		Serial.print(":\n");
//		if (tlSensors.data[n].sampleMethod == TLSampleMethodCVD) {
//			Serial.print("         * Type: capacitive\n");
//		}
//		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
//			Serial.print("         * Type: resistive\n");
//		}
//		Serial.print("         * Analog pin: A");
//		if (tlSensors.data[n].sampleMethod == TLSampleMethodCVD) {
//			pin = tlSensors.data[n].tlStructSampleMethod.CVD.pin;
//		}
//		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
//			pin = tlSensors.data[n].tlStructSampleMethod.resistive.pin;
//		}
//		Serial.print(pin - A0);
//		Serial.print("\n");
//		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
//			Serial.print("         * Ground pin: ");
//			Serial.print(tlSensors.data[n].tlStructSampleMethod.resistive.gndPin);
//			Serial.print("\n");
//		}
//		Serial.print("         */\n");
//		Serial.print("        tlSensors.initialize(");
//		Serial.print(n);
//		if (tlSensors.data[n].sampleMethod == TLSampleMethodCVD) {
//			Serial.print(", TLSampleMethodCVD);\n");
//		}
//		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
//			Serial.print(", TLSampleMethodResistive);\n");
//		}
//		Serial.print("        tlSensors.data[");
//		Serial.print(n);
//		Serial.print("].tlStructSampleMethod.");
//		if (tlSensors.data[n].sampleMethod == TLSampleMethodCVD) {
//			Serial.print("CVD.pin =               A");
//		}
//		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
//			Serial.print("resistive.pin =         A");
//		}
//		Serial.print(pin - A0);
//		Serial.print(";\n");
//		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
//			Serial.print("        tlSensors.data[");
//			Serial.print(n);
//			Serial.print("].tlStructSampleMethod.resistive.gndPin =      ");
//			Serial.print(tlSensors.data[n].tlStructSampleMethod.resistive.gndPin);
//			Serial.print(";\n");
//		}
//		Serial.print("        tlSensors.data[");
//		Serial.print(n);
//		Serial.print("].releasedToApproachedThreshold =              ");
//		Serial.print(tlSensors.data[n].releasedToApproachedThreshold);
//		Serial.print(";\n");
//		Serial.print("        tlSensors.data[");
//		Serial.print(n);
//		Serial.print("].approachedToReleasedThreshold =              ");
//		Serial.print(tlSensors.data[n].approachedToReleasedThreshold);
//		Serial.print(";\n");
//		Serial.print("        tlSensors.data[");
//		Serial.print(n);
//		Serial.print("].approachedToPressedThreshold =               ");
//		Serial.print(tlSensors.data[n].approachedToPressedThreshold);
//		Serial.print(";\n");
//		Serial.print("        tlSensors.data[");
//		Serial.print(n);
//		Serial.print("].pressedToApproachedThreshold =               ");
//		Serial.print(tlSensors.data[n].pressedToApproachedThreshold);
//		Serial.print(";\n");
//		Serial.print("        tlSensors.data[");
//		Serial.print(n);
//		Serial.print("].calibratedMaxDelta =                         ");
//		Serial.print(tlSensors.data[n].calibratedMaxDelta);
//		Serial.print(";\n");
//		Serial.print("        tlSensors.data[");
//		Serial.print(n);
//		Serial.print("].enableSlewrateLimiter =                      ");
//		if (tlSensors.data[n].enableSlewrateLimiter) {
//			Serial.print("true;\n");
//		} else {
//			Serial.print("false;\n");
//		}
//
//		if (tlSensors.data[n].forceCalibrationWhenApproachingFromPressed) {
//			Serial.print("        tlSensors.data[");
//			Serial.print(n);
//			Serial.print("].forceCalibrationWhenApproachingFromPressed"
//				" = 0x");
//			Serial.print(tlSensors.data[n].forceCalibrationWhenApproachingFromPressed,
//				HEX);
//			Serial.print(";\n");
//		}
//	}
//
//	Serial.print("\n");
//	Serial.print("        Serial.println(\"Calibrating sensors...\");\n");
//	Serial.print("        while(tlSensors.anyButtonIsCalibrating()) {\n");
//	Serial.print("                tlSensors.sample();\n");
//	Serial.print("        }\n");
//	Serial.print("        Serial.println(\"Calibration done...\");\n");
//	Serial.print("}\n");
//
//	Serial.print("\n");
//	Serial.print("void print_sensor_state(int n)\n");
//	Serial.print("{\n");
//	Serial.print("        char s[32] = {'\\0'};\n");
//	Serial.print("\n");
//	Serial.print("        Serial.print(\" #\");\n");
//	Serial.print("        Serial.print(n);\n");
//	Serial.print("        Serial.print(\": \");\n");
//	Serial.print("        Serial.print(tlSensors.isCalibrating(n));\n");
//	Serial.print("        Serial.print(\" \");\n");
//	Serial.print("        Serial.print(tlSensors.isReleased(n));\n");
//	Serial.print("        Serial.print(\" \");\n");
//	Serial.print("        Serial.print(tlSensors.isApproached(n));\n");
//	Serial.print("        Serial.print(\" \");\n");
//	Serial.print("        Serial.print(tlSensors.isPressed(n));\n");
//	Serial.print("        Serial.print(\" \");\n");
//	Serial.print("        Serial.print(tlSensors.getStateLabel(n));\n");
//	Serial.print("        memset(s, '\\0', sizeof(s));\n");
//	Serial.print("        memset(s, ' ', 22 - "
//			"strlen(tlSensors.getStateLabel(n)));\n");
//	Serial.print("        Serial.print(s);\n");
//	Serial.print("}\n");
//	Serial.print("\n");
//	Serial.print("#define BAR_LENGTH                     60 /* <-- Change "
//		"this to print longer or shorter visualizations */\n");
//	Serial.print("\n");
//	Serial.print("void loop(void)\n");
//	Serial.print("{\n");
//	Serial.print("        int k;\n");
//	Serial.print("\n");
//	Serial.print("        int n = 0; /* <-- Change this number to view a "
//		"different sensor */\n");
//	Serial.print("\n");
//	Serial.print("        tlSensors.sample(); /* <-- Take a series of new "
//		"samples for all sensors */\n");
//	Serial.print("\n");
//	Serial.print("        tlSensors.printBar(n, BAR_LENGTH); /* <-- Print "
//		"the visualization */\n");
//	Serial.print("\n");
//	Serial.print("        print_sensor_state(n); /* <-- Print summary of "
//		"sensor n */\n");
//	Serial.print("\n");
//	Serial.print("        /* For capacitive + resistive sensors: k is the "
//		"sensor that sensor n is paired with */\n");
//	Serial.print("        k = tlSensors.findSensorPair(n, (n + 1) % "
//		"N_SENSORS);\n");
//	Serial.print("        if (k > -1) {\n");
//	Serial.print("                print_sensor_state(k); /* <-- Print "
//		"summary of sensor k */\n");
//	Serial.print("        }\n");
//	Serial.print("        Serial.println(\"\");\n");
//	Serial.print("}\n");
//
//	Serial.print("\n");
//	Serial.print("********************************************************************************\n");
}

void loop()
{
	int n;

	askPower();
	askNSensors();
	Serial.println("First we need to know what type of sensors the system "
		"has and to which pins they are connected.");
	for (n = 0; n < nSensors; n++) {
		askSampleMethod(n);
		askPinning(n);
	}
	Serial.println("");

	Serial.println("Next step is to tune the sensors.");
	for (n = 0; n < nSensors; n++) {
		noiseTuning(n);
		touchTuning(n);
		maxTouchTuning(n);
		Serial.print("Finished tuning sensor ");
		Serial.println(n);
		Serial.println("");
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
	while(1);
}
