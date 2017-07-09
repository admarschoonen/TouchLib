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
#define N_SENSORS			4

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

void setup()
{
	int n;

	/* Delay to make sure serial monitor receives first message */
	delay(500);
	Serial.begin(9600);
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
	Serial.println();
	Serial.println();

	Serial.println("Welcome to the TouchLib tuning program.");
	Serial.println("");

	for (n = 0; n < N_SENSORS; n++) {
		/*
		 * Disable state machine so we can control updating average in
		 * the main loop.
		 */
		tlSensors.data[n].enableTouchStateMachine = false;

		/* Enable noise power measurement */
		tlSensors.data[n].enableNoisePowerMeasurement = true;
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

int processSerialDataForPin(int type)
{
	int pin = -1;
	char s[20] = {'\0'};
	int n, k;
	bool isAnalog = false, illegalChar;

	do {
		n = Serial_getString(s, sizeof(s));
		Serial.println(s);

		isAnalog = false;
		if (n == 0) {
		} else {
			k = 0;
			illegalChar = false;
			while (s[k] != '\0') {
				if (k == 0) {
					if ((s[k] == 'A') || (s[k] == 'a')) {
						isAnalog = true;
					} else if (!isNum(s[k])) {
						illegalChar = true;
						break;
					}
				}
				if ((k > 0) && (!isNum(s[k]))) {
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

void printWelcome(void)
{
	Serial.print("We will tuning ");
	Serial.print(N_SENSORS);
	Serial.println(" sensors. If this is not the right number of sensors "
		"you want to tune, exit this program now and adjust the "
		"N_SENSORS parameter on line 15 of this code.");
	Serial.println("");
	Serial.print(N_SENSORS);
	Serial.println(" sensors. If this is not the right number of sensors "
		"you want to tune, exit this program now and adjust the "
		"N_SENSORS parameter on line 15 of this code.");
	Serial.println("");
}

bool askPower(void)
{
	char s[20];
	bool ret;
	int n;

	Serial.println("How is your system powered:");
	Serial.println("a. Battery and / or power supply with earth "
		"connection connected to GND.");
	Serial.println("b. Power supply with no earth connection (floating "
		"ground).");
	Serial.println("c. Mixed: sometimes system by battery, sometimes it is"
		" powered by a suppy with floating ground.");
	Serial.println("");
	Serial.print("Enter your choice (a, b or c): ");

	do {
		n = Serial_getString(s, sizeof(s));
		Serial.println(s);
		if ((n != 1) || ((s[0] != 'a') && (s[0] != 'b') &&
				(s[0] != 'c'))) {
			Serial.print("Illegal choice. Enter your choice (a, b or c): ");
		} 
		if ((n == 1) && (s[0] == 'a')) {
			ret = false; /* no need for slewrate limiter */
			break;
		}
		if ((n == 1) && ((s[0] == 'b') || (s[0] == 'c'))) {
			ret = true; /* enable slewrate limiter */
			break;
		}
	} while (true);

	Serial.println("");

	for (n = 0; n < N_SENSORS; n++) {
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

	tlSensors.data[n].pin = pin;
}

bool askSampleMethod(int n)
{
	char c;
	int pin = -1;

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
				tlSensors.data[n].sampleMethod =
					TLSampleMethodCVD;
			}
			if (lower(c) == 'r') {
				tlSensors.data[n].sampleMethod =
					TLSampleMethodResistive;
				tlSensors.data[n].enableSlewrateLimiter = false;
			}
			break;
		}
	} while (true);

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

		tlSensors.data[n].sampleMethodResistive_gndPin = pin;
	}
	return false;
}

void noiseCalibration(int n)
{
	char c;
	bool highNoise = false;

	Serial.print("Performing noise measurement for sensor ");
	Serial.print(n);
	do {
		Serial.print(". Make sure to not touch the sensor. Enter y to "
			"start the noise measurement. ");
		c = Serial_getChar();
		Serial.println(c);
		if (lower(c) != 'y') {
			Serial.println("Illegal choice. Only y is allowed. ");
		}
	} while (lower(c) != 'y');

	tlSensors.data[n].enableTouchStateMachine = true;
	tlSensors.data[n].enableNoisePowerMeasurement = true;
	tlSensors.setState(n, TLStruct::buttonStatePreCalibrating);
	while (tlSensors.getState(n) != TLStruct::buttonStateReleased) {
		tlSensors.sample();
	}
	Serial.print("Noise measurement for sensor ");
	Serial.print(n);
	Serial.println(" is finished.");
	tlSensors.data[n].releasedToApproachedThreshold = 3 *
		sqrt(tlSensors.data[n].noisePower);
	tlSensors.data[n].approachedToReleasedThreshold = 0.9 *
		tlSensors.data[n].releasedToApproachedThreshold;

	if (tlSensors.data[n].releasedToApproachedThreshold <
		TL_RELEASED_TO_APPROACHED_THRESHOLD_DEFAULT) {
		tlSensors.data[n].releasedToApproachedThreshold =
			TL_RELEASED_TO_APPROACHED_THRESHOLD_DEFAULT;
	} else {
		highNoise = true;
	}

	if (tlSensors.data[n].approachedToReleasedThreshold <
		TL_APPROACHED_TO_RELEASED_THRESHOLD_DEFAULT) {
		tlSensors.data[n].approachedToReleasedThreshold =
			TL_APPROACHED_TO_RELEASED_THRESHOLD_DEFAULT;
	} else {
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
void touchCalibration(int n)
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

void maxTouchCalibration(int n)
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
	int cap_sensor, res_sensor;

	tlSensors.data[n].forceCalibrationWhenApproachingFromPressed = 0UL;

	if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
		res_sensor = n;
	}

	if (tlSensors.data[n].sampleMethod == TLSampleMethodCVD) {
		cap_sensor = n;
	}

	for (k = n; k < N_SENSORS; k++) {
		if (tlSensors.data[k].sampleMethod == TLSampleMethodResistive) {
			res_sensor = k;
		}

		if (tlSensors.data[k].sampleMethod == TLSampleMethodCVD) {
			cap_sensor = k;
		}

		if ((tlSensors.data[n].pin == tlSensors.data[k].pin) &&
				(((tlSensors.data[n].sampleMethod == TLSampleMethodResistive) &&
				(tlSensors.data[k].sampleMethod == TLSampleMethodCVD)) ||
				((tlSensors.data[k].sampleMethod == TLSampleMethodResistive) &&
				(tlSensors.data[n].sampleMethod == TLSampleMethodCVD)))) {
			Serial.print("Capacitive sensor ");
			Serial.print(cap_sensor);
			Serial.print(" and resistive sensor ");
			Serial.print(res_sensor);
			Serial.println(" use the same analog pin. I will "
				" enforce that the resistive sensor is "
				"automatically recalibrated when the capacitive"
				" sensor is released.");

			tlSensors.data[cap_sensor].forceCalibrationWhenApproachingFromPressed =
		                (1UL << res_sensor);
		}
	}
}

void printCode(void)
{
	char c;
	int n;

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

	Serial.println("#include <TouchLib.h>");
	Serial.println("");
	Serial.println("/*");
	Serial.println(" * Code generated by TouchLib SemiAutoTuning.");
	Serial.println(" *");
	Serial.println(" * Hardware configuration:");
	for (n = 0; n < N_SENSORS; n++) {
		Serial.print(" *   sensor ");
		Serial.print(n);
		Serial.print(": type: ");
		if (tlSensors.data[n].sampleMethod == TLSampleMethodCVD) {
			Serial.print("capacitive, ");
		}
		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
			Serial.print("resistive,  ");
		}
		Serial.print("analog pin A");
		Serial.print(tlSensors.data[n].pin - A0);
		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
			Serial.print(", ground pin: ");
			Serial.print(tlSensors.data[n].sampleMethodResistive_gndPin);
		}
		Serial.println("");
	}
	Serial.println(" */");
	Serial.println("");

	Serial.println("/*");
	Serial.println(" *  Number of sensors. Needs a minimum of 2. When "
		"using only one");
	Serial.println(" * sensor, set N_SENSORS to 2 and use an unused analog "
		"input pin for the second");
	Serial.println(" * sensor. For 2 or more sensors you don't need to add"
		" an unused analog input.");
 	Serial.println(" */");
	Serial.print("#define N_SENSORS                         ");
	Serial.println(N_SENSORS);
	Serial.println("");

	Serial.println("/*");
	Serial.println(" * Number of measurements per sensor to take in one "
		"cycle. More measurements");
	Serial.println(" * means more noise reduction / spreading, but is also "
		"slower.");
	Serial.println(" */");
	Serial.println("#define N_MEASUREMENTS_PER_SENSOR       16");
	Serial.println("");

	Serial.println("/* tlSensors is the actual object that contains all the"
		" sensors */");
	Serial.println("TLSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR> "
		"tlSensors;");
	Serial.println("");
	Serial.println("void setup()");
	Serial.println("{");
        Serial.println("        int n;");
	Serial.println("");
	Serial.println("        /* Delay to make sure serial monitor receives "
		"first message */");
        Serial.println("        delay(500);");
        Serial.println("        Serial.begin(9600);");
        Serial.println("        Serial.println();");
        Serial.println("        Serial.println();");
	Serial.println("        Serial.println(\"Switching baudrate to 115200. "
		"Make sure to adjust baudrate in serial monitor as well!\");");
        Serial.println("        Serial.println();");
        Serial.println("        Serial.println();");
        Serial.println("        Serial.end();");
	Serial.println("");
        Serial.println("        /*");
        Serial.println("         * Switch baudrate to highest baudrate "
		"available. With higher baudrate,");
	Serial.println("         * CPU has more time left to do capacitive "
		"sensing and thus get better");
	Serial.println("         * signal quality.");
        Serial.println("         */");
        Serial.println("        Serial.begin(115200);");
        Serial.println("        Serial.println();");
        Serial.println("        Serial.println();");

        for (n = 0; n < N_SENSORS; n++) {
		Serial.println("");
		Serial.println("        /* ");
		Serial.print("         * Configuration for sensor ");
		Serial.print(n);
		Serial.println(":");
		if (tlSensors.data[n].sampleMethod == TLSampleMethodCVD) {
			Serial.println("         * Type: capacitive");
		}
		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
			Serial.println("         * Type: resistive");
		}
		Serial.print("         * Analog pin: A");
		Serial.println(tlSensors.data[n].pin - A0);
		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
			Serial.print("        * Ground pin: ");
			Serial.println(tlSensors.data[n].sampleMethodResistive_gndPin);
		}
		Serial.println("         */");
		Serial.print("        tlSensors.data[");
		Serial.print(n);
		Serial.print("].pin = ");
		Serial.print(tlSensors.data[n].pin - A0);
		Serial.println(";");
		if (tlSensors.data[n].sampleMethod == TLSampleMethodResistive) {
			Serial.print("        tlSensors.data[");
			Serial.print(n);
			Serial.print("].sampleMethodResistive_gndPin = ");
			Serial.print(tlSensors.data[n].sampleMethodResistive_gndPin);
			Serial.println(";");
		}
		Serial.print("        tlSensors.data[");
		Serial.print(n);
		Serial.print("].releasedToApproachedThreshold = ");
		Serial.print(tlSensors.data[n].releasedToApproachedThreshold);
		Serial.println(";");
		Serial.print("        tlSensors.data[");
		Serial.print(n);
		Serial.print("].approachedToReleasedThreshold = ");
		Serial.print(tlSensors.data[n].approachedToReleasedThreshold);
		Serial.println(";");
		Serial.print("        tlSensors.data[");
		Serial.print(n);
		Serial.print("].approachedToPressedThreshold =  ");
		Serial.print(tlSensors.data[n].approachedToPressedThreshold);
		Serial.println(";");
		Serial.print("        tlSensors.data[");
		Serial.print(n);
		Serial.print("].pressedToApproachedThreshold =  ");
		Serial.print(tlSensors.data[n].pressedToApproachedThreshold);
		Serial.println(";");
		Serial.print("        tlSensors.data[");
		Serial.print(n);
		Serial.print("].calibratedMaxDelta =            ");
		Serial.print(tlSensors.data[n].calibratedMaxDelta);
		Serial.println(";");
		if ((tlSensors.data[n].sampleMethod == TLSampleMethodCVD) &&
				(tlSensors.data[n].forceCalibrationWhenApproachingFromPressed)) {
			Serial.print("        tlSensors.data[");
			Serial.print(n);
			Serial.print("].forceCalibrationWhenApproachingFromPressed"
				" = 0x");
			Serial.print(tlSensors.data[n].forceCalibrationWhenApproachingFromPressed,
				HEX);
			Serial.println(";");
		}
	}

	Serial.println("}");
}

void loop()
{
	static int b = 0;
	static bool firstTime = true;

	char s[200] = {'\0'};
	int n;
	const char *buttonStateLabel;
	float avg, delta, noisePower, noiseAmp, snr;
	int avgInt, avgFrac, deltaInt, deltaFrac, noisePowerInt, noisePowerFrac,
		noiseAmpInt, noiseAmpFrac, snrInt, snrFrac;

	if (firstTime == false) {
		return;
	}

	firstTime = false;
	printWelcome();

	askPower();
	for (n = 0; n < N_SENSORS; n++) {
		askPinning(n);
		askSampleMethod(n);
		noiseCalibration(n);
		touchCalibration(n);
		maxTouchCalibration(n);
	}
	
	for (n = 0; n < N_SENSORS; n++) {
		checkResistiveCapacitiveSensor(n);
	}
	Serial.println("");

	printCode();

	/*
	 * Call tlSensors.sample() take do a new measurement cycle for all
	 * sensors
	 */
	tlSensors.sample();

	/*
	 * Only print statistics for selected button
	 */
	if (n > -1) {
		b = n;
	}

	avg = tlSensors.getAvg(b);
	delta = tlSensors.getDelta(b);
	noisePower = tlSensors.data[b].noisePower;
	noiseAmp = sqrt(noisePower);
	snr = 10 * log10(delta * delta / noisePower);
	buttonStateLabel = tlSensors.getStateLabel(b);

	floatToIntFrac(avg, 1, 2, &avgInt, &avgFrac);
	floatToIntFrac(delta, 1, 2, &deltaInt, &deltaFrac);
	floatToIntFrac(noisePower, 1, 2, &noisePowerInt, &noisePowerFrac);
	floatToIntFrac(noiseAmp, 1, 2, &noiseAmpInt, &noiseAmpFrac);
	floatToIntFrac(snr, 1, 2, &snrInt, &snrFrac);

	snprintf(s, sizeof(s) -1, "button[%d]: avg: % 7d.%02d, delta: "
		"% 7d.%02d, noise: % 7d.%02d, SNR: % 7d.%02d dB, state: %s\r\n",
		b, avgInt, avgFrac, deltaInt, deltaFrac, noiseAmpInt,
		noiseAmpFrac, snrInt, snrFrac, buttonStateLabel);
	Serial.print(s);
}
