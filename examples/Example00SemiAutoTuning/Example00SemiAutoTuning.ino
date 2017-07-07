#include <EEPROM.h>
#include <TouchLib.h>

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

int processSerialData(int b)
{
	int n = -1;
	char c;

	while (Serial.available()) {
		c = (char) Serial.read();
		n = -1;
		if ((c >= '0') && (c <= '9')) {
			n = c - '0';
		}
		if ((c >= 'A') && (c <= 'F')) {
			n = c - 'A' + 10;
		}
		if ((c >= 'a') && (c <= 'f')) {
			n = c - 'a' + 10;
		}
		if ((n >= 0) && (n == b) && (n <= N_SENSORS)) {
			/* toggle sensor pressed state */
			if (tlSensors.getState(n) ==
					TLStruct::buttonStatePressed) {
				tlSensors.setState(n,
					TLStruct::buttonStateReleased);
			} else if (tlSensors.getState(n) ==
					TLStruct::buttonStateReleased) {
				tlSensors.setState(n,
					TLStruct::buttonStatePressed);
			}
		}
	}

	return n;
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

void loop()
{
	static int b = 0;
	char s[200] = {'\0'};
	int n;
	const char *buttonStateLabel;
	float avg, delta, noisePower, noiseAmp, snr;
	int avgInt, avgFrac, deltaInt, deltaFrac, noisePowerInt, noisePowerFrac,
		noiseAmpInt, noiseAmpFrac, snrInt, snrFrac;

	n = processSerialData(b);

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
