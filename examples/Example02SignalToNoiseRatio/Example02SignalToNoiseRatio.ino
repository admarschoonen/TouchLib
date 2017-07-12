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

	tlSensors.initialize(0, TLSampleMethodCVD);
	tlSensors.initialize(1, TLSampleMethodCVD);
	tlSensors.initialize(2, TLSampleMethodResistive);
	tlSensors.initialize(3, TLSampleMethodResistive);

	tlSensors.data[2].tlStructSampleMethod.resistive.pin = A0;
	tlSensors.data[2].tlStructSampleMethod.resistive.gndPin = 2;
	tlSensors.data[3].tlStructSampleMethod.resistive.pin = A1;
	tlSensors.data[3].tlStructSampleMethod.resistive.gndPin = 3;

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

void floatToIntFrac(float f, uint32_t precision, char *sign, uint32_t * iInt,
		uint32_t * iFrac)
{
	float fFrac;

	if (f >= 0) {
		*iInt = f;
		fFrac = f - *iInt;
		*sign = ' ';
	} else {
		*iInt = -(f);
		fFrac = -(f + *iInt);
		*sign = '-';
	}

	if (precision > 0) {
		*iFrac = floor(pow(10, precision) * fFrac);
	} else {
		*iFrac = 0;
	}
}

uint32_t intLength(uint32_t i)
{
	uint32_t l;
	uint32_t tmp;
	
	if (i < 0) {
		tmp = -i;
	} else {
		tmp = i;
	}
	
	l = 1;
	while (tmp >= 10) {
		l++;
		tmp = tmp / 10;
	}
	
	return l;
}

int floatToStr(float f, int scale, int precision, char *s, int length)
{
	char sign;
	uint32_t iInt;
	uint32_t iFrac;
	uint32_t i;
	uint32_t iLength, fLength;
	uint32_t tmp;
	
	floatToIntFrac(f, precision, &sign, &iInt, &iFrac);
	iLength = intLength(iInt);
	fLength = intLength(iFrac);
	
	if (scale < iLength) {
		i = snprintf(s, length - 1, "OVF");
	} else {
		i = 0;
		if (i < length - 1) {
			/* print spaces */
			tmp = (length - 1 - i < scale - iLength) ? length - 1 - i : scale - iLength;
			memset(&(s[i]), ' ', tmp);
			i += tmp;
			if (i < length - 1) {
				/* print sign */
				s[i++] = sign;
				if (i < length - 1) {
					/* print integer portion */
					i += snprintf(&(s[i]), length - 1 - i, "%lu", iInt);
					if (i < length - 1) {
						/* print dot */
						s[i++] = '.';
						if (i < length - 1) {
							/* print leading 0's for decimal portion */
							tmp = (length - 1 - i < precision - fLength) ?
								length - 1 - i : precision - fLength;
							tmp = (tmp < 0) ? 0 : tmp;
							memset(&(s[i]), '0', tmp);
							i += tmp;
							if (i < length - 1) {
								/* print decimal portion */
								i += snprintf(&(s[i]), length - 1 - i, "%lu", iFrac);
							}
						}
					}
				}
			}
		}
	}
	return i;
}



void loop()
{
	static int b = 0;
	char s[200] = {'\0'};
	char rawS[13] = {'\0'};
	char valueS[13] = {'\0'};
	char avgS[13] = {'\0'};
	char deltaS[13] = {'\0'};
	char noisePowerS[13] = {'\0'};
	char noiseAmpS[13] = {'\0'};
	char snrS[13] = {'\0'};
	int n;
	const char *buttonStateLabel;
	float raw, value, avg, delta, noisePower, noiseAmp, snr;
	int rawInt, rawFrac, valueInt, valueFrac, avgInt, avgFrac, deltaInt,
		deltaFrac, noisePowerInt, noisePowerFrac, noiseAmpInt,
		noiseAmpFrac, snrInt, snrFrac;

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

	raw = tlSensors.getRaw(b);
	value = tlSensors.getValue(b);
	avg = tlSensors.getAvg(b);
	delta = tlSensors.getDelta(b);
	noisePower = tlSensors.data[b].noisePower;
	noiseAmp = sqrt(noisePower);
	snr = 10 * log10(delta * delta / noisePower);
	buttonStateLabel = tlSensors.getStateLabel(b);

	floatToStr(raw, 6, 3, rawS, sizeof(rawS));
	floatToStr(value, 6, 3, valueS, sizeof(valueS));
	floatToStr(avg, 6, 3, avgS, sizeof(avgS));
	floatToStr(delta, 6, 3, deltaS, sizeof(deltaS));
	floatToStr(noisePower, 6, 3, noisePowerS, sizeof(noisePowerS));
	floatToStr(noiseAmp, 6, 3, noiseAmpS, sizeof(noiseAmpS));
	floatToStr(snr, 6, 3, snrS, sizeof(snrS));

	snprintf(s, sizeof(s) -1, "button[%d]: raw: %s, value: %s, avg: %s, "
		"delta: %s, noise: %s, " "SNR: %s dB, state: %s\r\n", b, rawS,
		valueS, avgS, deltaS, noiseAmpS, snrS, buttonStateLabel);
	Serial.print(s);
}
