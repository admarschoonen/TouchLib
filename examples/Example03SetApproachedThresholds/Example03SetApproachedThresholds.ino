#include <CVDSensor.h>

/*
 * CVDSense Library Demo Sketch
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

/* cvdSensors is the actual object that contains all the sensors */
CvdSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR> cvdSensors;

void setup()                    
{
	int n;

	Serial.begin(9600);

	/* Change approached threshold for each sensor */
	for (n = 0; n < N_SENSORS; n++) {
		/* 
		 * releasedToApproachedThreshold is set to 50 by default. Set to
		 * lower value to make sensor more sensitive.
		 */
		cvdSensors.data[n].releasedToApproachedThreshold = 25;

		/*
		 * approachedToReleasedThreshold is set to 40 by default. It
		 * should usually be a little lower than
		 * releasedToApproachedThreshold to prevent button instability.
		 */
		cvdSensors.data[n].approachedToReleasedThreshold = 20;
	}
}

void loop()
{
	int n;

	/* 
	 * Call cvdSensors.sample() take do a new measurement cycle for all
	 * sensors 
	 */
	cvdSensors.sample();

	/*
	 * For each button, print current value and if button is approached or
	 * not
	 */
	for (n = 0; n < N_SENSORS; n++) {
		Serial.print("button[");
		Serial.print(n);
		Serial.print("]: current value: ");
		Serial.print(cvdSensors.data[n].capacitance);
		Serial.print(", approached: ");
		Serial.print(cvdSensors.data[n].buttonIsApproached);
		if (n < N_SENSORS - 1) {	
			Serial.print("\t ");
		} else {
			Serial.println("");
		}
	}
}
