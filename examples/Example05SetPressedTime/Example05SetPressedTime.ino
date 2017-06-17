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

	/* Change time that sensor must be in pressed transitioning states */
	for (n = 0; n < N_SENSORS; n++) {
		/* 
		 * approachedToPressedTime is set to 10 ms by default. Set to
		 * larger value to make sensor more robust against noise (at the
		 * cost of increasing the response time of the sensor).
		 *
		 * Change the value here and below and observe the difference in
		 * response time.
		 */
		cvdSensors.data[n].approachedToPressedTime = 100;

		/*
		 * pressedToApproachedTime is set to 10 ms by default. Set to
		 * larger value to make sensor more robust against noise (at the
		 * cost of increasing the response time of the sensor).
		 */
		cvdSensors.data[n].pressedToApproachedTime = 100;
	}
}

void loop()
{
	int n;
	unsigned long t_start, t_stop;

	t_start = millis();
	/* 
	 * Call cvdSensors.sample() take do a new measurement cycle for all
	 * sensors 
	 */
	cvdSensors.sample();
	t_stop = millis();

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
		}
	}
	Serial.print(", measurement time: ");
	Serial.println(t_stop - t_start);
}
