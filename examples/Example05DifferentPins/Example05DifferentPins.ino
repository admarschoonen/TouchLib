#include <CVDSensor.h>

/*
 * CVDSense Library Demo Sketch
 * Admar Schoonen 2016
 * Connect 4 electrodes (piece of metal sheet / foil) to analog pins A0, A2, A4
 * and A6
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
	Serial.begin(9600);

	/*
	 * By default, pins are connected starting at A0 in successive order.
	 * Change the .pin parameter to select a different pin.
	 */
	cvdSensors.data[0].pin = 0; /* Analog pin 0 */
	cvdSensors.data[1].pin = 2; /* Analog pin 2 */
	cvdSensors.data[2].pin = 4; /* Analog pin 4 */
	cvdSensors.data[3].pin = 6; /* Analog pin 6 */
}

void loop()
{
	int n;

	/* 
	 * Call cvdSensors.sample() take do a new measurement cycle for all
	 * sensors 
	 */
	cvdSensors.sample();

	/* For each button, print current value and button state label */
	for (n = 0; n < N_SENSORS; n++) {
		Serial.print("button[");
		Serial.print(n);
		Serial.print("]: current value: ");
		Serial.print(cvdSensors.data[n].capacitance);
		Serial.print(", buttonStateLabel: ");
		Serial.print(cvdSensors.data[n].buttonStateLabel);
		if (n < N_SENSORS - 1) {	
			Serial.print("\t ");
		} else {
			Serial.println("");
		}
	}
}
