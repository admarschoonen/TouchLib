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
	Serial.begin(9600);

	/*
	 * The parameter N_MEASUREMENTS_PER_SENSOR (see above) determines the
	 * number of measurements per sensor per measurement cycle. CVDSensor
	 * uses a pseudo-random sampling order to spread noise randomly over
	 * multiple samples. A higher value uses more samples and thus more
	 * noise reduction. However, a higher value also makes the total
	 * measurement cycle longer.
	 *
	 * Emperically, a value of 16 seems to be a reasonable setting for many
	 * projects. However, for projects where the sensors are affected by a
	 * lot of noise, a higher value might be preferrable. On the other
	 * hands, for projects where a very fast response time is required, a
	 * smaller value might be required.
	 *
	 * Adjust the setting above and observe the difference in measurement
	 * time.
	 */
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
	 * For each button, print current value, background value and button
 	 * state label. Also print the time it takes to do a complete loop
 	 * cycle.
	 */
	for (n = 0; n < N_SENSORS; n++) {
		Serial.print("button[");
		Serial.print(n);
		Serial.print("]: current value: ");
		Serial.print(cvdSensors.data[n].capacitance);
		Serial.print(", background value: ");
		Serial.print(cvdSensors.data[n].avg);
		Serial.print(", buttonStateLabel: ");
		Serial.print(cvdSensors.data[n].buttonStateLabel);
		if (n < N_SENSORS - 1) {	
			Serial.print("\t ");
		}
	}
	Serial.print(", measurement time: ");
	Serial.println(t_stop - t_start);
}
