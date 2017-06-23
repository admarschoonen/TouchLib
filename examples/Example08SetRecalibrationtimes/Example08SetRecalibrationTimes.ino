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

	/* Change time that sensor must be in pressed transitioning states */
	for (n = 0; n < N_SENSORS; n++) {
		/* 
		 * approachedTimeout is set to 5 minutes by default. If a hand
		 * is held above the sensor for longer than this time, the
		 * sensor will start recalibrating. Reduce this value to recover
		 * sooner in case a button is accidentally held in approached
		 * state due to noise.
		 *
		 * Change the value here and observe the difference in time
		 * before recalibration is triggered.
		 */
		cvdSensors.data[n].approachedTimeout = 10000;

		/*
		 * pressedTimeout is set to 5 minutes by default. It is similar
		 * to approachedTimeout above, but for the pressed state.
		 */
		cvdSensors.data[n].pressedTimeout = 10000;

		/*
		 * calibrationTime is set to 100 ms by default. It is the time
		 * the sensor will stay in calibration state. Increasing this
		 * value will result in better estimate of the background value
		 * (the avg value), but at the cost of longer time at startup
		 * and during recalibration.
		 */
		cvdSensors.data[n].calibrationTime = 500;

		/*
		 * preCalibrationTime is set to 100 ms by default. It is the time
		 * the sensor will wait before entering calibration state
		 * immediately after start up. This allows supply voltages to
		 * settle. Increasing this value will result in longer time for
		 * setting at the cost of longer time at startup.
		 */
		cvdSensors.data[n].preCalibrationTime = 500;
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
		Serial.print(cvdSensors.data[n].delta);
		Serial.print(", approached: ");
		Serial.print(cvdSensors.data[n].buttonIsApproached);
		if (n < N_SENSORS - 1) {	
			Serial.print("\t ");
		}
	}
	Serial.print(", measurement time: ");
	Serial.println(t_stop - t_start);
}
