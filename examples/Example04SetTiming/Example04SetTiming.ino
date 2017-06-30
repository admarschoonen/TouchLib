#include <EEPROM.h>
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

	/* Change approached threshold for each sensor */
	for (n = 0; n < N_SENSORS; n++) {
		/* Set thresholds to levels you found in previous step. */
		/*cvdSensors.data[n].releasedToApproachedThreshold = ;
		cvdSensors.data[n].approachedToReleasedThreshold = ;
		cvdSensors.data[n].approachedToPressedThreshold = ;
		cvdSensors.data[n].pressedToApproachedThreshold = ;*/

		/* 
		 * approachedTimeout is set to 300000 milliseconds (5 minutes)
		 * by default. If a hand is held above the sensor for longer
		 * than this time, the sensor will start recalibrating (even if
		 * the hand is not touching). This is to prevent the sensor
		 * becoming insensitive when another conductive object is placed
		 * close to the sensor).
		 *
		 * Reduce this value to recover sooner in case a button is
		 * accidentally held in approached state due to noise. Set to 0
		 * to disable automatic recalibration after a certain time
		 * altogether.
		 *
		 * Change the value here and observe the difference in time
		 * before recalibration is triggered.
		 */
		cvdSensors.data[n].approachedTimeout = 300000;

		/*
		 * pressedTimeout is set to 5 minutes by default. It is similar
		 * to approachedTimeout above, but for the pressed state.
		 */
		cvdSensors.data[n].pressedTimeout = 300000;

		/*
		 * calibrationTime is set to 100 ms by default. It is the time
		 * the sensor will stay in calibration state. Increasing this
		 * value will result in better estimate of the background value
		 * (the avg value), but at the cost of longer time at startup
		 * and during recalibration.
		 */
		cvdSensors.data[n].calibrationTime = 500;

		/*
		 * preCalibrationTime is set to 100 ms by default. It is the
		 * time the sensor will wait before entering calibration state
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

	/* 
	 * Call cvdSensors.sample() take do a new measurement cycle for all
	 * sensors 
	 */
	cvdSensors.sample();

	/*
	 * Internally, CVDSensors uses many more states than just approached and
	 * pressed. Instead, it distinguises the following states:
	 *   - PreCalibrating
	 *   - Calibrating
	 *   - Released
	 *   - ReleasedToApproached
         *   - Approached
	 *   - ApproachedToPressed
	 *   - ApproachedToReleased
	 *   - Pressed
         *   - PressedToApproached
	 *
	 * The state machine is as follows:
	 *
	 *                     PreCalibrating
	 *                           |
	 *                           V
	 *                      Calibrating <---------------------+
	 *                           |                            |
	 *                           V                            |
	 *                       Released                         |
	 *                      __.    .__                        |
	 *                       /|    |\                         |
	 *                      /        \                        |
	 *                     /        __\|                      |
	 * ApproachedToReleased             ReleasedToApproached  |
	 *                    .__                                 |
	 *                    |\          /                       |
	 *                      \        /                        |
	 *                     __\|    |/__                       |
	 *                       Approached --------------------->*
	 *                      __.    .__                        ^
	 *                       /|    |\                         |
	 *                      /        \                        |
	 *                     /        __\|                      |
         *  PressedToApproached             ApproachedToPressed   |
	 *                    .__                                 |
	 *                    |\          /                       |
	 *                      \        /                        |
	 *                     __\|    |/__                       |
	 *                        Pressed ------------------------+
	 *
	 * Since it is difficult to directly determine from the state if a
	 * button is approached or pressed, the following booleans are available
	 * as well:
	 *   - buttonIsCalibrating: true if state is PreCalibrating or
	 *     Calibrating
	 *   - buttonIsReleased: true if state is Released or
	 *     ReleasedToApproached
	 *   - buttonIsApproached: true if state is Approached,
	 *     ApproachedToPressed, Pressed, PressedToApproached or
	 *     ApproachedToReleased
	 *   - buttonIsPressed: true if state is Pressed or PressedToApproached
	 */

	/* For each button, print current value and button state label */
	for (n = 0; n < N_SENSORS; n++) {
		Serial.print("button[");
		Serial.print(n);
		Serial.print("]: current value: ");
		Serial.print(cvdSensors.data[n].delta);
		Serial.print(", buttonStateLabel: ");
		Serial.print(cvdSensors.data[n].buttonStateLabel);
		if (n < N_SENSORS - 1) {	
			Serial.print("\t ");
		} else {
			Serial.println("");
		}
	}
}
