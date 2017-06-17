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
	Serial.begin(115200);
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
	 *                      Calibrating
	 *                           |
	 *                           V
	 *                       Released
	 *                      __.    .__
	 *                       /|    |\
	 *                      /        \
	 *                     /        __\|
	 * ApproachedToReleased             ReleasedToApproached
	 *                    .__
	 *                    |\          /
	 *                      \        /
	 *                     __\|    |/__
	 *                       Approached
	 *                      __.    .__
	 *                       /|    |\
	 *                      /        \
	 *                     /        __\|
         *  PressedToApproached             ApproachedToPressed
	 *                    .__
	 *                    |\          /
	 *                      \        /
	 *                     __\|    |/__
	 *                        Pressed
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
