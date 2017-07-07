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
#define N_SENSORS			2

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

	/* Change approached threshold for each sensor */
	for (n = 0; n < N_SENSORS; n++) {
		/*
		 * Set state transition times to 0 to get more direct feedback
		 * while tuning the threshold levels.
		 */
		tlSensors.data[n].releasedToApproachedTime = 0;
		tlSensors.data[n].approachedToReleasedTime = 0;
		tlSensors.data[n].approachedToPressedTime = 0;
		tlSensors.data[n].pressedToApproachedTime = 0;

		/* 
		 * releasedToApproachedThreshold is the threshold above which
		 * tracking the background value will stop (as it appears that
		 * someone is about to touch the sensor). Its value is a float
		 * and set to 50.0 by default. Set to lower value to make sensor
		 * more sensitive.
		 *
		 * A good initial value is usually 0.5 * the delta value at the
		 * distance that should be considered as "approached".
		 */
		tlSensors.data[n].releasedToApproachedThreshold = 50.0;

		/*
		 * approachedToReleasedThreshold is the threshold below which
		 * tracking the background value will continue again (as the
		 * person who touched the sensor is now far enough removed to
		 * not impact the background anymore). Its value is a float and
		 * set to 40.0 by default. It should usually be a little lower
		 * than releasedToApproachedThreshold to prevent button
		 * instability (~ 10% lower is usually a good initial guess).
		 */
		tlSensors.data[n].approachedToReleasedThreshold = 40.0;

		/* 
		 * approachedToPressedThreshold is the threshold above which a
		 * touch is registered. Its value is set to 150.0 by default.
		 * Set to lower value to make sensor more sensitive. Do not make
		 * it smaller than releasedToApproachedThreshold.
		 *
		 * A good initial value is usually 0.5 * the delta value of the
		 * lightest touch (usually with your little finger) that should
		 * still be registered.
		 */
		tlSensors.data[n].approachedToPressedThreshold = 75.0;

		/*
		 * pressedToApproachedThreshold is the threshold below which a
		 * touch release is registered. Its value is set to 120.0 by
		 * default. It should usually be a little lower than
		 * approachedToPressedThreshold to prevent button instability (~
		 * 10% lower is usually a good initial guess), but do not make
		 * it smaller than approachedToReleasedThreshold.
		 */
		tlSensors.data[n].pressedToApproachedThreshold = 60.0;
	}
}

void loop()
{
	int n;

	/* 
	 * Call tlSensors.sample() take do a new measurement cycle for all
	 * sensors 
	 */
	tlSensors.sample();

	/*
	 * Internally, TLSensors uses many more states than just approached and
	 * pressed. Instead, it distinguises the following states:
	 *   - PreCalibrating
	 *   - Calibrating
	 *   - NoisePowerMeasurement
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
	 *                 NoisePowerMeasurement                  |
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
		Serial.print("]: delta: ");
		Serial.print(tlSensors.getDelta(n));
		Serial.print(", buttonStateLabel: ");
		Serial.print(tlSensors.getStateLabel(n));
		if (n < N_SENSORS - 1) {	
			Serial.print("\t ");
		} else {
			Serial.println("");
		}
	}
}
