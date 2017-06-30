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

	for (n = 0; n < N_SENSORS; n++) {
		/*
		 * By default, the slewrate limiting filter is selected. This is
		 * a very robust filter especially in case of conducted noise
		 * (when the Arduino is supplied with a phone charger for
		 * example).
		 *
		 * However, it is also a bit slow and can make the sensor less
		 * sensitive to small changes. It is therefore not very well
		 * suited if the sensor is used as a distance sensor. If the
		 * system is battery powered or if the system ground is
		 * connected to earth ground, an exponential filter can be used
		 * instead.
		 *
		 * Select an exponential filter by disabling the slewrate
		 * limiter.
		 */
		cvdSensors.data[n].enableSlewrateLimiter = false;

		/*
		 * The exponential filter can be controlled with parameter
		 * filterCoeff. The default value is 128. Set to a larger value
		 * to use stronger filtering. Stronger filtering results in more
		 * accurate average calculation but it also makes the system
		 * slower to adjust to small changes in the background signal.
		 */
		cvdSensors.data[n].filterCoeff = 16;
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
	 * For each button, print current value, background value and button
 	 * state label
	 */
	for (n = 0; n < N_SENSORS; n++) {
		Serial.print("button[");
		Serial.print(n);
		Serial.print("]: current value: ");
		Serial.print(cvdSensors.data[n].delta);
		Serial.print(", background value: ");
		Serial.print(cvdSensors.data[n].avg);
		Serial.print(", buttonStateLabel: ");
		Serial.print(cvdSensors.data[n].buttonStateLabel);
		if (n < N_SENSORS - 1) {	
			Serial.print("\t ");
		} else {
			Serial.println("");
		}
	}
}
