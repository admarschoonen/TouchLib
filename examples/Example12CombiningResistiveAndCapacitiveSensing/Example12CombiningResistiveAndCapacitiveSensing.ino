#include <CVDSensor.h>

/*
 * CVDSense Library Demo Sketch
 * Admar Schoonen 2016
 * Connect an electrode (piece of metal sheet / foil) to an analog pin
 */

/* 
 * To make 1 set of capacitive + resistive sensors:
 *
 * Take 2 pieces of conductive fabric and 1 piece of ESD foam or velostat.
 * Attach the conductive fabric to the foam / velostat; make sure not to put too
 * much tension on the foam / velostat or performance of the resistive sensor
 * will be very limited.
 *
 * Attach one electrode to A0 and the other electrode to pin 2.
 *
 * Repeat this again for a second set of capacitive + resistive sensors; connect
 * this to A1 and (again) pin 2.
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

	/* 
	 * By default, capacitive sensors on A0 - A3 are assumed. Thus, for the
	 * capacitive part of our sensors (sensors 0 and 1) we don't need to
	 * change the pin settings or sample method.
	 *
	 * However, we do want to automatically trigger a recalibration of the
	 * resistive sensor if the capacitive sensor is released.
	 */

	cvdSensors.data[0].forceCalibrationWhenApproachingFromPressed = 
		(1 << 2); // Recalibrate sensor 2 if sensor 0 is released

	cvdSensors.data[1].forceCalibrationWhenApproachingFromPressed = 
		(1 << 3); // Recalibrate sensor 3 if sensor 1 is released

	/*
	 * The resistive part of our sensors are sensors 2 and 3. By default,
	 * the library assumes these are capacitive sensors connected to A2 and
	 * A3. However, for our capacitive + resistive sensors, they are
	 * connected to A0 and A1.
	 *
	 * The other side of the resistive part of the sensor is connected to
	 * pin 2 respectively pin 3.
	 */
	cvdSensors.data[2].pin = A0;
	cvdSensors.data[2].sampleMethodResistive_gndPin = 2;
	cvdSensors.data[2].sampleMethod = TLSampleMethodResistive;

	cvdSensors.data[3].pin = A1;
	cvdSensors.data[3].sampleMethodResistive_gndPin = 3;
	cvdSensors.data[3].sampleMethod = TLSampleMethodResistive;

	/* We also need to adjust the thresholds for the resistive sensors. */
	cvdSensors.data[2].releasedToApproachedThreshold = 100.0;
	cvdSensors.data[2].approachedToReleasedThreshold = 80.0;
	cvdSensors.data[2].approachedToPressedThreshold = 300.0;
	cvdSensors.data[2].pressedToApproachedThreshold = 240.0;

	cvdSensors.data[3].releasedToApproachedThreshold = 100.0;
	cvdSensors.data[3].approachedToReleasedThreshold = 80.0;
	cvdSensors.data[3].approachedToPressedThreshold = 300.0;
	cvdSensors.data[3].pressedToApproachedThreshold = 240.0;
}

void loop()
{
	cvdSensors.sample();

	Serial.print("raw[0]: ");
	Serial.print(cvdSensors.getRaw(0));
	Serial.print(", \tdelta: ");
	Serial.print(cvdSensors.getDelta(0));
	Serial.print(", \tstate: ");
	Serial.print(cvdSensors.getState(0));

	Serial.print("; \t\traw[1]: ");
	Serial.print(cvdSensors.getRaw(1));
	Serial.print(", \tdelta: ");
	Serial.print(cvdSensors.getDelta(1));
	Serial.print(", \tstate: ");
	Serial.print(cvdSensors.getState(1));

	Serial.print("; \t\traw[2]: ");
	Serial.print(cvdSensors.getRaw(2));
	Serial.print(", \tdelta: ");
	Serial.print(cvdSensors.getDelta(2));
	Serial.print(", \tstate: ");
	Serial.print(cvdSensors.getState(2));

	Serial.print("; \t\traw[3]: ");
	Serial.print(cvdSensors.getRaw(3));
	Serial.print(", \tdelta: ");
	Serial.print(cvdSensors.getDelta(3));
	Serial.print(", \tstate: ");
	Serial.println(cvdSensors.getState(3));
}
