#include <EEPROM.h>
#include <CVDSensor.h>

/*
 * CVDSense Library Demo Sketch
 * Admar Schoonen 2016
 * Connect an electrode (piece of metal sheet / foil) to an analog pin
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

	cvdSensors.data[0].pin = A0; /* Analog pin 0 */
	cvdSensors.data[0].setParallelCapacitanceManually = false;

	cvdSensors.data[1].pin = A1; /* Analog pin 1 */
	cvdSensors.data[1].setParallelCapacitanceManually = false;

	cvdSensors.data[2].pin = A3; /* Analog pin 3 */
	cvdSensors.data[2].setParallelCapacitanceManually = false;
	//cvdSensors.data[2].releasedToApproachedThreshold = 400.0;
	//cvdSensors.data[2].approachedToReleasedThreshold = 200.0;
	cvdSensors.data[2].approachedToPressedThreshold = 1400.0;
	cvdSensors.data[2].pressedToApproachedThreshold = 1300.0;

	/*
	 * When this button switches from pressed-to-approached to approached,
	 * recalibrate button 3 to reduce impact of hysteresis in the foam.
	 */
	cvdSensors.data[2].forceCalibrationWhenApproachingFromPressed = 
		(1 << 3);

	/*
	 * When this button switches from approached-to-released to released,
	 * recalibrate this button to reduce impact of hysteresis in the foam.
	 */
	//cvdSensors.data[2].forceCalibrationWhenReleasingFromApproached =
	//	(1 << 2);

	cvdSensors.data[3].pin = A3; /* Analog pin 3 */
	cvdSensors.data[3].sampleMethod = TLSampleMethodResistive;
	cvdSensors.data[3].sampleMethodResistive_gndPin = 2;

	cvdSensors.printScanOrder();
}

void loop()
{
	static int prev = millis();
	int now;

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

	//delay(100);
}
