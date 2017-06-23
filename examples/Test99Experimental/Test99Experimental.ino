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
#define N_SENSORS			2

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

	cvdSensors.data[0].pin = 0; /* Analog pin 0 */
	cvdSensors.data[0].setParallelCapacitanceManually = false;

	cvdSensors.data[1].pin = 1; /* Analog pin 1 */
	cvdSensors.data[1].setParallelCapacitanceManually = false;

	cvdSensors.printScanOrder();
}

void loop()
{
	static int prev = millis();
	int now;

	cvdSensors.sample();
	/*Serial.print("raw[0]: ");
	Serial.print(cvdSensors.data[0].raw);*/
	Serial.print("; \t\tcapacitance[0]: ");
	Serial.print(cvdSensors.data[0].capacitance);
	Serial.print(", \tavg: ");
	Serial.print(cvdSensors.data[0].avg);
	Serial.print(", \tdelta: ");
	Serial.print(cvdSensors.data[0].delta);
	Serial.print(", \tstate: ");
	Serial.print(cvdSensors.data[0].buttonState);
	/*Serial.print(", \tnCharges: ");
	Serial.print(cvdSensors.data[0].nCharges);*/
	/*Serial.print(", \tpcap: ");
	Serial.print(cvdSensors.data[0].parallelCapacitance);*/

	/*Serial.print("; \t\traw[1]: ");
	Serial.print(cvdSensors.data[1].raw);*/
	Serial.print("; \t\tcapacitance[1]: ");
	Serial.print(cvdSensors.data[1].capacitance);
	Serial.print(", \tavg: ");
	Serial.print(cvdSensors.data[1].avg);
	Serial.print(", \tdelta: ");
	Serial.print(cvdSensors.data[1].delta);
	Serial.print(", \tstate: ");
	Serial.print(cvdSensors.data[1].buttonState);
	/*Serial.print(", \tnCharges: ");
	Serial.print(cvdSensors.data[1].nCharges);*/
	/*Serial.print(", \tpcap: ");
	Serial.print(cvdSensors.data[1].parallelCapacitance);*/
	Serial.print("; \tLoop time: ");
	now = millis();
	Serial.println(now - prev);
	prev = now;

	//delay(100);
}
