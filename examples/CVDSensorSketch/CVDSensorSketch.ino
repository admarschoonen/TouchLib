#include <CVDSensor.h>

/*
 * CVDSense Library Demo Sketch
 * Admar Schoonen 2016
 * Connect an electrode (piece of metal sheet / foil) to an analog pin
 */

/* 
 * Number of capacitive sensors. Needs a minimum of 2. When using only one
 * sensor, set N_SENSORS to 2 and use an unused analog input pin for the second
 * sensor.
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
	Serial.begin(115200);

	cvdSensors.data[0].pin = 0; /* Analog pin 0 */
	cvdSensors.data[0].enableSlewrateLimiter = false;
	cvdSensors.data[0].sampleType = CvdStruct::sampleTypeNormal;
	cvdSensors.data[0].setParallelCapacitanceManually = true;

	cvdSensors.data[1].pin = 1; /* Analog pin 1 */
	cvdSensors.data[1].enableSlewrateLimiter = false;
	cvdSensors.data[1].sampleType = CvdStruct::sampleTypeNormal;
	cvdSensors.data[1].setParallelCapacitanceManually = true;

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
	Serial.print(cvdSensors.data[0].nCharges);
	Serial.print(", \tcounter: ");
	Serial.print(cvdSensors.data[0].counter);*/

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
	Serial.print(cvdSensors.data[1].nCharges);
	/*Serial.print(", \tcounter: ");
	Serial.print(cvdSensors.data[1].counter);*/
	Serial.print("; \tLoop time: ");
	now = millis();
	Serial.println(now - prev);
	prev = now;

	//delay(100);
}
