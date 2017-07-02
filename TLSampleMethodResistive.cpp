/*
 * TLSampleMethodResistive.cpp - Resistive sensing implementation for
 * TouchLibrary for Arduino Teensy 3.x
 * 
 * https://github.com/AdmarSchoonen/CVDSensor
 * Copyright (c) 2016 - 2017 Admar Schoonen
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "CVDSensor.h"
#include "TLSampleMethodResistive.h"

int TLSampleMethodResistive(struct CvdStruct * data, uint8_t nSensors, uint8_t ch,
		bool inv)
{
	struct CvdStruct * dCh;
	int ch_pin, gnd_pin, sample;
	bool useInternalPullup;
	
	if (inv) {
		/* Pseudo differential measurements are not supported */
		sample = 0;
	} else {
		dCh = &(data[ch]);
		ch_pin = dCh->pin;
		gnd_pin = dCh->sampleMethodResistive_gndPin;
		useInternalPullup =
			dCh->sampleMethodResistive_useInternalPullup;

		if (useInternalPullup) {
			/* Enable internal pull-up on analog input */
			//digitalWrite(ch_pin, HIGH);
			digitalWrite(ch_pin, INPUT_PULLUP);
		} else {
			/* Disable internal pull-up on analog input */
			digitalWrite(ch_pin, INPUT);
		}
		
		if (gnd_pin >= 0) {
			/* Configure gnd_pin as digital output, low (gnd) */
			pinMode(gnd_pin, OUTPUT);
			digitalWrite(gnd_pin, LOW);
		}

		/* Read */
		sample = analogRead(ch_pin);

		/* Disable internal pull-up on analog input */
		//digitalWrite(ch_pin, LOW);
		digitalWrite(ch_pin, INPUT);
		
		if (gnd_pin >= 0) {
			/* Leave gnd_pin floating */
			pinMode(gnd_pin, INPUT);
			digitalWrite(gnd_pin, LOW);
		}
	}

	return sample;
}
