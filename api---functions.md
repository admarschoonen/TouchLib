---
layout: page
title: Functions
permalink: /api/functions/
in_menu: true
weight: 70
in_book: 2
---

### initialize

```C++
int initialize(uint8_t channel, int (*sampleMethod)(struct TLStruct *d, uint8_t nSensors, uint8_t ch))
```
Inizialize channel `ch` with sample method `sampleMethod`. `sampleMethod` can be:
* `TLSampleMethodCVD`: for capacitive sensing with CVD method function (if CVD is implemented for the target platform)
* `TLSampleMethodTouchRead`: for capacitive sensing with native `touchRead()` function (if supported by the target platform)
* `TLSampleMethodResistive`: for resistive sensing with native `analogRead()` function

See [supported boards](../../getting-started/supported-boards/) for which methods are supported on your hardware.

Return value: 0 on success, negative value on error.

---

### anyButtonIsCalibrating

```C++
bool anyButtonIsCalibrating()
```
Returns true if one or more sensors are in calibrating state. 

A common usage is to repeatedly call the `sample` function in the ```setup``` function until no more
channels are in calibrating state:

```C++
void setup() 
{
	... 
	
	/* configure sensors here */

	...

	while (tlSensors.anyButtonIsCalibrating()) {
		tlSensors.sample();
	}
}
```

---

### sample

```C++
uint8_t sample()
```
Perfoms a complete cycle of scanning all sensors and updating button state machines.

---

### isCalibrating

```C++
bool isCalibrating(int n)
```
Returns true if sensor `n` is in calibrating state.

---

### isReleased

```C++
bool isReleased(int n)
```
Returns true if sensor `n` is in released state.

---

### isApproached

```C++
bool isApproached(int n)
```
Returns true if sensor `n` is in approached state.

---

### isPressed

```C++
bool isPressed(int n)
```
Returns true if sensor `n` is in pressed state.

---

### getRaw

```C++
uint32_t getRaw(int n)
```

Returns the raw value of sensor `n`.

---

### getValue

```C++
int32_t getValue(int n)
```

Returns the converted value of sensor `n`.

---

### getAvg

```C++
int32_t getAvg(int n)
```

Returns the average value (background value) of sensor `n`.

---

### getDelta

```C++
int32_t getDelta(int n)
```

Returns the delta value of sensor `n`. The delta value is the difference of the
converted value and the average value and is thus a measure for how much the
sensor has changed with respect to the background value.

---

### getSensorWithLargestDelta

```C++
int getSensorWithLargestDelta()
```
If one or more sensors are in pressed state, this function returns the sensor
with the largest delta level. If no sensors are pressed, it returns -1.

---

### printBar
```C++
int printBar(uint8_t ch_k, int length)
```

Prints a visualization of the current delta value of the sensor on the serial
monitor. The maximum value is determined by the ```calibratedMaxDelta```
property of the sensor and the actual mapping is specific to the sample method.

