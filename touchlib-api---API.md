---
layout: page
title: API
permalink: /touchlib-api/api/
in_menu: true
weight: 30
in_book: 2
---

State machine properties
----

## State machine transition thresholds

### releasedToApproachedThreshold

```C++
int32_t tlSensors.data[<n>].releasedToApproachedThreshold
```

If a sensor is in released state and the `delta` value is more than this
threshold, the state is changed to `releasedToApproached`.

---

### approachedToReleasedThreshold

```C++
int32_t tlSensors.data[<n>].approachedToReleasedThreshold
```
If a sensor is in appraoched state and the `delta` value is less than this
threshold, the state is changed to `approachedToReleased`. This value should
always be less than or equal to `releasedToApproachedThreshold`. Typically it
is about 10% smaller.

---

### approachedToPressedThreshold

```C++
int32_t tlSensors.data[<n>].approachedToPressedThreshold
```
If a sensor is in approached state and the `delta` value is more than this
threshold, the state is changed to `approachedToPressed`. This value should
always be larger than or equal to releasedToApproachedThreshold.

---

### pressedToApproachedThreshold

```C++
int32_t tlSensors.data[<n>].pressedToApproachedThreshold
```
If a sensor is in pressed state and the `delta` value is less than this
threshold, the state is changed to `pressedToApproached`. This value should
always be less than or equal to `approachedToPressedThreshold`. Typically it is
about 10% smaller.

---

## State machine transition timing

### releasedToApproachedTime

```C++
uint32_t tlSensors.data[<n>].releasedToApproachedTime
```
If a sensor is in releasedToApproached state and the `delta` value is more
than `releasedToApproachedThreshold` for longer than `releasedToApproachedTime`
milliseconds, the state is changed to `approached`.

On the other hand, if a sensor is in releasedToApproached state and during
`releasedToApproachedTime` milliseconds there is at least one sample that has a
`delta` value less than `releasedToApproachedThreshold`, the state is changed
back to released.

Default: 10 ms

---

### approachedToPressedTime

```C++
uint32_t tlSensors.data[<n>].approachedToPressedTime
```
If a sensor is in approachedToPressed state and the `delta` value is more
than `approachedToPressedThreshold` for longer than `approachedToPressedTime`
milliseconds, the state is changed to `pressed`. 

On the other hand, if a sensor is in approachedtoPressed state and during
`approachedToPressedTime` milliseconds there is at least one sample that has a
`delta` value less than `approachedToPressedThreshold`, the state is changed
back to approached.

Default: 10 ms

---

### pressedToApproachedTime

```C++
uint32_t tlSensors.data[<n>].pressedToApproachedTime
```
If a sensor is in pressedToApproached state and the `delta` value is less
than `pressedToApproachedThreshold` for longer than `pressedToApproachedTime`
milliseconds, the state is changed to `approached`.

On the other hand, if a sensor is in pressedToApproached stated and during
`pressedToApproachedTime` milliseconds there is at least one sample that has a
`delta` value more than `pressedToApproachedThreshold`, the state is changed
back to pressed.

Default: 10 ms

---

### approachedToReleasedTime

```C++
uint32_t tlSensors.data[<n>].approachedToReleasedTime
```
If a sensor is in approachedToReleased state and the `delta` value is less
than `approachedToReleasedThreshold` for longer than `approachedToReleasedTime`
milliseconds, the state is changed to `released`.

On the other hand, if a sensor is in approachedToReleased state and during
`approachedToReleasedTime` milliseconds there is at least one sample that has a
`delta` value more than `approachedToReleasedThreshold`, the state is changed
back to approached.

Default: 10 ms

---

### preCalibrationTime

```C++
uint32_t tlSensors.data[<n>].preCalibrationTime
```
If a sensor is in preCalibrating state, it will automatically transfer to
calibrating state after `preCalibrationTime` milliseconds. This allows for
settling of power supplies etc.

Default: 100 ms

---

### calibrationTime

```C++
uint32_t tlSensors.data[<n>].calibrationTime
```
If a sensor is in calibrating state, it will transfer to noisePowerMeasurement
state after `calibrationTime` milliseconds. During this time, call the
`sample()` function as often as possible so TouchLib can determine the
background level of this sensor.

Default: 500 ms

### approachedTimeout

```C++
uint32_t tlSensors.data[<n>].approachedTimeout
```
To prevent stuck buttons, a sensor that is in approached state for longer than
`approachedTimeout` milliseconds, will be forced to go to calibrating state.

Default: 5 minutes

---


### pressedTimeout

```C++
uint32_t tlSensors.data[<n>].pressedTimeout
```
To prevent stuck buttons, a sensor that is in pressed state for longer than
`pressedTimeout` milliseconds, will be forced to go to calibrating state.

Default: 5 minutes

---

## Properties of sample methods

### pin

```C++
tlSensors.data[<n>].tlStructSampleMethod.<sampleMethod>.pin
```

---

## Functions

### initialize

```C++
int initialize(uint8_t channel, int (*sampleMethod)(struct TLStruct *d, uint8_t nSensors, uint8_t ch))
```
Inizialize channel `ch` with sample method `sampleMethod`. `sampleMethod` can be:
* `TLSampleMethodCVD`: for capacitive sensing with CVD method function (if CVD is implemented for the target platform)
* `TLSampleMethodTouchRead`: for capacitive sensing with native touchRead() function (if supported by the target platform)
* `TLSampleMethodResistive`: for resistive sensing with native analogRead() function

See [supported boards](../../getting-started/supported-boards/) for which methods are supported on your hardware.

Return value: 0 on success, negative value on error.

---

### anyButtonIsCalibrating

```C++
bool anyButtonIsCalibrating()
```
Returns true if one or more channels are in calibrating state. Repeatedly call
the `sample()` function until no more channels are in calibrating state.

---

### sample

```C++
uint8_t sample()
```
Perfoms a complete cycle of scanning all channels and updating button state machines.

---

### isCalibrating

```C++
bool isCalibrating(int n)
```
Returns true if channel `n` is in calibrating state.

---

### isReleased

```C++
bool isReleased(int n)
```
Returns true if channel `n` is in released state.

---

### isApproached

```C++
bool isApproached(int n)
```
Returns true if channel `n` is in approached state.

---

### isPressed

```C++
bool isPressed(int n)
```
Returns true if channel `n` is in pressed state.

---

### getSensorWithLargestDelta

```C++
int getSensorWithLargestDelta()
```
If one or more sensors are in pressed state, this function returns the sensor
with the largest delta level. If no sensors are pressed, it returns -1.

---

## Callbacks

### buttonStateChangeCallback

```C++
void buttonStateChangeCallback(int ch, enumTLStruct::ButtonState oldState, enum TLStruct::ButtonState newState)
```
Called whenever a button changes to a major state.

```ch```: sensor number

```oldState```: previous state

```newState```: current state

---

### buttonMeasurementProgressCallback

```C++
int (*buttonMeasurementProgressCallback)(uint16_t idx, uint8_t ch, bool isStarted)
```
Called whenever a new measurement of a channel is started or finished.

```idx```: index in scanning order array

```ch```: sensor number

```isStarted```: true when a measurement is started, false when a measurement is finished

---

### sequenceMeasurementProgressCallback

```C++
int (*sequenceMeasurementProgressCallback)(bool isStarted)
```
Called whenever a new sequence of measurements is started or finished.

```isStarted```: true when a measurement is started, false when a measurement is finished
