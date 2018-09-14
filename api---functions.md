---
layout: page
title: Functions
permalink: /api/functions/
in_menu: true
weight: 50
in_book: 2
---

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
