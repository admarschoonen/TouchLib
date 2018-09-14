---
layout: page
title: Properties of sample methods
permalink: /api/properties-of-sample-methods/
in_menu: true
weight: 60
in_book: 2
---

Besides general properties there exist also properties which are specific
to a certain sample method. These properties can be accessed 
via ```tlSensors.data[<n>].tlStructSampleMethod.<sampleMethod>.<property>```,
for example ```tlSensors.data[0].tlStructSampleMethod.touchRead.pin```.

Sample method CVD
---

### pin

```C++
int tlSensors.data[<n>].tlStructSampleMethod.CVD.pin
```

Pin to which the sensor is connected.

---

### useNChargesPadding

```C++
bool tlSensors.data[<n>].tlStructSampleMethod.CVD.useNChargesPadding
```

Set to true to ensure that the CVD method always takes the same amount of time.
This means that TouchLib will always use ```nChargesMax``` measurements, even
if fewer measurements would be sufficient.

Default: true.

---

### nChargesMin

```C++
bool tlSensors.data[<n>].tlStructSampleMethod.CVD.nChargesMin
```

Minimum number of measurements that the CVD method should take.

Default: depends on processor; see table below.

---

### nChargesMax

```C++
bool tlSensors.data[<n>].tlStructSampleMethod.CVD.nChargesMax
```

Maximum number of measurements that the CVD method should take.

Default: depends on processor; see table below.

---

### chargeDelaySensor

```C++
bool tlSensors.data[<n>].tlStructSampleMethod.CVD.chargeDelaySensor
```

Delay in microseconds during charging or discharging the sensor.

Default: depends on processor; see table below.

---

### chargeDelayADC

```C++
bool tlSensors.data[<n>].tlStructSampleMethod.CVD.chargeDelayADC
```

Delay in microseconds during charging or discharging the sensor.

Default: depends on processor; see table below.

---

| Processor | default for nChargesMin | default for nChargesMax | default for chargeDelaySensor | default for chargeDelayADC |
|-----|-----|-----|-----|-----|
| ATmega (Arduino UNO, Mega, Lilypad USB etc) | 1 | 1 | 0 | 0 |
| STM32F20x (Particle Photon etc) | 1 | 1 | 0 | 0 |
| Freescale MK20DX256 (Teensy 3.2) | 4 | 4 | 0 | 0 |

---

Sample method resistive
---

### pin

```C++
int tlSensors.data[<n>].tlStructSampleMethod.resistive.pin
```

Pin to which the sensor is connected.

---

### gndPin

```C++
int tlSensors.data[<n>].tlStructSampleMethod.resistive.gndPin
```

Input / output pin to which the negative side of the sensor is connected. Set
to -1 if the negative side is not connected to a pin but always connected to
groud instead.

If a resistive sensor is also to be used as a capacitive sensor, the negative
side must always be connected to an input / output pin so the Arduino can set
this pin as input (floating) in order to use it as a capacitive sensor and set
as output (low / ground) in order to use it as a resistive sensor.

---

### useInternalPullup

```C++
bool tlSensors.data[<n>].tlStructSampleMethod.resistive.useInternalPullup
```

Set to true if the internal pull up on the analog input of the Arduino should
be used for this sensor.

If a resistive sensor is also to be used as a capacitive sensor, this must be
set to true and no external pull up should be connected. The Arduino can then
disable the internal pull up this (make the pin floating) in order to use it as
a capacitive sensor and enable it to use it as a resistive sensor.

---

### valueMax

```C++
int32_t valueMax
```

Maximum value that sensor should report. If actual value of the sensor is
larger than this, the value will be clipped to ```valueMax```. 

This property is currently not used.


---

Sample method touchRead
---

### pin

```C++
int tlSensors.data[<n>].tlStructSampleMethod.touchRead.pin
```

Pin to which the sensor is connected.

