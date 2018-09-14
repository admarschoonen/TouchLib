---
layout: page
title: Properties of sample methods
permalink: /api/properties-of-sample-methods/
in_menu: true
weight: 40
in_book: 2
---

Besides general properties such as [state machine
thesholds](../state-machine-thresholds) or [state machine
timing](../state-machine-timing) there exist also properties which are specific
to a certain sample method. These properties can be accessed 
via ```tlSensors.data[<n>].tlStructSampleMethod.<sampleMethod>.<property>```,
for example ```tlSensors.data[0].tlStructSampleMethod.touchRead.pin```.

Sample method Resistive
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

