---
layout: page
title: General properties
permalink: /api/general-properties/
in_menu: true
weight: 20
in_book: 2
---

### scanOrder

```C++
uint8_t scanOrder[N_SENSORS * N_MEASUREMENTS_PER_SENSOR]
```

```scanOrder``` defines the order in which the sensors will be measured.
TouchLib combines N_MEASUREMENTS_PER_SENSOR measurements in one sample to
reduce noise. The order in which the sensors are measured is determined on
startup by a random generator and determines on the number of sensors
(```N_SENSORS```) and the number of measurements per sensor
(```N_MEASUREMENTS_PER_SENSOR```, typically 8 or 16).

The random order will reduce the influence of periodic noise sources. Since the
order is random but deterministic (determined by ```N_SENSORS``` and
```N_MEASUREMENTS_PER_SENSOR```), it will always be the same for all boards if
```N_SENSORS``` and ```N_MEASUREMENTS_PER_SENSOR``` is unchanged.

This also means that if there is only one sensor, there is no benefit of the
random order. If a system with only one sensor experiences interference from
(periodic) noise sources, it can therefore be beneficial to add a few dummy
sensors.

---

### calibratedMaxDelta

```C++
int tlSensors.data[<n>].calibratedMaxDelta
```

Maximum delta value that was recorded during semi-automatic tuning. This value is only used in the ```printBar``` function.
