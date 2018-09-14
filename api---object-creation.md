---
layout: page
title: Object creation
permalink: /api/object-creation/
in_menu: true
weight: 15
in_book: 2
---

To use TouchLib in your project, you need to create a TouchLib object. This
object contains all the sensors that you want to use with TouchLib as well as
all related parameters and functions. Note that creating multiple TouchLib
objects in one project is not supported; instead it is better to add the
sensors to the existing object.

Creating a TouchLib object is easy:

```C++
TLSensors<N_SENSORS, N_MEASUREMENTS_PER_SENSOR> tlSensors;
```

Here, `N_SENSORS` is the number of sensors you want to use and
`N_MEASUREMENTS_PER_SENSOR` is the number of measurements TouchLib should take
to create one sample.

A larger value of `N_MEASUREMENTS_PER_SENSOR` allows for more noise reduction
and better spreading of interfering signals (see `scanOrder` in
[General Properties](../general-properties/)) but also takes more time. Typical
values of `N_MEASUREMENTS_PER_SENSOR` are 8 or 16.

