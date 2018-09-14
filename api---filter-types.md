---
layout: page
title: Filter types
permalink: /api/filter-types/
in_menu: true
weight: 30
in_book: 2
---

Everytime TouchLib completes a sequence of measurements, it converts the series
of measurements into a single sample. The way this is done depends on the type
of filter that is used.

Currently, three types of filters are implemented: an averaging filter, a
slewrate limiting filter and a median filter. The filter type can be set with
the property ```enum FilterType tlSensors.data[<n>].filterType```.

### filterTypeAverage

This is a simple averaging (or more accurately: integrating) filter. It is
useful if there are no (common mode) noise sources and a very sensitive sensor
is required (for example for a distance sensor).

Advantages: no reduction in signal strength.

Disadvantages: sensitive to spikes and common mode noise.

Example: ```tlSensors.data[0].filterType = TLStruct::filterTypeAverage;```

---

### filterTypeSlewrateLimiter

This is a simple slewrate limiting filter. It is useful if your processor has
very little memory (such as ATmega-based boards) and the sensor experiences
(common mode) noise sources or spikes.

Advantages: relatively robust against spikes and (common mode) noise.

Disadvantages: some reduction in signal strength. If a spike occurs during
first measurement of a new sample sequence, an incorrect value could be
produced. This filter therefore requires good debounce filtering (long [timings
for state transitions](../state-machine-timing/)).

Example: ```tlSensors.data[0].filterType = TLStruct::filterTypeSlewrateLimiter;```

---

### filterTypeMedian

This is a median filter. It is useful if your processor has plenty of
memory (such as Teensy or Particle boards) and the sensor experiences
(common mode) noise sources or spikes.

Advantages: no reduction in signal strength, not sensitive to spikes.

Disadvantages: requires a lot of memory and is therefore not available on ATmega boards.

Example: ```tlSensors.data[0].filterType = TLStruct::filterTypeMedian;```

