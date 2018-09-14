---
layout: page
title: Callbacks
permalink: /api/callbacks/
in_menu: true
weight: 60
in_book: 2
---

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
void (*buttonMeasurementProgressCallback)(uint16_t idx, uint8_t ch, bool isStarted)
```
Called whenever a new measurement of a channel is started or finished.

```idx```: index in scanning order array

```ch```: sensor number

```isStarted```: true when a measurement is started, false when a measurement is finished

---

### sequenceMeasurementProgressCallback

```C++
void (*sequenceMeasurementProgressCallback)(bool isStarted)
```
Called whenever a new sequence of measurements is started or finished.

```isStarted```: true when a measurement is started, false when a measurement is finished
