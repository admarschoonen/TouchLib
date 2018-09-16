---
layout: page
title: State machine timing
permalink: /api/state-machine-timing/
in_menu: true
weight: 50
in_book: 2
---

The properties listed on this page determine the timing upon which the state
machine determines to change a sensor from one state to the next. The state
transition diagram is shown here as a reminder; for more information on the
state machine see FIXME.

![state transition diagram](../../state-machine.svg)

---

### preCalibrationTime

```C++
uint32_t tlSensors.data[<n>].preCalibrationTime
```
If a sensor is in `preCalibrating` state, it will automatically transfer to
`calibrating` state after `preCalibrationTime` milliseconds. This allows for
settling of power supplies etc.

Default: 100 ms

---

### calibrationTime

```C++
uint32_t tlSensors.data[<n>].calibrationTime
```
If a sensor is in `calibrating` state, it will transfer to `noisePowerMeasurement`
state after `calibrationTime` milliseconds. During this time, call the
`sample()` function as often as possible so TouchLib can determine the
background level of this sensor.

Default: 500 ms

### releasedToApproachedTime

```C++
uint32_t tlSensors.data[<n>].releasedToApproachedTime
```
If a sensor is in `releasedToApproached` state and the `delta` value is more
than `releasedToApproachedThreshold` for longer than `releasedToApproachedTime`
milliseconds, the state is changed to `approached`.

On the other hand, if a sensor is in `releasedToApproached` state and during
`releasedToApproachedTime` milliseconds there is at least one sample that has a
`delta` value less than `releasedToApproachedThreshold`, the state is changed
back to `released`.

Default: 10 ms

---

### approachedToPressedTime

```C++
uint32_t tlSensors.data[<n>].approachedToPressedTime
```
If a sensor is in `approachedToPressed` state and the `delta` value is more
than `approachedToPressedThreshold` for longer than `approachedToPressedTime`
milliseconds, the state is changed to `pressed`. 

On the other hand, if a sensor is in `approachedtoPressed` state and during
`approachedToPressedTime` milliseconds there is at least one sample that has a
`delta` value less than `approachedToPressedThreshold`, the state is changed
back to `approached`.

Default: 10 ms

---

### pressedToApproachedTime

```C++
uint32_t tlSensors.data[<n>].pressedToApproachedTime
```
If a sensor is in `pressedToApproached` state and the `delta` value is less
than `pressedToApproachedThreshold` for longer than `pressedToApproachedTime`
milliseconds, the state is changed to `approached`.

On the other hand, if a sensor is in `pressedToApproached` stated and during
`pressedToApproachedTime` milliseconds there is at least one sample that has a
`delta` value more than `pressedToApproachedThreshold`, the state is changed
back to `pressed`.

Default: 10 ms

---

### approachedToReleasedTime

```C++
uint32_t tlSensors.data[<n>].approachedToReleasedTime
```
If a sensor is in `approachedToReleased` state and the `delta` value is less
than `approachedToReleasedThreshold` for longer than `approachedToReleasedTime`
milliseconds, the state is changed to `released`.

On the other hand, if a sensor is in `approachedToReleased` state and during
`approachedToReleasedTime` milliseconds there is at least one sample that has a
`delta` value more than `approachedToReleasedThreshold`, the state is changed
back to `approached`.

Default: 10 ms

---

### approachedTimeout

```C++
uint32_t tlSensors.data[<n>].approachedTimeout
```
To prevent stuck buttons, a sensor that is in `approached` state for longer than
`approachedTimeout` milliseconds, will be forced to go to `calibrating` state.

Default: 5 minutes

---


### pressedTimeout

```C++
uint32_t tlSensors.data[<n>].pressedTimeout
```
To prevent stuck buttons, a sensor that is in `pressed` state for longer than
`pressedTimeout` milliseconds, will be forced to go to `calibrating` state.

Default: 5 minutes

