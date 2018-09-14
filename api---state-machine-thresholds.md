---
layout: page
title: State machine thresholds
permalink: /api/state-machine-thresholds/
in_menu: true
weight: 40
in_book: 2
---

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