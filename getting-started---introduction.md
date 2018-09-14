---
layout: page
title: Introduction
permalink: /getting-started/introduction/
in_menu: true
weight: 10
in_book: 1
---

[TouchLib](https://github.com/admarschoonen/TouchLib/) is a generic touch
library for Arduino and Arduino-like boards and can be used for both resistive
and capacitive touch. It features advanced [signal processing
techniques](../../signal-processing/introduction/) and an extended button state
machine as well as a code generator to tune your sensors and create an Arduino
program tuned for your sensors that you can use as starting point for your next
touch project.

To get started with TouchLib you need an Arduino UNO board or one of the other
[supported boards](../supported-boards/) and one or more capacitive (or
resistive) sensors. If you don't have a sensor, you can easily make your own
[capacitive](../making-a-capacitive-sensor/) or [resistive
sensor](../making-a-resistive-sensor/).

Note that while Arduino UNO boards do work and are supported, they have only
limited memory and processor power. Since other boards such as Teensy 3.x,
Particle Photon and ESP32 boards have much more memory and faster processors,
it's recommended to use these or similar boards instead. See [supported
boards](../supported-boards/) for more details.

On the software side you will probably want the latest release from TouchLib. If you use
a Particle Photon board, you can simply search for TouchLib in the library
browser of the IDE on [particle.io](https://build.particle.io/build/new). If you
use an Arduino UNO, Mega or Teensy 3.x board, download the latest stable release
from [github.com](https://github.com/admarschoonen/TouchLib/releases).

If you are new to capacitive sensing and want some more background on how they
work and in which cases they won't work (as well), please start with the
[theory of capacitive sensors](../theory-of-capacitive-sensors/). If you are
already familiar with capacitive sensing, you can skip ahead to making your own
[capacitive](../making-a-capacitive-sensor/) or [resistive
sensor](../making-a-resistive-sensor/) sensor or directly to [tuning a
sensor](../tuning-a-sensor/) or [using TouchLib in a real
project](../using-touchlib-in-a-real-project/).

