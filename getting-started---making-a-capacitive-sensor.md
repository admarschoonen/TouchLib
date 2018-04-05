---
layout: page
title: Making a capacitive sensor
permalink: /getting-started/making-a-capacitive-sensor/
in_menu: true
weight: 25
in_book: 1
---

<script type="text/javascript" async
  src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-MML-AM_CHTML">
</script>


Basic capacitive sensor
-----

A basic capacitive touch sensor is shown below. This is a large sensor meant to
be touched with your whole hand or used as distance sensor. It is made out of a
10 x 10 cm circuit board with copper on both sides.
![basic-capacitive-touch-sensor](../../basic-capacitive-touch-sensor.jpg)

Note that this sensor has no insulation and touching it directly can confuse
TouchLib and might even damage your electronics (read the
[theory](../theory-of-capacitive-sensors/) if you want to know why). To use
this sensor in an actual application, always use some non-conductive material
on top and bottom to prevent people from directly touching the sensor. If you
make the sensor out of conductive textile, cotton or nylon based fabrics are
usually good insulators. Be aware though that sweat and moisture might still
penetrate these materials and thus can still cause the user to directly touch
the sensor. If that is a possibility, consider using a water proof fabric.

The stackup for a basic sensor is shown in the figure below. 
![sensor-stackup-basic](../../sensor-stackup-basic.png)

Note that this figure is not to scale. For real world sensors, you want the
conductive material that forms the actual sensor to be as close to the surface
as possible. Hence, the insulating material is usually only a few mm thick. 

The conductive material for the sensor does not have to be thick; capacitive
sensors use only very little current so losses due to a relatively high
resistance (up to a few 100 Ohm) are usually not an issue. This also means that
you usually can make the sensors out of carbon based material or material that
has a conductive coating. 

Cover material
-----
In many projects, the capacitive sensor will be in a casing. This casing should
not be made of metal as that can shield off the electrical field from the
sensor to the users hand and thus prevent the capacitive touch to work
properly. For the same reason, it is not recommended to use paint with a lot of
conductive particles (such as metal or carbon). Some paints with metallic
particles seem to work fine though; you'll have to test which paints work well
and which don't. Also, be aware that some black paints contain large amounts of
carbon and might affect the capacitive touch sensing.

As said before, the material should not be made of metal. Most other materials
such as wood, plastic and glass as well as textiles (cotton, nylon, wool) are
fine though. Note however that if the cover material is thick, the type of
material can have an influence on the sensitivity of the sensor, since
different materials conduct the electric field in a different way.

The degree to which the electric field is conducted is called the
[permittivity](https://en.wikipedia.org/wiki/Permittivity) 
(symbol $$ \varepsilon $$) and expressed in F/m (farad per meter). Usually, it
is expressed as a number relative to the permittivity of vacuum. The
permittivity of vacuum (symbol $$ \varepsilon_0 $$) is approximately $$ 8.85
\cdot 10^{-12} $$ F/m. The relative permittivity (symbol $$ \varepsilon_r $$ )
is then a unitless number that you have to mulitply with $$ \varepsilon_0 $$ to
get the absolute permittivity of a material.

For capacitive sensing, usually a higher permittivity results in a better
capacitive sensing, and thus with a material with a higher permittivity you can
use a thicker cover material.

A small overview of some different materials is shown below.

| Material | Relative permittivity ( $$ \varepsilon_r $$ ) |
|-----|-----:|
| Air | 1 |
| Teflon | 2.1 |
| Paper | 3.85 |
| Mica | 3 - 6|
| Concrete | 4.5 |
| Rubber | 7 |
| Glass | 3 - 10 |
| Water | 80 |

Bridging an air gap in an enclosure
-----
If you need to cover a larger distance (for example, if you need to cover a gap
of a few cm when your pcb is on the bottom of an enclosure and you want the
buttons to be on top), you can leave out the insulation and instead stick the
touch pad to the inside of the enclosure and use a metal spring or conductive
foam to make a connection between the touch pad on the top of the enclosure and
the sensor on the pcb.
![capacitive-sensor-at-a-distance](../../capacitive-sensor-at-a-distance.png)

The sensor inside the enclosure does not have to be the same size or shape as
the copper pad; it only needs to be large enough to make a proper connection
with the metal spring.

Shields and guards
-----
* for the sensor itself
* for the wire

Note also that the wire that connects the sensor to the Arduino has insulation
on both sides as we don't want the user to be able to directly touch this wire.

