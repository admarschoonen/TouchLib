---
layout: page
title: Pseudo-differential sampling
permalink: /signal-processing/pseudo-differential-sampling/
in_menu: true
weight: 30
in_book: 3
---

Some methods to measure capacitance support a so called pseudo-differential
sampling, which can help reduce noise.

The idea behind pseudo-differential sampling is that two samples are taken. The
first sample is taken using normal polarity and the second sample is taken with
reversed polarity. In the presence of a additive noise source with low
bandwidth, the noise can be (partially) cancelled out by subtracting the two
samples (low bandwidth in this case means that the instantaneous noise does not
change significantly from one sample to the next).

This might be best explained with an example for pseudo-differential resistive
sampling. Consider therefore the following schematic, where we want to measure
the resistance of resistor $$R_{x}$$ by measuring the voltage $$V_{x}$$ at node
x. The value of the reference resistor $$R_{1}$$ and the supply voltage
$$V_{cc}$$ are known.

![resistive measurement schematic](../../resistive-measurement-schematic.png)

The voltage at $$V_{x}$$ is then:

$$
V_{x,1} = { {R_{x}} \over {R_{1} + R_{x}} } V_{cc} + n_{1}
$$

where $$n_{1}$$ is noise from an additive noise source.

From this, we can estimate the value of $$R_{x}$$ with 

$$
\hat{R}_{x,1} = { {V_{x,1}} \over {V_{cc} - V_{x,1}} } R_{1} - { {n_{1}} \over {V_{cc} - V_{x,1}} } (R_{1} + R_{x}).
$$

If instead of powering this circuit with a fixed source and a fixed ground, we
would power the circuit with 2 GPIO pins from a microcontroller (for eample
pins 2 and 3 of an Arduino), we can take 2 measurements and combine them as one
which has lower noise than any of the two measurements individually.

Assume a circuit as shown below where the top of $$R_{1}$$ is connected to pin
2 and the bottom of $$R_{x}$$ is connected to pin 3.

![resistive measurement Arduino](../../resistive-measurement-arduino.png)

By setting pin 2 as output and high, and pin 3 as output and low, we
effectively create the circuit as shown on the top of this page. Hence, the
measurement of $$R_{x}$$ can be estimated with the formula of $$\hat{R}_{x,1}$$
as shown above.

However, we can also take a second measurement with the polarity of the pins
reversed: pin 2 becomes output and low and pin 3 becomes output and high.
In that case, the top of $$R_{1}$$ would be connected to ground and the bottom
of $$R_{x}$$ would be connected to $$V_{cc}$$. The measured voltage $$V_{x}$$
would then be

$$
V_{x,2} = { {R_{1}} \over {R_{1} + R_{x}} } V_{cc} + n_{2}
$$

and the value of $$R_{x}$$ can be estimated with

$$
\hat{R}_{x,2} = { {V_{cc} - V_{x,2}} \over {V_{x,2}} } R_{1} + { {n_{2}} \over {V_{x,2}} } (R_{1} + R_{x}).
$$

We can now use the property $$V_{x,2} = V_{cc} - V_{x,1}$$ to rewrite the estimate
$$\hat{R}_{x,2}$$ as

$$
\hat{R}_{x,2} = { {V_{x,1}} \over {V_{cc} - V_{x,1}} } R_{1} + { {n_{2}} \over {V_{cc} - V_{x,1}} } (R_{1} + R_{x}).
$$

If the noise source now has a limited bandwidth ($$n_{1} \approx n_{2}$$), the
noise term for $$\hat{R}_{x,1}$$ approximates opposite of the noise term of
$$\hat{R}_{x,2}$$ and thus we can remove the noise by averaging both
measurements:

$$
\begin{align}
\hat{R}_{x} &= { {\hat{R}_{x,1} + \hat{R}_{x,2}} \over {2} } \\
  &\approx { {V_{x,1}} \over {V_{cc} - V_{x,1}} } R_{1}.
\end{align}
$$

Therefore, if the bandwidth of the noise source is much lower than the sample
frequency, taking two measurements with opposite polarities and combining them
can significantly reduce the noise.

Note that pseudo-differential measurement is not always possible. At the moment
pseudo-differential measurement (`sampleTypeDifferential`) is only implemented
for the CVD method; any other sample methods in TouchLib (including the
resistive sample method) only support normal (`sampleTypeNormal`).

