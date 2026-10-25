# Ximea camera trigger circuit

This Arduino based sketch is for cameras that have only one trigger-input GPIO when used with LightCrafter trigger outputs. The same camera input is used for both frame triggering and sequence marking: at the start of a sequence, the HIGH level is extended by 100 microseconds beyond the normal falling edge.

Connect the signals as follows:

| Signal | Arduino pin |
| --- | --- |
| LightCrafter TRIG_OUT1 (per-pattern exposure signal) | D2 |
| LightCrafter TRIG_OUT2 (sequence-start signal) | D3 |
| Combined output to the camera trigger input | D6 |
| Status LED only | D13 |

Use a pin labelled **GND** for the shared signal reference where required by the camera interface; D1/TX is not ground. Use the voltage-level interface required by the camera input rather than assuming a 5 V Arduino output can connect directly. Select the matching camera input and rising-edge triggering.

The XIMEA backend does not measure pulse duration. For each acquired image it reads the GPIO level from `image.GPI_level`, then checks the configured sequence-marker bit and polarity to set the frame's sequence-start flag. On MQ cameras, the GPIO level is sampled at exposure end. To distinguish the first frame, that sample must occur after the normal pulse falls but before the extended first-frame pulse falls. The sequence-start signal must arm the Arduino before the corresponding per-pattern falling edge. A working frame trigger alone does not establish that sequence marking is timed correctly.

The Arduino sketch is [rt_lc4500_exptime.ino](./rt_lc4500_exptime.ino).
