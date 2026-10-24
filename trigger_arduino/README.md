# Ximea camera trigger circuit

This Arduino based sketch is for cameras that have only one trigger-input GPIO when used with LightCrafter trigger outputs. The same camera input is used for both frame triggering and sequence marking: a normal frame trigger is short, while the signal stays LOW longer at the start of a sequence.

The XIMEA backend does not measure the LOW pulse duration. For each acquired image it reads the GPIO level from `image.GPI_level`, then checks the configured sequence-marker bit and polarity to set the frame's sequence-start flag. The longer LOW level keeps the marker asserted for the first frame, so the capture code can recognize the sequence boundary. 

The Arduino sketch is [rt_lc4500_exptime.ino](./rt_lc4500_exptime.ino).
