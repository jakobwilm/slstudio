# Hardware Setup

SLStudio supports several camera and projector types. Availability depends on the platform, installed SDKs, and package or source-build configuration. The [README compatibility matrix](../README.md#platform-feature-matrix) summarizes package and source availability; it does not guarantee compatibility with every device.

## Cameras

Supported camera families include:

- Aravis-compatible USB3 Vision and GigE Vision cameras
- IIDC USB and FireWire cameras through libdc1394
- XIMEA cameras through xiAPI
- FLIR cameras through Spinnaker
- Standard webcams supported by OpenCV

Some camera backends require the manufacturer's SDK and may not be included in portable packages. Aravis can be used with compatible USB3 Vision and GigE Vision cameras without a vendor-specific SDK.

## Projectors

Desktop projectors can display patterns on a secondary screen. A normal desktop output is convenient, but other windows, notifications, or display processing may affect projected patterns. Exclusive projector output is available on some platform and display configurations.

LightCrafter 3000 and 4500 devices can play patterns stored on the projector. The LightCrafter 4500 can also display patterns streamed over HDMI. Select the corresponding projector and output mode in SLStudio settings.

## Synchronization

Software-triggered capture is a practical starting point. SLStudio captures after each pattern is displayed, with a settling delay to allow the projection to stabilize. This mode is generally slower than hardware synchronization.

For low-latency scanning, synchronize camera exposures with the projector's pattern timing using trigger signals supported by the devices. Continuous capture also benefits from a frame-associated sequence-start signal so that SLStudio can identify the beginning of each pattern sequence. Wiring and trigger behavior vary by camera and projector; consult their hardware manuals before connecting trigger inputs.

The included [Arduino trigger-circuit sketch](../trigger_arduino/rt_lc4500_exptime.ino) combines LightCrafter frame and sequence triggers for cameras with limited I/O. See the [circuit notes](../trigger_arduino/README.md) for wiring and sequence-marking details.

## LightCrafter Patterns

Patterns can be uploaded to projector flash or streamed over HDMI:

- **Flash playback:** use the **Upload scan patterns...** or **Upload calibration patterns...** controls for the direct LightCrafter projector. Starting a scan does not upload patterns. Stored patterns must match the selected coding method, projector resolution, and intensity compensation.
- **HDMI streaming:** for the LightCrafter 4500, select **HDMI patterns (8-bit)** in **Tools > LC4500 Settings...**, apply the setting, and choose the matching display output. Patterns are supplied by the computer; synchronize camera exposures with the projector trigger outputs.

Flash playback timing depends on pattern count and projector buffer loading. The projector may lengthen the requested frame period to preserve the trigger interval and allow patterns to load. For example, a 16.667 ms requested exposure and period can require a 16.898 ms period; longer pattern sequences may require additional time. Check the resulting timing with the actual projector and camera setup.

Flash programming is supported only for compatible firmware and layouts. Keep USB and projector power connected throughout an upload; uploads cannot be cancelled.

On Linux, install the LightCrafter 4500 hidraw permission rule, then reconnect USB:

```sh
sudo install -m 0644 udev/70-slstudio-lc4500.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

The **Tools > LightCrafter3000 Configuration** and **Tools > LightCrafter4500 Configuration** entries open the corresponding configuration tools.