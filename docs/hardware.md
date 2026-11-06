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

For LightCrafter 4500 desktop fullscreen HDMI output, configure the projector as a secondary, extended desktop display (not mirrored), with 100% scaling. In Ubuntu display settings, select **912x1140 for HDMI pattern mode** or **1280x800 for HDMI video mode**. The desktop HDMI resolution must match the mode selected on the projector.

For DRM lease output, select an advertised **Display mode** in SLStudio before opening the output: **912x1140** for LC4500 HDMI pattern mode or **1280x800** for HDMI video mode. **Preferred (automatic)** uses the display's preferred mode, which is 1280x800 on the LC4500. The selection is saved separately from pixel layout; an unavailable saved mode must be reselected before opening the output. See [DRM lease output](#drm-lease-output) for automatic reservation and command-line setup.

For HDMI pattern mode, select **Diamond** under **Pixel layout** in SLStudio's projector settings before opening the display output. This generates patterns on an 1824x1140 logical grid and maps them to the 912x1140 diamond input. Leave **Rectangular** selected for HDMI video mode and ordinary displays. The pixel-layout choice is saved with the application settings; calibration must match the selected logical geometry.

Flash playback timing depends on pattern count and projector buffer loading. The projector may lengthen the requested frame period to preserve the trigger interval and allow patterns to load. For example, a 16.667 ms requested exposure and period can require a 16.898 ms period; longer pattern sequences may require additional time. Check the resulting timing with the actual projector and camera setup.

Flash programming is supported only for compatible firmware and layouts. Keep USB and projector power connected throughout an upload; uploads cannot be cancelled.

On Linux, install the LightCrafter 4500 hidraw permission rule, then reconnect USB:

```sh
sudo install -m 0644 udev/70-slstudio-lc4500.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

The **Tools > LightCrafter3000 Configuration** and **Tools > LightCrafter4500 Configuration** entries open the corresponding configuration tools.

## DRM Lease Output

On GNOME Wayland with DRM leasing enabled, SLStudio lists eligible external, non-primary displays as **DRM lease** options even while they are part of the desktop. Discovery does not change the desktop. Opening a candidate temporarily reserves its connector, acquires the lease, and starts projector output. Closing the output, or a failed open, restores its previous desktop configuration. If the desktop configuration changes independently while the output is open, SLStudio leaves the reservation in place rather than overwriting those changes. An abrupt application exit can also leave the connector reserved; use the command-line restore procedure below if needed.

Displays already reserved for leasing are listed as lease outputs without a duplicate fullscreen-window option. SLStudio preserves these externally managed reservations when closing the output. Other compositors must offer the connector for leasing themselves; automatic reservation is GNOME-specific.

### GNOME Command-Line Reservation

Run these commands in the graphical session, without `sudo`. First inspect the current configuration:

```bash
display_config() {
	gdbus call --session \
		--dest org.gnome.Mutter.DisplayConfig \
		--object-path /org/gnome/Mutter/DisplayConfig \
		--method "org.gnome.Mutter.DisplayConfig.$1" "${@:2}"
}
display_config GetCurrentState
```

The result contains the configuration serial, monitor identities and mode IDs, and logical desktop layout. Adapt the following example to those values. It keeps a primary `DP-3` display at 3840x2160 with scale 2 and reserves an LC4500 on `HDMI-1`. Include all displays that should remain on the desktop and any existing lease reservations; omitted desktop displays are disabled.

```bash
serial=15  # Replace with the current uint32 serial from GetCurrentState
display_config ApplyMonitorsConfig "$serial" 1 \
	"[(0, 0, 2.0, uint32 0, true, [('DP-3', '3840x2160@59.997', @a{sv} {})])]" \
	"{'layout-mode': <uint32 1>, 'monitors-for-lease': <[('HDMI-1', 'DLP', 'LCr 4500', '0x00000001')]>}"
```

Click **Refresh projectors** after reserving the connector, or start SLStudio after the command. Select its DRM lease output, display mode, and pixel layout before opening it. Merely disabling a monitor in Display Settings is not equivalent to reserving it for lease.

To return this example projector to the extended desktop, close its SLStudio output first, read the current serial again, and apply the original layout:

```bash
display_config GetCurrentState
serial=15  # Replace with the current serial
display_config ApplyMonitorsConfig "$serial" 1 \
	"[(0, 0, 2.0, uint32 0, true, [('DP-3', '3840x2160@59.997', @a{sv} {})]), (1920, 0, 1.0, uint32 0, false, [('HDMI-1', '912x1140@59.944', @a{sv} {})])]" \
	"{'layout-mode': <uint32 1>, 'monitors-for-lease': <@a(ssss) []>}"
```

Method `1` applies a temporary configuration; method `2` saves it persistently. A temporary reservation is not automatically undone when a command-line lease client exits. GNOME also stores persistent reservations in the `<forlease>` section of its monitor configuration.