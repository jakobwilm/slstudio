<p align="center"><img src="icon.svg" alt="SLStudio icon" width="128"></p>

# SLStudio — Real-Time Structured Light

[![GitHub stars](https://img.shields.io/github/stars/jakobwilm/slstudio)](https://github.com/jakobwilm/slstudio/stargazers)
[![CI build](https://github.com/jakobwilm/slstudio/actions/workflows/build.yml/badge.svg)](https://github.com/jakobwilm/slstudio/actions/workflows/build.yml)
[![Sponsor](https://img.shields.io/badge/Sponsor-ea4aaa?logo=githubsponsors)](https://github.com/sponsors/jakobwilm)

[![Download Linux AppImage](https://img.shields.io/badge/Download-Linux_AppImage-2ab3f7?style=for-the-badge)](https://github.com/jakobwilm/slstudio/releases)
[![Download macOS DMG](https://img.shields.io/badge/Download-macOS_DMG-2ab3f7?style=for-the-badge)](https://github.com/jakobwilm/slstudio/releases)
[![Download Windows ZIP](https://img.shields.io/badge/Download-Windows_ZIP-2ab3f7?style=for-the-badge)](https://github.com/jakobwilm/slstudio/releases)

SLStudio is an open-source framework for building a 3D scanner from a camera and a light projector. It combines calibration, pattern projection, image acquisition and point-cloud reconstruction in a desktop application.

Version 2 is a completely reworked codebase with fewer and lighter weight dependencies, and many new features. The complexity of getting it to work with diverse hardware is also dramatically reduced.

The goal is a flexible platform with low-latency, real-time reconstruction. Hardware trigger synchronization can achieve point-cloud rates of 20 Hz or more; actual speed depends on the pattern sequence, camera, projector and processing. Ordinary projectors and webcams work at lower rates, without the same timing guarantees.

The point cloud stream can be published as a ROS2 topic. 

![Skull 2x3 point-cloud reconstruction in SLStudio's ImGui interface](resources/skull-2x3-imgui.png)

*The skull example reconstructed from six projected patterns.*

## Scope and approach

- **Structured-light methods:** phase shifting, heterodyne unwrapping, spatial unwrapping, motion-compensated and other phase-shift variants, plus sparse Gray-code reconstruction.
- **Calibration and reconstruction:** camera/projector geometry, projector intensity compensation, configurable filtering, live point-cloud viewing and PLY/PCD export. 
- **Hardware:** Aravis (USB3 Vision/GigE Vision), XIMEA xiAPI, FLIR Spinnaker and OpenCV webcams; desktop projector outputs and optional direct LightCrafter 3000/4500 control.
- **Live and offline scanning:** low-latency point clouds, reconstruction of recorded sequences and optional ROS 2 output.

Hardware integration remains setup-dependent. Software tests and simulated devices do not establish physical timing, camera compatibility or safe projector flash operation.

The code has a clear separation boundary between algorithms, capture code and GUI. As such it is straight forward to isolate only reconstruction or capture routines and use them with a different interface. Separate cmake targets make that boundary clear.

## Installation

### Portable packages

The download buttons link to [GitHub Releases](https://github.com/jakobwilm/slstudio/releases). Version tags publish packages and SHA-256 checksums after all CI builds and package checks pass. Branch builds remain available as [Actions artifacts](https://github.com/jakobwilm/slstudio/actions).

| Platform | Package | Notes |
| --- | --- | --- |
| Linux x86-64 | AppImage | Bundles application libraries, Aravis, ROS 2 Jazzy and Fast DDS; requires compatible glibc >= 2.39 and host graphics drivers. |
| macOS | DMG with .app | Drag to Applications. Architecture-specific, ad-hoc signed, not notarized; Gatekeeper may require explicit approval. Right-click -> Open, then see Privacy & Security settings. |
| Windows x64 | ZIP folder | Extract everything and keep the executables, DLLs and bundled MSVC runtime together. |

On Linux, make the AppImage executable with `chmod +x SLStudio-linux-x86_64.AppImage`. If FUSE is unavailable, launch with `APPIMAGE_EXTRACT_AND_RUN=1 ./SLStudio-linux-x86_64.AppImage`. 

Packages omit example datasets and vendor XIMEA/Spinnaker SDKs. To get started, download the example_data directory from this repository and try Scan -> Load Sequence.

### Platform feature matrix

**✓** means included in the packages configured by the [CI workflow](.github/workflows/build.yml), or available in a source build, depending on the column. It is not a guarantee of a published artifact or tested hardware. **Optional** requires additional dependencies or configuration. **X** means not included in the package or not supported on the platform.

| Feature | Linux package | Linux source | Windows package | Windows source | macOS package | macOS source |
| --- | --- | --- | --- | --- | --- | --- |
| Desktop GUI, calibration, scanning and point-cloud viewer | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| CLI reconstruction, codecs and PLY/PCD export | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| OpenCV webcam capture | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| Aravis USB3 Vision / GigE Vision cameras | ✓ | Optional | ✓ | Optional | ✓ | Optional |
| XIMEA xiAPI cameras | X | Optional | X | Optional | X | Optional |
| FLIR / Teledyne Spinnaker cameras | X | Optional | X | Optional | X | Optional |
| Desktop fullscreen projector output | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| Desktop-independent fullscreen | ✓ | Optional | X | X | X | X |
| LightCrafter 4500 USB control and pattern upload | ✓ | Optional | ✓ | Optional | ✓ | Optional |
| LightCrafter 3000 network control | ✓ | Optional | X | Optional | X | Optional |
| ROS 2 PointCloud2 publishing | ✓ | Optional | X | Optional | X | Optional |

Source-build options are reported during CMake configuration. Aravis needs its shared library and pkg-config; vendor camera backends need compatible manufacturer SDKs for the target OS and architecture. LC4500 uses HIDAPI. DRM leasing requires Linux libraries and compositor/display support. ROS 2 publishing support requires `rclcpp` and `sensor_msgs` in the build environment. 

### Building from source

Requires CMake >= 3.21, a C++17 compiler, OpenCV 4 and Eigen >= 3.3. The GUI additionally uses GLFW, GLEW, OpenGL 3.3 Core and Dear ImGui, which CMake downloads at a pinned version.

For Ubuntu 24.04 or newer:

```sh
sudo apt install cmake git g++ pkg-config libopencv-dev libeigen3-dev \
  libglfw3-dev libglew-dev libaravis-dev libudev-dev libdrm-dev \
  libwayland-dev wayland-protocols
```

```sh
cmake --preset desktop
cmake --build --preset desktop -j4
ctest --preset desktop
./build/desktop/SLStudio
```

Optional camera SDKs, ROS 2 and projection features are detected automatically. Install the manufacturer's SDK for XIMEA or Spinnaker cameras; alternatively Aravis supports compatible USB3 Vision/GigE Vision cameras without vendor SDKs. The build reports which integrations are available. 

On Linux, launching the local GUI build normally also registers a per-user launcher in `$XDG_DATA_HOME/applications` (default `~/.local/share/applications`) with its icon and the executable's absolute path. 

For Windows/macOS build dependencies, see the [CI workflow](.github/workflows/build.yml). Linux is the currently validated development platform.

On Windows, install Visual Studio 2022 with **Desktop development with C++**, CMake and Git. From a **Developer PowerShell for VS 2022**, run these commands in the source directory:

```powershell
git clone https://github.com/microsoft/vcpkg.git .vcpkg
$baseline = (Get-Content vcpkg.json -Raw | ConvertFrom-Json).'builtin-baseline'
git -C .vcpkg checkout $baseline
.\.vcpkg\bootstrap-vcpkg.bat -disableMetrics
cmake --preset desktop -G "Visual Studio 17 2022" -A x64 "-DCMAKE_TOOLCHAIN_FILE=$PWD/.vcpkg/scripts/buildsystems/vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows
cmake --build --preset desktop --config Release --parallel 4
ctest --preset desktop -C Release
.\build\desktop\Release\SLStudio.exe
```

CMake installs the dependencies listed in [vcpkg.json](vcpkg.json), using the same pinned versions as Windows CI, including Aravis with USB support. Eigen remains pinned to the compatible 3.x series. The first build compiles dependencies and can take a while; subsequent builds reuse the local installation.

Local builds use the standard `x64-windows` triplet, providing both Debug and Release dependencies. To build and test Debug, replace `--config Release` with `--config Debug` and `-C Release` with `-C Debug`. 

## Usage

### Calibrate and scan

1. Open **View > Settings...** and select the camera, projector and coding method. Start with software triggering; configure exposure, crop and settling for your setup.
   Software-triggered capture takes one frame per projected pattern, after the settle delay allows the projection to stabilize. Settle time applies only to software-triggered capture and is hidden when hardware triggering is selected. Intensity measurement always uses software triggering and retains its own settle-time control.
2. Open **Calibration > Geometric Calibration...**. Settings default to **Symmetric circles (white)**. Partial circle grids are detected automatically, so the full target need not be visible and the row/column counts are maximums. Capture varied positions and angles across the field of view. **Accept** saves and activates the calibration; **Load Geometric Calibration...** imports an existing one. [Calibration target generator](https://calib.io/pages/camera-calibration-pattern-generator).
   Use **Capture sequence** below the image; during acquisition it becomes **Cancel capture** with frame progress. Select a pose for its averaged preview, or expand it to review individual raw frames. **Live camera** returns from review to the preview, ready for another capture. **Remove pose** and **Clear poses** ask for confirmation before deleting captured frames.
3. Optionally use **Calibration > Intensity Calibration...** to measure the system response and accept a compensation curve. Measurement defaults to 255 input levels, with one image per level. 
4. Choose **Single sequence** or **Continuous** in Settings, then **Scan > Start Scan**. The selected geometric calibration must match your camera and crop dimensions; the bundled example calibration is for the example datasets, not your camera. Continuous scanning shows the completed capture sequence number, capture drops, processing drops and displayed point count in fixed-width status fields. **Stop Scan** preserves the camera preview and last complete sequence.
5. Use **Scan > Save Sequence...** / **Load Sequence...** for recorded frames, and **Export Point Cloud...** for PLY or PCD. The load picker shows files alongside subfolders in the opened folder; select a folder to load, not an individual file. Loaded sequences restore coding parameters, while reconstruction uses the active calibration and reconstruction settings.

Scroll to zoom and double-click a point to make it the rotation pivot. The first non-empty cloud establishes the display framing; subsequent clouds keep the same center, scale and rotation pivot, including during continuous scanning. Fitting ignores the outer 1% of coordinates on each axis to reduce outlier influence, without removing points; clouds with fewer than 100 points use their full bounds. Depth coloring uses the same trimming for its color range, with outlying depths clamped to the endpoint colors. **View > Reset View** or **Refit view** in **Render Settings** explicitly refits the current cloud and resets pan, rotation and zoom. **View > Render Settings** controls appearance; **View > Frame sequence** shows the latest complete capture.

Phase-shift methods trade pattern count, precision and motion sensitivity. Something like 2x3 phase shifted patterns with default fringe count is usually a good starting point. Gray code produces sparse measured transitions, not dense stripe interiors. Spatial unwrapping uses fewer patterns, but requires a correct **Central period** assumption in the coding-method controls. This is the zero-based fringe order near the camera image center; an incorrect value can produce plausible geometry at the wrong depth.


### Projection and synchronization

Desktop fullscreen outputs are available on secondary screens. Linux Wayland also supports exclusive DRM leases where the compositor and driver permit them; on X11, detach the projector from the desktop before leasing it. Desktop projection can be affected by other windows and display processing.

For optional X11 DRM leasing, install `libxrandr-dev libx11-xcb-dev libxcb-randr0-dev` and rerun CMake configuration. 

Real-time scanning requires suitable projector-to-camera trigger wiring. Free-running continuous capture additionally needs frame-associated sequence-start triggers for reliable continous scanning, albeit it can work without. Projectors based on the LightCrafter4500 platform (and the older LightCrafter3000) support hardware patterns uploaded via USB.

In hardware-triggered capture, waiting for the next trigger preserves the current partial sequence. Incomplete images, frame-counter gaps and sequence-start markers still discard a partial sequence so frames from different pattern cycles are not combined.
Between captures, native cameras use free-running preview regardless of the selected trigger mode. Before projection, acquisition stops, switches to software or hardware triggering and restarts with fresh image buffers; completion or cancellation restores preview. Intensity measurement remains software-triggered. XIMEA failures include readable descriptions and numeric SDK codes; expected image-wait timeouts do not produce routine SDK error messages.

![Example synchronized setup](example_data/realtime_trigger_enclosure.jpg)


### LightCrafter4500

Patterns can be supplied in two ways:

- **Upload to flash:** open the direct LC4500 projector and use **Upload scan patterns...** or **Upload calibration patterns...**, then play the resident sequence. Starting a scan does not upload patterns; stored patterns must match the selected codec, crop and intensity compensation.
  Flash playback keeps at least 231 us between exposures so camera trigger pulses and the Arduino sequence marker remain distinct. A requested 16,667 us exposure and period therefore uses a 16,898 us period. Up to six 8-bit patterns fit in the two image buffers and need no flash-loading delay. Longer sequences share a conservative 200 ms image-loading budget across the patterns in each buffer, rather than adding 200 ms to every frame. For twelve patterns at 16,667 us exposure this gives a minimum frame period of 66,667 us; longer requested periods are preserved. Repeating sequences ending with a partially filled image buffer need a larger per-frame budget.
- **Stream over HDMI:** in **Tools > LC4500 Settings...**, select **HDMI patterns (8-bit)** and apply the settings, then select the corresponding HDMI display output in SLStudio. Patterns come from the computer rather than flash; synchronize camera exposures with the projector's trigger outputs.

For cameras with limited I/O, the included [Arduino trigger-circuit sketch](trigger_arduino/rt_lc4500_exptime.ino) combines LightCrafter frame and sequence triggers onto one camera input. See the [circuit notes](trigger_arduino/README.md) for sequence marking and XIMEA GPIO metadata.

Flash programming is experimental and restricted to supported firmware/layouts. Keep power and USB connected throughout; uploads cannot be cancelled.

On Linux, install the LC4500 hidraw permission rule, then reconnect USB:

```sh
sudo install -m 0644 udev/70-slstudio-lc4500.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

The **Tools > LightCrafter3000 Configuration** and **Tools > LightCrafter4500 Configuration** entries open the corresponding configuration tools. The tools allow you to verify and experiment with LightCrafter hardware settings.

### ROS 2 point clouds

The Linux AppImage includes ROS output. For source builds, source your ROS 2 Jazzy environment before building to enable automatic discovery.

Enable **Tools > ROS 2 Output > Publish point clouds**. The default topic is `/slstudio/points`, with frame ID `slstudio_camera_optical_frame`. Set **Metres per calibration unit** to `0.001` for millimetres or `1` for metres. The frame ID labels camera coordinates; no transform is broadcast.

In RViz, use the matching fixed frame and a PointCloud2 display with best-effort reliability. Nodes must share compatible domain/discovery settings. Output is off at startup; publishing errors are reported without stopping scanning.

### Command-line reconstruction

Reconstruct a saved sequence without the GUI:

```sh
./build/core/slstudio-reconstruct calibration.xml sequence-folder output.ply
```

## Scientific references

When using SLStudio in academic work, please consider citing:

- Wilm et al. **SLStudio: Open-Source Framework for Real-Time Structured Light.** IPTA, 2014. [IEEE Xplore](https://ieeexplore.ieee.org/document/7002001).
- Wilm, Jakob, Oline V. Olesen, and Rasmus Larsen. **Accurate and simple calibration of DLP projector systems.** *Emerging Digital Micromirror Device Based Systems and Applications VI*, vol. 8979, SPIE, 2014.
- Wilm, Jakob, et al. **Correction of motion artifacts for real-time structured light.** *Scandinavian Conference on Image Analysis*, Springer, 2015.

## Support and license

Building a scanner requires optics, synchronization and often C++ integration work; not every camera/projector combination is tested. Focused bug reports and pull requests are welcome through [GitHub](https://github.com/jakobwilm/slstudio/issues).

SLStudio is available under [GPLv3](LICENSE), with commercial licensing available from [jakob@viscon.dk](mailto:jakob@viscon.dk).

Dependencies retain their own licenses. See [third-party notices](THIRD_PARTY_NOTICES.md) for the component overview, TI API license terms and the locations of notices in portable packages.
