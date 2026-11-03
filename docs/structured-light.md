# Structured-Light Methods

Structured-light scanning projects a known pattern sequence onto an object and observes the patterns with a camera. SLStudio decodes the captured sequence and uses camera-projector calibration to reconstruct a point cloud.

## Pattern Decoding

A codec maps camera pixels to one or two projector coordinates. Phase-shifting codecs estimate wrapped phase and modulation from captured intensities; unwrapping resolves the phase's repeated $2\pi$ cycles into a projector coordinate. Pixels with insufficient modulation or unreliable phase are excluded by a validity mask. Directional codecs can encode horizontal coordinates, vertical coordinates, or both.

## Methods and Tradeoffs

- **Phase shifting:** projects several sinusoidal fringes with different phase offsets. The decoder estimates phase and modulation from the intensity changes. The default three-step, 16-period sequence uses six patterns per selected axis: three phase steps plus a three-pattern full-width cue to unwrap the repeated fringes. More steps can improve phase estimation, but require more exposures. A 2x3 sequence is a practical starting point.
- **Heterodyne unwrapping:** captures phase-shifted sets at two frequencies whose cycle counts differ by one across the projector axis. Their phase difference forms a lower-frequency cue that resolves the fringe order. With the default three steps, this uses six patterns per selected axis.
- **Spatial unwrapping:** uses a phase-shifted fringe sequence followed by quality-guided spatial phase unwrapping. The default sequence uses three patterns across four periods. The central fringe order is the zero-based fringe order near the center of the camera image; the default is 3. An incorrect value can yield plausible geometry at the wrong depth.
- **Motion-compensated phase shifting:** uses two shifted fringe patterns and a flat reference pattern. When compensation is enabled, image translation is estimated from successive reference frames and used to align the fringe captures before phase decoding. This can reduce artifacts from inter-frame translation, but does not correct arbitrary or deforming motion.
- **Micro phase shifting:** combines a phase-shifted base fringe with a fixed set of ten pitches total, including the base pitch, to identify projector position and unwrap the base phase. The pitch set is jointly designed; changing its members independently breaks the code. With the default three phase steps, the sequence uses 12 patterns.
- **Modulated phase shifting:** multiplies horizontal signal fringes by phase-shifted vertical carrier fringes. The carrier sequence is demodulated to recover the signal phase, which is then unwrapped with a separate phase cue. The default three signal steps, three carrier steps, and three cue steps use 12 patterns.
- **Gray code:** starts with white and black reference patterns, followed by bit patterns for each selected axis. The defaults are 10 horizontal and 6 vertical bits. Decoding identifies a projector-coordinate interval for each camera pixel. SLStudio then interpolates code transitions between neighboring camera pixels and triangulates those subpixel transition samples, producing a sparse point cloud rather than dense measurements across stripe interiors.

Method suitability depends on the surface, motion, available capture time, and projector-camera setup. More exposures can improve phase or code discrimination, but increase capture time and the opportunity for motion between patterns. Gray code is useful when transition points are sufficient; phase-based methods produce dense coordinate maps when their validity masks permit.

## From Coordinates to Points

For dense codecs, each valid camera pixel is paired with its decoded projector coordinate or coordinates. The geometric calibration supplies the camera and projector intrinsics, lens distortion, and their relative pose. Triangulation combines the camera ray with the corresponding projector ray or coordinate plane to produce a 3D point. Invalid phase estimates and geometrically invalid results are omitted from the cloud.

Gray code follows a sparse path: each interpolated transition provides a subpixel camera location and a projector-coordinate boundary. The calibrated camera-projector geometry triangulates each sample independently, so the resulting cloud contains points along code boundaries rather than one point per camera pixel.

## Calibration

Calibration determines both where reconstructed points lie and how accurately the projected patterns can be decoded. Geometric calibration establishes the camera-projector geometry; intensity calibration linearizes the projected brightness used by phase-based methods. They address different error sources and are not substitutes for one another.

### Geometric Calibration

Geometric calibration estimates camera and projector intrinsics, lens distortion, and the projector's pose relative to the camera. Reconstruction combines this geometry with decoded projector coordinates, so a poor calibration can bias point-cloud depth, scale, and shape throughout a scan.

Use a flat, rigid target with accurately known spacing. The default **Symmetric circles (white)** target supports partial-grid detection; the entire target does not need to be visible in every view. Checkerboards are also available. Enter the actual center-to-center circle spacing or checkerboard corner spacing. A [calibration target generator](https://calib.io/pages/camera-calibration-pattern-generator) can help produce a target with known dimensions.

Capture at least three usable poses; more varied views usually give a better-constrained calibration. Move and tilt the target so detected features cover different parts of the camera image and provide a range of orientations, rather than capturing repeated, front-facing views. Inspect the accepted-pose count and camera, projector, and stereo RMS results. Low RMS is useful, but does not compensate for poor image coverage or inaccurate target spacing.

The calibration is tied to the camera image size and projector resolution used to capture it. Recalibrate if either changes, or if the camera-projector alignment, lens focus, or zoom changes enough to alter the geometry. A saved calibration for an example dataset is not a substitute for one made with your own setup.

### Intensity Calibration

Projector intensity is not always proportional to the requested pixel value. This nonlinear projector-camera response distorts fringes and shifts estimated phase. Intensity calibration measures the combined response and builds a single 8-bit inverse lookup table for projected patterns; it is not a spatial correction map.

Choose a uniform, matte measurement region. Keep exposure and gain fixed, disable automatic camera controls, limit ambient-light changes, and allow the projection to settle. Use the same camera, projector, display mode, and capture settings when scanning.

Measure from black through white with 5-255 levels (255 by default). Check the plots: SLStudio rejects measurements with over 1% saturated pixels, under 10 digital numbers of black-to-white range, or a noticeably non-monotonic response. Reduce exposure or gain to address saturation; increase usable contrast if the range is too small. Non-monotonic readings often indicate automatic controls, changing ambient light, or insufficient settling. Accept the curve to apply it; use a compensated measurement only to verify an existing curve. Recalibrate if the setup or camera response settings change materially.
