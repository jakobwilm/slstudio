# Architecture

SLStudio separates device acquisition, structured-light decoding, triangulation, and presentation. The desktop application coordinates these stages; the reconstruction algorithms are also usable without the GUI.

## Desktop scan flow

```mermaid
flowchart LR
    GUI["GUI loop and ApplicationController"] --> Session["CameraSession and CameraDiscovery"]
    Session --> Capture["CaptureWorker"]
    Capture -->|preview and completed sequences polled by GUI| GUI
    Capture -->|scan sequences| Decoder["DecoderWorker"]
    GUI -->|saved-sequence reconstruction| Decoder
    Decoder -->|dense maps or sparse transitions| Triangulator["TriangulatorWorker"]
    Triangulator -->|completed point clouds polled by GUI| GUI
    GUI --> Renderer["Point-cloud renderer"]
    GUI --> ROS["Optional ROS 2 output"]
```

`CameraSession` coordinates the active camera and its preview lifecycle. `CameraDiscovery` performs backend enumeration and trigger-source queries asynchronously. Both are GUI-side coordinators; camera and projector operations are issued through `CaptureWorker`.

`CaptureWorker` owns device instances and calls them on its acquisition thread. Replacing a device releases the old one before opening its replacement. Stopping a scan ends sequence capture without closing the camera; stopping the camera preserves its configuration. The GUI polls command futures and completed frames rather than blocking on device calls.

Completed scans go directly from `CaptureWorker` to `DecoderWorker`, without waiting for GUI polling. The GUI retains completed sequences for saving and calibration; calibration captures are not decoded. Each reconstruction worker replaces pending work with the newest input. Dense decoding produces projector-coordinate maps; Gray-code decoding produces sparse transitions. The decoder submits its result directly to `TriangulatorWorker`, which exposes completed point clouds for GUI polling. Worker errors are surfaced through their polling APIs.

The GUI thread owns ImGui, preview textures, and point-cloud rendering. `CaptureWorker` owns every projector backend, including its pattern rendering and presentation waits. `ProjectorPresentationController` dispatches required native window operations to the GUI thread. Optional ROS 2 publishing consumes completed point clouds from the GUI update loop.

## Offline reconstruction

`slstudio-reconstruct` loads a saved sequence, then calls the synchronous `DecodeStage` and `TriangulationStage` directly. It shares the core algorithms and calibration data with the desktop application without creating GUI workers or OpenGL resources.

## Validation boundaries

Core tests exercise codecs, calibration, persistence, reconstruction, and point-cloud behavior without the GUI. Acquisition tests verify device-thread ownership, handover, cancellation, and sequence behavior. GUI smoke tests cover the integrated update loop and rendering paths; hardware diagnostics are separate and require connected devices.