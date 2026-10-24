#pragma once

#include "PointCloud.h"
#include <chrono>
#include <memory>
#include <string>

namespace slstudio {

struct Ros2OutputSettings {
  std::string topic = "/slstudio/points";
  std::string frameId = "slstudio_camera_optical_frame";
  double metresPerUnit = 1.0;
};

// Publishes unorganized XYZ/packed-RGB PointCloud2 with best-effort, volatile, depth-one QoS.
class Ros2Output {
public:
  static bool available();
  explicit Ros2Output(const Ros2OutputSettings &settings);
  ~Ros2Output();
  Ros2Output(const Ros2Output &) = delete;
  Ros2Output &operator=(const Ros2Output &) = delete;
  // Shares the immutable cloud and replaces pending output; timestamp is supplied by the caller.
  void submit(PointCloudConstPtr cloud, std::chrono::system_clock::time_point timestamp);
  // Poll for asynchronous failures even when no new clouds are submitted.
  void checkError() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl;
};

}
