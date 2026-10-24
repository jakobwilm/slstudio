#pragma once

#include <opencv2/core.hpp>
#include <chrono>
#include <cstdint>
#include <optional>
#include <stdexcept>

namespace slstudio {

struct CameraDescriptor;

enum class CameraReadStatus { Complete, Timeout, Incomplete };

struct CameraCapabilities {
  bool softwareTrigger = false, externalTrigger = false;
  bool sequenceStart = false;
};

struct CameraFrameMetadata {
  std::optional<std::uint64_t> frameId, lineStatus;
  std::optional<std::chrono::nanoseconds> deviceTimestamp;
  std::chrono::steady_clock::time_point hostTimestamp{};
  std::optional<bool> sequenceStart;
  std::uint64_t frameIdMaximum = UINT64_MAX;
};

struct CameraImage {
  cv::Mat1b image;
  CameraFrameMetadata metadata;
};

struct CameraReadResult {
  CameraReadStatus status = CameraReadStatus::Timeout;
  CameraImage frame;
};

class CameraError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

class CameraDisconnected : public CameraError {
public:
  using CameraError::CameraError;
};

class Camera {
public:
  virtual ~Camera() = default;
  virtual const CameraDescriptor &descriptor() const = 0;
  virtual CameraCapabilities capabilities() const = 0;
  virtual cv::Size resolution() const = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual bool isCapturing() const = 0;
  virtual void softwareTrigger() = 0;
  virtual CameraReadResult read(std::chrono::milliseconds timeout) = 0;
};

CameraImage cameraImageSnapshot(const cv::Mat &image, CameraFrameMetadata metadata = {});

}