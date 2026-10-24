#pragma once

#include <opencv2/core.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <cstddef>
#include <cstdint>
#include <vector>
#include <future>
#include "projector/Projector.h"

struct CaptureFrame {
  cv::Mat image;
  bool sequenceStart = false;
  std::optional<std::uint64_t> frameNumber{};
  std::uint64_t frameNumberMaximum = UINT64_MAX;
};

struct CapturedSequence {
  std::uint64_t sequence = 0;
  std::vector<cv::Mat> frames;
};

struct SequenceCaptureStatistics {
  std::uint64_t completed = 0;
  std::uint64_t dropped = 0;
};

class CaptureWorker {
public:
  class Camera {
  public:
    virtual ~Camera() = default;
    virtual CaptureFrame read() = 0;
  };
  using CameraFactory = std::function<std::unique_ptr<Camera>()>;
  using ProjectorFactory = std::function<std::unique_ptr<slstudio::Projector>()>;
  struct ProjectorInfo {
    cv::Size size;
    slstudio::ProjectorGeometry outputGeometry;
    slstudio::ProjectorCapabilities features;
    cv::Size resolution() const { return size; }
    slstudio::ProjectorGeometry geometry() const { return outputGeometry; }
    slstudio::ProjectorCapabilities capabilities() const { return features; }
  };

  using Reader = std::function<cv::Mat()>;
  using Initializer = std::function<void()>;
  using StreamReader = std::function<CaptureFrame()>;
  CaptureWorker();
  explicit CaptureWorker(Reader reader, Initializer initialize = {}, bool retryEmptyFrames = false);
  CaptureWorker(std::size_t patternCount, StreamReader reader, Initializer initialize = {});
  ~CaptureWorker();
  CaptureWorker(const CaptureWorker &) = delete;
  CaptureWorker &operator=(const CaptureWorker &) = delete;

  // Factories, device calls and release run on the worker; replacement releases the old device first.
  // GUI clients should poll command futures rather than block on get().
  std::future<void> openCamera(CameraFactory factory);
  std::future<void> closeCamera();
  std::future<void> openProjector(ProjectorFactory factory);
  std::future<void> closeProjector();
  std::optional<ProjectorInfo> projector() const;
  std::future<void> blankProjector(bool white = false);
  std::future<void> showSolid(std::uint8_t level, std::chrono::milliseconds settling = {});
  std::future<void> programPatterns(std::vector<cv::Mat> patterns,
      slstudio::ProjectorProgrammingProgress progress = {});
  std::future<void> captureSequence(std::vector<cv::Mat> patterns,
      std::chrono::milliseconds settling, bool continuous = false,
      bool hardware = false, slstudio::ProjectorSequenceSettings timing = {});
  // Finishes in-flight device calls but preserves the camera and completed sequences.
  std::future<void> stopSequence();
  std::size_t sequenceIndex() const;
  bool framePending() const;
  bool requestFrame();
  bool start();
  bool ready() const;
  std::optional<cv::Mat> tryTake();
  std::optional<CapturedSequence> tryTakeSequence();
  SequenceCaptureStatistics statistics() const;
  void cancel();

private:
  struct Impl;
  std::unique_ptr<Impl> impl;
};