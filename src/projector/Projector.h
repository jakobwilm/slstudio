#pragma once

#include <vector>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <opencv2/core.hpp>
#include "patternTools.h"

namespace slstudio {

struct ProjectorCapabilities {
  bool hardwareSequence = false;
  bool triggerOutput = false;
  bool persistentPatterns = false;
};

struct ProjectorSequenceSettings {
  std::chrono::microseconds exposure{16667};
  std::chrono::microseconds framePeriod{16667};
  bool repeat = true;
};

// Called on the programming thread; fraction describes work completed, not elapsed time.
using ProjectorProgrammingProgress = std::function<void(float fraction, const char *stage)>;

class Projector {
public:
  virtual ~Projector() = default;
  virtual cv::Size resolution() const = 0;
  virtual ProjectorGeometry geometry() const { return {resolution(), resolution()}; }
  virtual ProjectorCapabilities capabilities() const { return {}; }
  // Display backends prepare full-size 8-bit grayscale/RGB storage and finish uploads here.
  virtual void setPatterns(const std::vector<cv::Mat> &patterns) = 0;
  // Select a prepared pattern without allocating or uploading pattern pixels.
  virtual void present(std::size_t index) = 0;
  virtual void blank(bool white = false) = 0;
  virtual void showSolid(std::uint8_t level) {
    setPatterns({cv::Mat(resolution(), CV_8UC1, cv::Scalar(level))});
    present(0);
  }
  virtual void programPatterns(const std::vector<cv::Mat> &,
                               const ProjectorProgrammingProgress & = {}) {
    throw std::logic_error("This projector does not support persistent pattern programming");
  }
  virtual void configureSequence(const ProjectorSequenceSettings &) {
    throw std::logic_error("This projector does not support hardware-timed sequences");
  }
  virtual void startSequence() {
    throw std::logic_error("This projector does not support hardware-timed sequences");
  }
  virtual void stopSequence() {}
};

inline void validateProjectorSequence(const ProjectorSequenceSettings &settings) {
  if (settings.exposure.count() <= 0 || settings.framePeriod < settings.exposure ||
      settings.framePeriod.count() > std::numeric_limits<std::uint32_t>::max())
    throw std::invalid_argument("Projector timing requires positive exposure <= frame period, in microseconds");
}
}