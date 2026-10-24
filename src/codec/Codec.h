#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <stdexcept>
#include <limits>
#include <cmath>

enum CodecDir {
  CodecDirNone = 0,
  CodecDirHorizontal = 1 << 0,
  CodecDirVertical = 1 << 1,
  CodecDirBoth = CodecDirHorizontal | CodecDirVertical
};

struct PhaseShiftSettings {
  int steps = 3;
  int periods = 16;
};

struct GrayCodeSettings {
  int horizontalBits = 10;
  int verticalBits = 6;
};

struct PhaseShiftHeterodyneSettings {
  int steps = 3;
  double primaryPitch = 60.0;
};

struct PhaseShiftMicroSettings {
  int steps = 3;
};

struct PhaseShiftModulatedSettings {
  int signalSteps = 3;
  int carrierSteps = 3;
  int cueSteps = 3;
  int signalPeriods = 24;
  int carrierPeriods = 16;
};

struct CodecAlgorithmSettings {
  PhaseShiftSettings phaseShift;
  GrayCodeSettings grayCode;
  PhaseShiftHeterodyneSettings heterodyne;
  PhaseShiftSettings spatialUnwrap{3, 4};
  PhaseShiftMicroSettings micro;
  PhaseShiftModulatedSettings modulated;
};

inline void validatePhaseSteps(int steps) {
  if (steps < 3 || steps > 256)
    throw std::invalid_argument("Phase shifting requires 3-256 steps");
}

inline void validateFringePeriods(int periods) {
  if (periods < 1 || periods > 255)
    throw std::invalid_argument("Phase shifting requires 1-255 fringe periods");
}

inline void validatePitch(double pitch) {
  if (!std::isfinite(pitch) || pitch < 2 || pitch > std::numeric_limits<float>::max())
    throw std::invalid_argument("Fringe pitches must be finite and at least two projector pixels");
}

inline void validateCodecSettings(const CodecAlgorithmSettings &settings) {
  if (settings.phaseShift.steps < 3 || settings.phaseShift.steps > 256 ||
      settings.phaseShift.periods < 1 || settings.phaseShift.periods > 255)
    throw std::invalid_argument("Phase shifting requires 3-256 steps and 1-255 fringe periods");
  if (settings.grayCode.horizontalBits < 1 || settings.grayCode.horizontalBits > 31 ||
      settings.grayCode.verticalBits < 1 || settings.grayCode.verticalBits > 31)
    throw std::invalid_argument("Gray code requires 1-31 bits per axis");
  validatePhaseSteps(settings.heterodyne.steps);
  validatePitch(settings.heterodyne.primaryPitch);
  validatePhaseSteps(settings.spatialUnwrap.steps);
  validateFringePeriods(settings.spatialUnwrap.periods);
  validatePhaseSteps(settings.micro.steps);
  validatePhaseSteps(settings.modulated.signalSteps);
  validatePhaseSteps(settings.modulated.carrierSteps);
  validatePhaseSteps(settings.modulated.cueSteps);
  validateFringePeriods(settings.modulated.signalPeriods);
  validateFringePeriods(settings.modulated.carrierPeriods);
}

inline bool operator==(const CodecAlgorithmSettings &a, const CodecAlgorithmSettings &b) {
  return a.phaseShift.steps == b.phaseShift.steps && a.phaseShift.periods == b.phaseShift.periods &&
      a.grayCode.horizontalBits == b.grayCode.horizontalBits && a.grayCode.verticalBits == b.grayCode.verticalBits &&
      a.heterodyne.steps == b.heterodyne.steps && a.heterodyne.primaryPitch == b.heterodyne.primaryPitch &&
      a.spatialUnwrap.steps == b.spatialUnwrap.steps && a.spatialUnwrap.periods == b.spatialUnwrap.periods &&
      a.micro.steps == b.micro.steps &&
      a.modulated.signalSteps == b.modulated.signalSteps && a.modulated.carrierSteps == b.modulated.carrierSteps &&
      a.modulated.cueSteps == b.modulated.cueSteps && a.modulated.signalPeriods == b.modulated.signalPeriods &&
      a.modulated.carrierPeriods == b.modulated.carrierPeriods;
}

inline void validateCodecDimensions(unsigned int columns, unsigned int rows, CodecDir direction) {
  if (!columns || !rows || columns > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
      rows > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
      direction < CodecDirHorizontal || direction > CodecDirBoth)
    throw std::invalid_argument("Invalid codec dimensions or direction");
}

inline void validateCodecFrames(const std::vector<cv::Mat> &frames, unsigned int patternCount,
                                bool requireMono8 = true) {
  if (frames.empty() || frames.size() != patternCount)
    throw std::invalid_argument("Frame count does not match the selected codec and direction");
  for (const auto &frame : frames)
    if (frame.empty() || frame.dims != 2 || frame.channels() != 1 ||
        (requireMono8 && frame.depth() != CV_8U) || frame.type() != frames.front().type() ||
        frame.size() != frames.front().size())
      throw std::invalid_argument(requireMono8 ? "Decoding requires matching Mono8 frames" :
                                               "Decoding requires matching single-channel frames");
}

// Base class for all encoders
class Encoder {
public:
  Encoder(unsigned int _screenCols, unsigned int _screenRows,
          CodecDir _dir = CodecDirHorizontal)
      : N(0), screenCols(_screenCols), screenRows(_screenRows), dir(_dir) {
    validateCodecDimensions(screenCols, screenRows, dir);
  }
  unsigned int getNPatterns() const { return N; }
  CodecDir getDir() const { return dir; }
  // Encoding
  virtual cv::Mat getEncodingPattern(unsigned int depth) const = 0;
  virtual ~Encoder() {}

protected:
  unsigned int N;
  unsigned int screenCols, screenRows;
  CodecDir dir;
};

class Decoder {
public:
  Decoder(unsigned int _screenCols, unsigned int _screenRows,
          CodecDir _dir = CodecDirHorizontal)
      : N(0), screenCols(_screenCols), screenRows(_screenRows), dir(_dir) {
    validateCodecDimensions(screenCols, screenRows, dir);
  }
  unsigned int getNPatterns() const { return N; }
  CodecDir getDir() const { return dir; }
  // Decoding
  void setMinimumModulation(int value) {
    if (value < 0 || value > 255)
      throw std::invalid_argument("Modulation threshold must be between 0 and 255");
    configuredModulation = value;
  }
  virtual void decodeFrames(const std::vector<cv::Mat> &frames, cv::Mat &up, cv::Mat &vp,
                            cv::Mat &mask, cv::Mat &shading) = 0;
  virtual ~Decoder() {}

protected:
  double modulationThreshold(double fallback) const {
    return configuredModulation < 0 ? fallback : configuredModulation;
  }
  int configuredModulation = -1;
  unsigned int N;
  unsigned int screenCols, screenRows;
  CodecDir dir;
};
