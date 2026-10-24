#include "CodecPhaseShift.h"
#include "pstools.h"

namespace {
unsigned int patternCount(CodecDir direction, const PhaseShiftSettings &settings) {
  CodecAlgorithmSettings requestedSettings;
  requestedSettings.phaseShift = settings;
  validateCodecSettings(requestedSettings);
  return (settings.steps + (settings.periods > 1 ? 3 : 0)) * (direction == CodecDirBoth ? 2 : 1);
}
}

EncoderPhaseShift::EncoderPhaseShift(unsigned int columns, unsigned int rows, CodecDir direction,
                                     const PhaseShiftSettings &settings)
    : Encoder(columns, rows, direction) {
  N = patternCount(dir, settings);
  const float pi = static_cast<float>(CV_PI);
  const auto append = [&](unsigned int length, bool horizontal) {
    const auto frequency = [&](int steps, int periods) {
      for (int shift = 0; shift < steps; ++shift) {
        const float phase = 2.0 * pi / steps * shift;
        const float pitch = static_cast<float>(length) / periods;
        cv::Mat pattern = pstools::computePhaseVector(length, phase, pitch);
        if (horizontal) pattern = pattern.t();
        patterns.push_back(std::move(pattern));
      }
    };
    frequency(settings.steps, settings.periods);
    if (settings.periods > 1) frequency(3, 1);
  };
  if (dir & CodecDirHorizontal) append(screenCols, true);
  if (dir & CodecDirVertical) append(screenRows, false);
}

cv::Mat EncoderPhaseShift::getEncodingPattern(unsigned int index) const { return patterns.at(index); }

DecoderPhaseShift::DecoderPhaseShift(unsigned int columns, unsigned int rows, CodecDir direction,
                                     const PhaseShiftSettings &requestedSettings)
    : Decoder(columns, rows, direction), settings(requestedSettings) {
  N = patternCount(dir, settings);
}

void DecoderPhaseShift::decodeFrames(const std::vector<cv::Mat> &frames, cv::Mat &up, cv::Mat &vp,
                                     cv::Mat &mask, cv::Mat &shading) {
  decodeFramesWithThreshold(frames, up, vp, mask, shading, modulationThreshold(55));
}

void DecoderPhaseShift::decodeFramesWithThreshold(const std::vector<cv::Mat> &frames,
                                                 cv::Mat &up, cv::Mat &vp, cv::Mat &mask,
                                                 cv::Mat &shading, double threshold) const {
  validateCodecFrames(frames, N);
  up.release();
  vp.release();
  const float pi = static_cast<float>(CV_PI);
  const unsigned int perAxis = N / (dir == CodecDirBoth ? 2 : 1);
  const auto decode = [&](unsigned int offset, unsigned int length, cv::Mat &map, bool first) {
    cv::Mat magnitude;
    if (settings.steps == 3) {
      if (first)
        pstools::getPhaseAndMagnitude(frames[offset], frames[offset + 1], frames[offset + 2], map, magnitude);
      else
        map = pstools::getPhase(frames[offset], frames[offset + 1], frames[offset + 2]);
    } else {
      const std::vector<cv::Mat> main(frames.begin() + offset, frames.begin() + offset + settings.steps);
      const auto components = pstools::getDFTComponents(main);
      cv::phase(components[2], -components[3], map);
      if (first) {
        cv::magnitude(components[2], components[3], magnitude);
        magnitude *= 6.0 / settings.steps;
      }
    }
    if (settings.periods > 1) {
      const auto cue = offset + settings.steps;
      map = pstools::unwrapWithCue(map,
          pstools::getPhase(frames[cue], frames[cue + 1], frames[cue + 2]), settings.periods);
    }
    map *= length / (2 * pi);
    if (first) {
      mask = magnitude > threshold;
      magnitude.convertTo(shading, CV_8U);
    }
  };
  if (dir & CodecDirHorizontal) decode(0, screenCols, up, true);
  if (dir & CodecDirVertical) decode(dir & CodecDirHorizontal ? perAxis : 0,
                                    screenRows, vp, !(dir & CodecDirHorizontal));
}
