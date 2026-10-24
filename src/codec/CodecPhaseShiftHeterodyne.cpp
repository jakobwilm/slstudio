#include "CodecPhaseShiftHeterodyne.h"
#include "pstools.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr double tau = 2 * CV_PI;
}

EncoderPhaseShiftHeterodyne::EncoderPhaseShiftHeterodyne(unsigned int columns, unsigned int rows, CodecDir direction,
                                                       const PhaseShiftHeterodyneSettings &settings)
    : Encoder(columns, rows, direction) {
  CodecAlgorithmSettings requestedSettings;
  requestedSettings.heterodyne = settings;
  validateCodecSettings(requestedSettings);
  auto encode = [&](unsigned int length, bool horizontal) {
    for (const double cycles : {length / settings.primaryPitch, length / settings.primaryPitch + 1})
      for (int shift = 0; shift < settings.steps; ++shift) {
        cv::Mat pattern = pstools::computePhaseVector(length, shift * tau / settings.steps, length / cycles);
        if (horizontal) pattern = pattern.t();
        patterns.push_back(pattern);
      }
  };
  if (dir & CodecDirHorizontal) encode(screenCols, true);
  if (dir & CodecDirVertical) encode(screenRows, false);
  N = static_cast<unsigned int>(patterns.size());
}

cv::Mat EncoderPhaseShiftHeterodyne::getEncodingPattern(unsigned int index) const {
  return patterns.at(index);
}

DecoderPhaseShiftHeterodyne::DecoderPhaseShiftHeterodyne(unsigned int columns, unsigned int rows, CodecDir direction,
                                                       const PhaseShiftHeterodyneSettings &requestedSettings)
    : Decoder(columns, rows, direction), settings(requestedSettings) {
  CodecAlgorithmSettings algorithmSettings;
  algorithmSettings.heterodyne = settings;
  validateCodecSettings(algorithmSettings);
  N = 2 * settings.steps * (dir == CodecDirBoth ? 2 : 1);
}

void DecoderPhaseShiftHeterodyne::decodeFrames(const std::vector<cv::Mat> &frames, cv::Mat &up, cv::Mat &vp,
                                              cv::Mat &mask, cv::Mat &shading) {
  validateCodecFrames(frames, N);
  const auto size = frames.front().size();
  up.release();
  vp.release();
  mask = cv::Mat1b(size, std::uint8_t(255));
  shading = cv::Mat1b(size, std::uint8_t(255));
  std::vector<double> cosineWeights(settings.steps), sineWeights(settings.steps);
  for (int shift = 0; shift < settings.steps; ++shift) {
    cosineWeights[shift] = std::cos(shift * tau / settings.steps) * 6.0 / settings.steps;
    sineWeights[shift] = std::sin(shift * tau / settings.steps) * 6.0 / settings.steps;
  }
  auto decode = [&](unsigned int offset, unsigned int length, cv::Mat &map) {
    map.create(size, CV_32FC1);
    const double primaryCycles = length / settings.primaryPitch;
    for (int row = 0; row < size.height; ++row) {
      for (int col = 0; col < size.width; ++col) {
        double phase[2], magnitude[2];
        for (int frequency = 0; frequency < 2; ++frequency) {
          const auto index = offset + settings.steps * frequency;
          double cosine = 0, sine = 0;
          if (settings.steps == 3) {
            const double a = frames[index].at<std::uint8_t>(row, col);
            const double b = frames[index + 1].at<std::uint8_t>(row, col);
            const double c = frames[index + 2].at<std::uint8_t>(row, col);
            cosine = 2 * a - b - c;
            sine = std::sqrt(3.0) * (b - c);
          } else {
            for (int shift = 0; shift < settings.steps; ++shift) {
              const double intensity = frames[index + shift].at<std::uint8_t>(row, col);
              cosine += intensity * cosineWeights[shift];
              sine += intensity * sineWeights[shift];
            }
          }
          phase[frequency] = std::atan2(sine, cosine);
          if (phase[frequency] < 0) phase[frequency] += tau;
          magnitude[frequency] = std::hypot(cosine, sine);
        }
        double cue = phase[1] - phase[0];
        if (cue < 0) cue += tau;
        double coordinate = 0;
        for (int frequency = 0; frequency < 2; ++frequency) {
          const double cycles = primaryCycles + frequency;
          // Signed rounding is essential near the first fringe; CV_8U rounding clips negative orders.
          const double order = std::round((cue * cycles - phase[frequency]) / tau);
          coordinate += (phase[frequency] + tau * order) * length / (2 * tau * cycles);
        }
        map.at<float>(row, col) = static_cast<float>(coordinate);
        const auto modulation = cv::saturate_cast<std::uint8_t>(std::min(magnitude[0], magnitude[1]));
        auto &shade = shading.at<std::uint8_t>(row, col);
        shade = std::min(shade, modulation);
        if (modulation <= modulationThreshold(55) || coordinate < 0 || coordinate >= length)
          mask.at<std::uint8_t>(row, col) = 0;
      }
    }
  };
  if (dir & CodecDirHorizontal) decode(0, screenCols, up);
  if (dir & CodecDirVertical) decode(dir == CodecDirBoth ? 2 * settings.steps : 0, screenRows, vp);
}
