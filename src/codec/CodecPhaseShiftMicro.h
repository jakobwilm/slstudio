#pragma once

#include "Codec.h"
#include <array>

// This jointly designed code and its ordering must not be tuned independently.
inline constexpr std::array<float, 10> MicroPhasePitches{
    75.02f, 70.00f, 71.32f, 72.47f, 73.72f, 76.23f, 77.35f, 78.40f, 79.22f, 80.00f};

class EncoderPhaseShiftMicro : public Encoder {
public:
  EncoderPhaseShiftMicro(unsigned int _screenCols, unsigned int _screenRows,
                         CodecDir _dir, const PhaseShiftMicroSettings &settings = {});
  // Encoding
  cv::Mat getEncodingPattern(unsigned int depth) const;

private:
  std::vector<cv::Mat> patterns;
};

class DecoderPhaseShiftMicro : public Decoder {
public:
  DecoderPhaseShiftMicro(unsigned int _screenCols, unsigned int _screenRows,
                         CodecDir _dir, const PhaseShiftMicroSettings &settings = {});
  // Decoding
  void decodeFrames(const std::vector<cv::Mat> &frames, cv::Mat &up, cv::Mat &vp,
                    cv::Mat &mask, cv::Mat &shading) override;

private:
  PhaseShiftMicroSettings settings;
};
