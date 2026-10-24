#pragma once

#include "Codec.h"

class EncoderPhaseShiftModulated : public Encoder {
public:
  EncoderPhaseShiftModulated(unsigned int _screenCols, unsigned int _screenRows,
                             CodecDir _dir, const PhaseShiftModulatedSettings &settings = {});
  // Encoding
  cv::Mat getEncodingPattern(unsigned int depth) const;

private:
  std::vector<cv::Mat> patterns;
};

class DecoderPhaseShiftModulated : public Decoder {
public:
  DecoderPhaseShiftModulated(unsigned int _screenCols, unsigned int _screenRows,
                             CodecDir _dir, const PhaseShiftModulatedSettings &settings = {});
  // Decoding
  void decodeFrames(const std::vector<cv::Mat> &frames, cv::Mat &up, cv::Mat &vp,
                    cv::Mat &mask, cv::Mat &shading) override;

private:
  PhaseShiftModulatedSettings settings;
};
