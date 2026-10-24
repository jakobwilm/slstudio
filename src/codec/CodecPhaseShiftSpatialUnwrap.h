#pragma once

#include "Codec.h"
#include "spatialUnwrapping.h"

class EncoderPhaseShiftSpatialUnwrap : public Encoder {
public:
  EncoderPhaseShiftSpatialUnwrap(unsigned int _screenCols, unsigned int _screenRows,
                           CodecDir _dir, const PhaseShiftSettings &settings = {3, 4});
  // Encoding
  cv::Mat getEncodingPattern(unsigned int depth) const;

private:
  std::vector<cv::Mat> patterns;
};

class DecoderPhaseShiftSpatialUnwrap : public Decoder {
public:
  DecoderPhaseShiftSpatialUnwrap(unsigned int _screenCols, unsigned int _screenRows,
                           CodecDir _dir, const PhaseShiftSettings &settings = {3, 4},
                           int centerFringeOrder = DefaultCenterFringeOrder);
  // Decoding
  void decodeFrames(const std::vector<cv::Mat> &frames, cv::Mat &up, cv::Mat &vp,
                    cv::Mat &mask, cv::Mat &shading) override;

private:
  PhaseShiftSettings settings;
  const int centerFringeOrder;
};
