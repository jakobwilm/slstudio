#pragma once

#include "Codec.h"

// 8 step phase shifting codec with phase unwrapping

class EncoderPhaseShiftNStep : public Encoder {
public:
  EncoderPhaseShiftNStep(unsigned int _screenCols, unsigned int _screenRows,
                         CodecDir _dir);
  // Encoding
  cv::Mat getEncodingPattern(unsigned int depth) const;

private:
  std::vector<cv::Mat> patterns;
};

class DecoderPhaseShiftNStep : public Decoder {
public:
  DecoderPhaseShiftNStep(unsigned int _screenCols, unsigned int _screenRows,
                         CodecDir _dir);
  // Decoding
  void setFrame(unsigned int depth, cv::Mat frame);
  void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask,
                    cv::Mat &shading) const;

private:
  std::vector<cv::Mat> frames;
};
