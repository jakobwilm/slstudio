#pragma once

#include "Codec.h"

class EncoderPhaseShift4 : public Encoder {
public:
  EncoderPhaseShift4(unsigned int _screenCols, unsigned int _screenRows,
                     CodecDir _dir);
  // Encoding
  cv::Mat getEncodingPattern(unsigned int depth) const;

private:
  std::vector<cv::Mat> patterns;
};

class DecoderPhaseShift4 : public Decoder {
public:
  DecoderPhaseShift4(unsigned int _screenCols, unsigned int _screenRows,
                     CodecDir _dir);
  // Decoding
  void setFrame(unsigned int depth, cv::Mat frame);
  void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask,
                    cv::Mat &shading) const;

private:
  std::vector<cv::Mat> frames;
};
