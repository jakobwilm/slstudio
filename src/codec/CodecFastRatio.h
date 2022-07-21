#pragma once

#include "Codec.h"

class EncoderFastRatio : public Encoder {
public:
  EncoderFastRatio(unsigned int _screenCols, unsigned int _screenRows,
                   CodecDir _dir);
  // Encoding
  cv::Mat getEncodingPattern(unsigned int depth);

private:
  std::vector<cv::Mat> patterns;
};

class DecoderFastRatio : public Decoder {
public:
  DecoderFastRatio(unsigned int _screenCols, unsigned int _screenRows,
                   CodecDir _dir);
  // Decoding
  void setFrame(unsigned int depth, cv::Mat frame);
  void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading);

private:
  std::vector<cv::Mat> frames;
};
