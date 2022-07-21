#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

enum CodecDir {
  CodecDirNone = 0,
  CodecDirHorizontal = 1 << 0,
  CodecDirVertical = 1 << 1,
  CodecDirBoth = CodecDirHorizontal | CodecDirVertical
};

// Base class for all encoders
class Encoder {
public:
  Encoder(unsigned int _screenCols, unsigned int _screenRows,
          CodecDir _dir = CodecDirHorizontal)
      : N(0), screenCols(_screenCols), screenRows(_screenRows), dir(_dir) {}
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
      : N(0), screenCols(_screenCols), screenRows(_screenRows), dir(_dir) {}
  unsigned int getNPatterns() const { return N; }
  CodecDir getDir() const { return dir; }
  // Decoding
  virtual void setFrame(unsigned int depth, const cv::Mat frame) = 0;
  virtual void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask,
                            cv::Mat &shading) const = 0;
  virtual ~Decoder() {}

protected:
  unsigned int N;
  unsigned int screenCols, screenRows;
  CodecDir dir;
};
