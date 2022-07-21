/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#pragma once

#include <QElapsedTimer>
#include <QObject>

#include "Codec.h"

class DecoderWorker : public QObject {
  Q_OBJECT

public:
  DecoderWorker() : screenResX(0), screenResY(0) {}
  ~DecoderWorker();
public slots:
  void setup();
  void decodeSequence(std::vector<cv::Mat> frameSeq);
signals:
  void imshow(const char *windowName, cv::Mat mat, unsigned int x,
              unsigned int y);
  void showShading(cv::Mat mat);
  void showDecoderUp(cv::Mat mat);
  void showDecoderVp(cv::Mat mat);
  void newUpVp(cv::Mat up, cv::Mat vp, cv::Mat mask, cv::Mat shading);
  void logMessage(const QString &msg);

private:
  std::unique_ptr<Decoder> decoder;
  unsigned int screenResX, screenResY;
  QElapsedTimer timer;
  bool busy;
};
