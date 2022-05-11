/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#ifndef SLDECODERWORKER_H
#define SLDECODERWORKER_H

#include <QElapsedTimer>
#include <QObject>

#include "Codec.h"

class SLDecoderWorker : public QObject {
  Q_OBJECT

public:
  SLDecoderWorker() : screenCols(0), screenRows(0) {}
  ~SLDecoderWorker();
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
  void error(QString err);
  // void finished();
private:
  Decoder *decoder;
  unsigned int screenCols, screenRows;
  QElapsedTimer timer;
  bool busy;
};

#endif
