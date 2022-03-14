/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#ifndef SLWORKER_H
#define SLWORKER_H

#include <QMainWindow>
#include <QObject>
#include <QThread>

#include "Camera.h"
#include "Codec.h"
#include "Projector.h"

#include "SLDecoderWorker.h"
#include "SLTriangulatorWorker.h"

enum ScanAquisitionMode { aquisitionContinuous, aquisitionSingle };

class SLScanWorker : public QObject {
  Q_OBJECT

public:
  SLScanWorker(QObject * /*parent*/) : isWorking(false) {}
  ~SLScanWorker();
public slots:
  void setup();
  void doWork();
  void stopWorking();
signals:
  void showHistogram(cv::Mat im);
  void newFrame(cv::Mat frame);
  void newFrameSeq(std::vector<cv::Mat> frameSeq);
  void error(QString err);
  void finished();

private:
  bool isWorking;
  Camera *camera;
  Projector *projector;
  Encoder *encoder;

  CameraTriggerMode triggerMode;
  ScanAquisitionMode aquisition;
  bool writeToDisk;
};

#endif
