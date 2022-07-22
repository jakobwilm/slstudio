/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#pragma once

#include <QMainWindow>
#include <QObject>
#include <QThread>

#include "Camera.h"
#include "Codec.h"
#include "Projector.h"

#include "DecoderWorker.h"
#include "TriangulatorWorker.h"

enum ScanAquisitionMode { aquisitionContinuous, aquisitionSingle };

class ScanWorker : public QObject {
  Q_OBJECT

public:
  ScanWorker(QObject * /*parent*/) : isWorking(false) {}
  ~ScanWorker();
  static bool setPatterns(const Encoder *encoder, Projector *projector);
public slots:
  void setup();
  void doWork();
  void stopWorking();
signals:
  void showHistogram(cv::Mat im);
  void newFrame(cv::Mat frame);
  void newFrameSeq(std::vector<cv::Mat> frameSeq);
  void logMessage(const QString &msg);
  void finished();

private:
  bool isWorking;
  std::unique_ptr<Camera> camera;
  std::unique_ptr<Projector> projector;
  std::unique_ptr<Encoder> encoder;

  CameraTriggerMode triggerMode;
  ScanAquisitionMode aquisition;
  bool writeToDisk;
};
