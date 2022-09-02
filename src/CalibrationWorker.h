/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#pragma once

#include "CalibrationData.h"
#include "Codec.h"
#include <QObject>

class CalibrationWorker : public QObject {
  Q_OBJECT
public:
  CalibrationWorker(QObject *parent, unsigned int _screenCols,
                    unsigned int _screenRows);
  bool calibrate(CalibrationData &calibrationData,
                 std::vector<std::vector<cv::Mat>> &frameSeqs,
                 std::vector<size_t> &activeSeqs);

  ~CalibrationWorker() {}
public slots:

  bool findPartialCirclesGrid(const cv::Mat &im, std::vector<cv::Point2f> &q,
                              std::vector<cv::Point3f> &Q,
                              const float circleSpacing);
signals:
  void newSequenceResult(const cv::Mat &img, size_t idx, bool success);
  void logMessage(const QString &message);

private:
  unsigned int screenCols, screenRows;
  std::unique_ptr<Decoder> decoder;
  cv::Ptr<cv::FeatureDetector> blobDetector;
};
