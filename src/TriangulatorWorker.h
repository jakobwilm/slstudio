/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#pragma once

#include <QElapsedTimer>
#include <QObject>

#include "CalibrationData.h"
#include "Triangulator.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "PointCloudWidget.h"

class TriangulatorWorker : public QObject {
  Q_OBJECT

public:
  TriangulatorWorker() : frameWidth(0), frameHeight(0), writeToDisk(false) {}
  ~TriangulatorWorker();
public slots:
  void setup();
  void triangulatePointCloud(cv::Mat up, cv::Mat vp, cv::Mat mask,
                             cv::Mat shading);
signals:
  void imshow(const char *windowName, cv::Mat mat, unsigned int x,
              unsigned int y);
  void newPointCloud(PointCloudConstPtr pointCloud);
  void logMessage(const QString &msg);

private:
  unsigned int frameWidth, frameHeight;
  bool writeToDisk;
  CalibrationData *calibration;
  Triangulator *triangulator;
  QElapsedTimer timer;
  bool busy;
};
