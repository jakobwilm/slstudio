/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#ifndef SLRECONSTRUCTORWORKER_H
#define SLRECONSTRUCTORWORKER_H

#include <QElapsedTimer>
#include <QObject>

#include "CalibrationData.h"
#include "Triangulator.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "SLPointCloudWidget.h"

class SLTriangulatorWorker : public QObject {
  Q_OBJECT

public:
  SLTriangulatorWorker() : frameWidth(0), frameHeight(0), writeToDisk(false) {}
  ~SLTriangulatorWorker();
public slots:
  void setup();
  void triangulatePointCloud(cv::Mat up, cv::Mat vp, cv::Mat mask,
                             cv::Mat shading);
signals:
  void imshow(const char *windowName, cv::Mat mat, unsigned int x,
              unsigned int y);
  void newPointCloud(PointCloudConstPtr pointCloud);
  void error(QString err);
  // void finished();
private:
  unsigned int frameWidth, frameHeight;
  bool writeToDisk;
  CalibrationData *calibration;
  Triangulator *triangulator;
  QElapsedTimer timer;
  bool busy;
};

#endif
