/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) 2013 -- 2014 Jakob Wilm, DTU, Kgs.Lyngby, Denmark
 *
*/

#ifndef SLRECONSTRUCTORWORKER_H
#define SLRECONSTRUCTORWORKER_H

#include <QObject>
#include <QTime>

#include "CalibrationData.h"
#include "Triangulator.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "SLPointCloudWidget.h"

class SLTriangulatorWorker : public QObject {
    Q_OBJECT

    public:
        SLTriangulatorWorker() : frameWidth(0), frameHeight(0), writeToDisk(false){}
        ~SLTriangulatorWorker();
    public slots:
        void setup();
        void triangulatePointCloud(cv::Mat up, cv::Mat vp, cv::Mat mask, cv::Mat shading);
    signals:
        void imshow(const char* windowName, cv::Mat mat, unsigned int x, unsigned int y);
        void newPointCloud(PointCloudConstPtr pointCloud);
        void error(QString err);
        //void finished();
    private:
        unsigned int frameWidth, frameHeight;
        bool writeToDisk;
        CalibrationData *calibration;
        Triangulator *triangulator;
        QTime time;
        bool busy;
};

#endif
