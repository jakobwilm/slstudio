/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) 2013 -- 2014 Jakob Wilm, DTU, Kgs.Lyngby, Denmark
 *
*/

#ifndef SLTRACKERWORKER_H
#define SLTRACKERWORKER_H

#include <QObject>
#include <QTime>

#include <fstream>

#include "Tracker.h"

#ifndef Q_MOC_RUN
    #include <Eigen/Eigen>
#endif


class SLTrackerWorker : public QObject {
    Q_OBJECT

    public:
        SLTrackerWorker() : busy(false), tracker(NULL), referenceSet(false), writeToDisk(false){}
        ~SLTrackerWorker();
    public slots:
        void setup();
        void trackPointCloud(PointCloudConstPtr pointCloud);
        void setReference(PointCloudConstPtr referencePointCloud);
    signals:
        void newPoseEstimate(const Eigen::Affine3f & T);
        void error(QString err);
    private:
        bool busy;
        Tracker *tracker;
        QTime performanceTime;
        QTime trackingTime;
        bool referenceSet;
        bool writeToDisk;
        std::ofstream *ofStream;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
