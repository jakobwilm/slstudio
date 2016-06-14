/*
 *
 MVTracker - Multi View Tracking for Pose Estimation
 (c) 2014 Jakob Wilm, DTU, Kgs.Lyngby, Denmark
 *
*/

#ifndef PoseFilter_H
#define PoseFilter_H

#include <QObject>
#include <QTime>

#include <opencv2/video/tracking.hpp>
#include <Eigen/Eigen>

class PoseFilter : public QObject{
    Q_OBJECT

    public:
        PoseFilter();
        ~PoseFilter();
        void filterPoseEstimate(const Eigen::Affine3f &Told, Eigen::Affine3f &Tnew);
    public slots:
        void filterPoseEstimate(const Eigen::Affine3f & T);
    signals:
        void newFilteredPoseEstimate(const Eigen::Affine3f & T);
        // For debugging purposes
        //void imshow(const char* windowName, cv::Mat im, unsigned int x, unsigned int y);
    private:
        QTime time;

        // Extended Kalman filter structures
        // Notation according to Goddard "Pose and Motion Estimation...", U Tennessee, 1997
        Eigen::Matrix<float, 13, 1> s; // state [tx,ty,tz,qw,qx,qy,qz,vx,xy,vz,ox,oy,oz]
        Eigen::Matrix<float, 13, 13> Q; // process noise covariance matrix (Q)
        Eigen::Matrix<float, 7, 7> R; // measurement noise covariance matrix (R)
        Eigen::Matrix<float, 13, 13> P; // error covariance

};

#endif // PoseFilter_H
