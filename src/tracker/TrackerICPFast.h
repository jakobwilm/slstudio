#ifndef TRACKERICPFAST_H
#define TRACKERICPFAST_H

#include "Tracker.h"

class TrackerICPFast : public Tracker {
    public:
        TrackerICPFast();
        void setReference(PointCloudConstPtr _refPointCloud);
        void determineTransformation(PointCloudConstPtr pointCloud, Eigen::Matrix4f &T, bool &converged);
        ~TrackerICPFast();
        void setCameraMatrix(Eigen::Matrix3f _cameraMatrix){}
    private:
        Eigen::Matrix4f lastTransformation;
        PointCloudConstPtr refPointCloud;

};

#endif // TRACKERICPFAST_H
