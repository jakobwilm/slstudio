#ifndef TRACKERNDT_H
#define TRACKERNDT_H

#include "Tracker.h"

#include <pcl/registration/ndt.h>

class TrackerNDT : public Tracker {
    public:
        TrackerNDT();
        void setReference(PointCloudConstPtr refPointCloud);
        void determineTransformation(PointCloudConstPtr pointCloud, Eigen::Affine3f &T, bool &converged);
        ~TrackerNDT();
    private:
        pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> *ndt;
};

#endif // TRACKER_H
