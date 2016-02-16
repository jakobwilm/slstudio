#ifndef TRACKERNDT_H
#define TRACKERNDT_H

#ifndef Q_MOC_RUN  //this is a qt bug with qt version lower than 5
#include "Tracker.h"

#include <pcl/registration/ndt.h>
#endif

class TrackerNDT : public Tracker {
    public:
        TrackerNDT();
        void setReference(PointCloudConstPtr refPointCloud);
        void determineTransformation(PointCloudConstPtr pointCloud, Eigen::Affine3f &T, bool &converged, float &RMS);
        ~TrackerNDT();
    private:
        pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> *ndt;
};

#endif // TRACKER_H
