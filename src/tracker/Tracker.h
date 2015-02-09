#ifndef TRACKER_H
#define TRACKER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr PointCloudConstPtr;

class Tracker {
    public:
        Tracker(){}
        virtual void setReference(PointCloudConstPtr refPointCloud) = 0;
        virtual void determineTransformation(PointCloudConstPtr pointCloud, Eigen::Affine3f &T, bool &converged, float &RMS) = 0;
        virtual ~Tracker(){}
        virtual void setCameraMatrix(Eigen::Matrix3f _cameraMatrix) = 0;
    protected:

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // TRACKER_H
