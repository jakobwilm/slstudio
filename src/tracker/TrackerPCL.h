#ifndef TRACKERPCL_H
#define TRACKERPCL_H

#include "Tracker.h"

#include <pcl/tracking/kld_adaptive_particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/filters/approximate_voxel_grid.h>

//typedef pcl::tracking::KLDAdaptiveParticleFilterTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYZRPY> TrackerType;
typedef pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYZRPY> TrackerType;

//typedef pcl::tracking::ParticleFilterTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYZRPY> TrackerType;
typedef pcl::tracking::ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGB> CoherenceType;

typedef pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> FilterType;

class TrackerPCL : public Tracker {
    public:
        TrackerPCL();
        void setReference(PointCloudConstPtr refPointCloud);
        void determineTransformation(PointCloudConstPtr pointCloud, Eigen::Affine3f &T, bool &converged);
        ~TrackerPCL();
        void setCameraMatrix(Eigen::Matrix3f _cameraMatrix){}


    private:
        boost::shared_ptr<TrackerType> tracker;
        boost::shared_ptr<CoherenceType> coherence;
        boost::shared_ptr<FilterType> approximateVoxelFilter;
        //Eigen::Matrix4f lastTransformation;

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr refPointCloudNormals;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif // TRACKER_H
