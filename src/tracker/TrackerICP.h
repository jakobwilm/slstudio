#ifndef TRACKERICP_H
#define TRACKERICP_H

#include "Tracker.h"

#include "PoseFilter.h"

#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>

//#include <pcl/registration/correspondence_estimation.h>
//#include "CorrEstKdTreeFast.h"
#include "CorrEstOrgProjFast.h"

//#include <pcl/registration/correspondence_rejection_organized_boundary.h>
#include "CorrRejectOrgBoundFast.h"
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>

#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>

class TrackerICP : public Tracker {
    public:
        TrackerICP();
        void setReference(PointCloudConstPtr refPointCloud);
        void determineTransformation(PointCloudConstPtr pointCloud, Eigen::Affine3f &T, bool &converged, float &RMS);
        ~TrackerICP();
        void setCameraMatrix(Eigen::Matrix3f _cameraMatrix);
    private:

        pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> *icp;

        pcl::ApproximateVoxelGrid<pcl::PointXYZRGB>::Ptr approximateVoxelFilter;

        //CorrEstKdTreeFast<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr correspondenceEstimator;
        CorrEstOrgProjFast<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr correspondenceEstimator;

        CorrRejectOrgBoundFast::Ptr correspondenceRejectorBoundary;
        pcl::registration::CorrespondenceRejectorMedianDistance::Ptr correspondenceRejectorMedian;
        pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr correspondenceRejectorVar;

        //pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr transformationEstimator;
        pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr transformationEstimator;
        //pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr transformationEstimator;

        Eigen::Affine3f lastTransformation;

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr refPointCloudNormals;

        PoseFilter *poseFilter;

};

#endif // TRACKER_H
