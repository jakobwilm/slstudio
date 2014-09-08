#ifndef TRACKERICP_H
#define TRACKERICP_H

#include "Tracker.h"

#include "PoseFilter.h"

#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>

//#include <pcl/registration/correspondence_estimation.h>
#include "CorrEstOrgProjFast.h"
//#include "CorrEstKdTreeFast.h"

//#include <pcl/registration/correspondence_rejection_organized_boundary.h>
#include "CorrRejectOrgBoundFast.h"

#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>


//typedef pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> ICPType;
typedef pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ICPType;

typedef pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> FilterType;

//typedef pcl::registration::CorrespondenceEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> CorrespondenceEstimationType;
typedef CorrEstOrgProjFast<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> CorrespondenceEstimationType;
//typedef CorrEstKdTreeFast<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> CorrespondenceEstimationType;

//typedef pcl::registration::CorrespondenceRejectionOrganizedBoundary CorrespondenceRejectionType;
//typedef boost::shared_ptr< CorrespondenceRejectionType > CorrespondenceRejectionTypePtr;
typedef CorrRejectOrgBoundFast CorrespondenceRejectionType;

//typedef pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> TransformationEstimationType;
typedef pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> TransformationEstimationType;


class TrackerICP : public Tracker {
    public:
        TrackerICP();
        void setReference(PointCloudConstPtr refPointCloud);
        void determineTransformation(PointCloudConstPtr pointCloud, Eigen::Affine3f &T, bool &converged);
        ~TrackerICP();
        void setCameraMatrix(Eigen::Matrix3f _cameraMatrix);
    private:
        ICPType *icp;
        FilterType *approximateVoxelFilter;
        CorrespondenceEstimationType::Ptr correspondenceEstimator;
        CorrespondenceRejectionType::Ptr correspondenceRejector;
        TransformationEstimationType::Ptr transformationEstimator;
        Eigen::Affine3f lastTransformation;

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr refPointCloudNormals;

        PoseFilter *poseFilter;

};

#endif // TRACKER_H
