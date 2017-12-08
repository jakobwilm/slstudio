#include "TrackerICP.h"
#include <pcl/features/integral_image_normal.h>

#include <pcl/io/ply_io.h>

TrackerICP::TrackerICP(){
//std::cout << "TrackerICP Constructor..." << std::endl;
    icp = new pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>;

    // Set up the registration algorithm object
    icp->setRANSACIterations(0);
    icp->setUseReciprocalCorrespondences(false);
    // Set the max Eucl. correspondence distance in native point cloud unit
    icp->setMaxCorrespondenceDistance(100.0);
    // Set the maximum number of iterations (criterion 1)
    icp->setMaximumIterations(40);
    // Set the transformation epsilon (criterion 2)
    icp->setTransformationEpsilon(1e-4);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp->setEuclideanFitnessEpsilon(1e-4);

    // Set up downsampling filter
    approximateVoxelFilter = boost::shared_ptr< pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> >(new pcl::ApproximateVoxelGrid<pcl::PointXYZRGB>);
    approximateVoxelFilter->setLeafSize(1.5, 1.5, 1.5);

    // Set up correspondance estimator
    correspondenceEstimator = boost::shared_ptr< CorrEstOrgProjFast<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> >(new CorrEstOrgProjFast<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>);
    icp->setCorrespondenceEstimation(correspondenceEstimator);

    // Set up correspondance rejector (boundary)
    correspondenceRejectorBoundary = boost::shared_ptr<CorrRejectOrgBoundFast>(new CorrRejectOrgBoundFast);
    correspondenceRejectorBoundary->setDepthStepThreshhold(3.0);
    correspondenceRejectorBoundary->setNumberOfBoundaryNaNs(3);
    correspondenceRejectorBoundary->setWindowSize(2); // 8 neighborhood
    icp->addCorrespondenceRejector(correspondenceRejectorBoundary);

//    // Set up correspondance rejector (median)
//    correspondenceRejectorMedian = boost::shared_ptr<pcl::registration::CorrespondenceRejectorMedianDistance>(new pcl::registration::CorrespondenceRejectorMedianDistance);
//    correspondenceRejectorMedian->setMedianFactor(1.5);
//    icp->addCorrespondenceRejector(correspondenceRejectorMedian);

    transformationEstimator = boost::shared_ptr< pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> >(new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>);
    icp->setTransformationEstimation(transformationEstimator);

    lastTransformation = Eigen::Matrix4f::Identity();

    poseFilter = new PoseFilter();
}

void TrackerICP::setReference(PointCloudConstPtr refPointCloud){

    // Compute normals
    refPointCloudNormals = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::copyPointCloud(*refPointCloud, *refPointCloudNormals);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT); // around 40 ms
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(refPointCloud);
    ne.compute(*refPointCloudNormals);

    correspondenceEstimator->setInputTarget(refPointCloudNormals);
    correspondenceRejectorBoundary->setInputTarget<pcl::PointXYZRGBNormal>(refPointCloudNormals);

    icp->setInputTarget(refPointCloudNormals);

}

void TrackerICP::setCameraMatrix(Eigen::Matrix3f _cameraMatrix){

    // Set camera parameters
    correspondenceEstimator->setProjectionMatrix(_cameraMatrix);
}

void TrackerICP::determineTransformation(PointCloudConstPtr pointCloud, Eigen::Affine3f &T, bool &converged, float &RMS){

    // Filter
    approximateVoxelFilter->setInputCloud(pointCloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    approximateVoxelFilter->filter(*filteredPointCloud);
    std::cout << "Reduced number of points in source from " << pointCloud->points.size() << " to " << filteredPointCloud->points.size() << std::endl;

////pcl::io::savePLYFileBinary(fileName.toStdString(), *pointCloudPCL);
//pcl::PLYWriter w;
//// Write to ply in binary without camera
//w.write<pcl::PointXYZRGB> ("filteredPointCloud.ply", *filteredPointCloud, true, false);

    // Add normal fields to source (due to PCL bug)
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointCloudNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::copyPointCloud(*filteredPointCloud, *pointCloudNormals);

//    correspondenceEstimator->setInputSource(pointCloudNormals);
//    correspondenceRejectorBoundary->setInputSource<pcl::PointXYZRGBNormal>(pointCloudNormals);
    icp->setInputSource(pointCloudNormals);

    // Align
    pcl::PointCloud<pcl::PointXYZRGBNormal> registeredPointCloud;
    icp->align(registeredPointCloud, lastTransformation.matrix());

    //std::cout << "Nr of iterations: " << icp->nr_iterations_ << std::endl;

    // Computes nearest neighbors from scratch
    //std::cout << "Mean error: " << icp->getFitnessScore(100.0) << std::endl;

    pcl::Correspondences correspondences = *(icp->correspondences_);
    int nCorrespondences = correspondences.size();
    std::cout << "Number of correspondences: " << nCorrespondences << std::endl;
    float sumDistances = 0.0;
    for(int i=0; i<nCorrespondences; i++){
       sumDistances += correspondences[i].distance;
    }
    RMS = sumDistances/nCorrespondences;
    std::cout << "RMS: " << RMS << std::endl;

    //std::cout << "Median distance: " << correspondenceRejectorMedian->getMedianDistance() << std::endl;

    converged = icp->hasConverged();

    std::cout << "Converged: " << converged << std::endl;

    if(converged){
        Eigen::Affine3f Traw;
        Traw.matrix() = icp->getFinalTransformation();
        poseFilter->filterPoseEstimate(Traw, T);
        //T = Traw;
        lastTransformation = T;
    } else {
        T = lastTransformation;
    }

}

TrackerICP::~TrackerICP(){
    delete icp;
}
