#include "TrackerNDT.h"

#include <pcl/filters/approximate_voxel_grid.h>

TrackerNDT::TrackerNDT(){

    // Set up the registration algorithm object
    ndt = new pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB>();

    // Set settings
    ndt->setMaximumIterations(50);
    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt->setTransformationEpsilon(0.01);
    // Setting maximum step size for More-Thuente line search.
    ndt->setStepSize(0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt->setResolution(20);

}

void TrackerNDT::setReference(PointCloudConstPtr refPointCloud){

    ndt->setInputTarget(refPointCloud);

}

void TrackerNDT::determineTransformation(PointCloudConstPtr pointCloud, Eigen::Affine3f &T, bool &converged, float &RMS){

    // Filtering input scan to roughly 10% of original size to increase speed of registration.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredPointCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> approximateVoxelFilter;
    approximateVoxelFilter.setLeafSize(5, 5, 5);
    approximateVoxelFilter.setInputCloud(pointCloud);
    approximateVoxelFilter.filter(*filteredPointCloud);

    std::cout << "Downsampled: " << pointCloud->size() << "->" << filteredPointCloud->size() << std::endl;

    ndt->setInputSource(filteredPointCloud);

    pcl::PointCloud<pcl::PointXYZRGB> registeredPointCloud;
    ndt->align(registeredPointCloud);

    std::cout << "Nr of iterations: " << ndt->getFinalNumIteration() << std::endl;

    T = ndt->getFinalTransformation();

    converged = ndt->hasConverged();

    std::cout << "Converged: " << converged << std::endl;
}

TrackerNDT::~TrackerNDT(){
    delete ndt;
}
