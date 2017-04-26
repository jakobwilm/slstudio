#include "TrackerPCL.h"

#include <pcl/tracking/distance_coherence.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

TrackerPCL::TrackerPCL(){

    tracker = boost::shared_ptr<TrackerType>(new TrackerType());

    //Set all parameters
    pcl::tracking::ParticleXYZRPY bin_size;
    bin_size.x = 0.1f;
    bin_size.y = 0.1f;
    bin_size.z = 0.1f;
    bin_size.roll = 0.1f;
    bin_size.pitch = 0.1f;
    bin_size.yaw = 0.1f;
    tracker->setMaximumParticleNum(1000);
    tracker->setDelta(0.99);
    tracker->setEpsilon(0.2);
    tracker->setBinSize(bin_size);

    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;

    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

    tracker->setTrans(Eigen::Affine3f::Identity ());
    tracker->setStepNoiseCovariance(default_step_covariance);
    tracker->setInitialNoiseCovariance(initial_noise_covariance);
    tracker->setInitialNoiseMean(default_initial_mean);
    tracker->setIterationNum(10);
    tracker->setParticleNum(500);
    tracker->setResampleLikelihoodThr(0.00);
    tracker->setUseNormal(false);


    //Setup coherence object for tracking
    coherence = boost::shared_ptr<CoherenceType>(new CoherenceType());

    boost::shared_ptr<pcl::tracking::DistanceCoherence<pcl::PointXYZRGB> > distance_coherence
      = boost::shared_ptr<pcl::tracking::DistanceCoherence<pcl::PointXYZRGB> > (new pcl::tracking::DistanceCoherence<pcl::PointXYZRGB> ());
    //coherence->addPointCoherence (distance_coherence);

    boost::shared_ptr<pcl::search::Octree<pcl::PointXYZRGB> > search (new pcl::search::Octree<pcl::PointXYZRGB> (0.01));
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.01);

    tracker->setCloudCoherence (coherence);


    // Set up downsampling filter
    approximateVoxelFilter = boost::shared_ptr<FilterType>(new FilterType());
    approximateVoxelFilter->setLeafSize(20, 20, 20);

}

void TrackerPCL::setReference(PointCloudConstPtr refPointCloud){

    // Downsample reference point cloud
    approximateVoxelFilter->setInputCloud(refPointCloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    approximateVoxelFilter->filter(*filteredPointCloud);

    std::cout << "Reduced number of points in source from " << refPointCloud->points.size() << " to " << filteredPointCloud->points.size() << std::endl;

    // Demean reference point cloud
    Eigen::Vector4f c;
    pcl::compute3DCentroid<pcl::PointXYZRGB>(*filteredPointCloud, c);
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    trans.translation().matrix() = Eigen::Vector3f (c[0], c[1], c[2]);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr demeanPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud<pcl::PointXYZRGB>(*filteredPointCloud, *demeanPointCloud, trans.inverse());

    tracker->setReferenceCloud(demeanPointCloud);
    tracker->setTrans(trans);
}

//void TrackerPCL::setCameraMatrix(Eigen::Matrix3f _cameraMatrix){
//}

void TrackerPCL::determineTransformation(PointCloudConstPtr pointCloud, Eigen::Affine3f &T, bool &converged, float &RMS){

    // Filter
    approximateVoxelFilter->setInputCloud(pointCloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    approximateVoxelFilter->filter(*filteredPointCloud);
    std::cout << "Reduced number of points in source from " << pointCloud->points.size() << " to " << filteredPointCloud->points.size() << std::endl;

    tracker->setInputCloud(filteredPointCloud);
    //coherence->setTargetCloud(pointCloud);
    //tracker->changed_ = true;
    tracker->compute();
//    pcl::tracking::ParticleXYZRPY res = tracker->getResult();

    // Save point cloud of particles
    TrackerType::PointCloudStatePtr particles = tracker->getParticles();
    if (particles){
        //Set pointCloud with particle's points
        pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        particle_cloud->is_dense = true;
        particle_cloud->width = particles->points.size();
        particle_cloud->height = 1;
        for (size_t i = 0; i < particles->points.size(); i++){
            pcl::PointXYZ point;

            point.x = particles->points[i].x;
            point.y = particles->points[i].y;
            point.z = particles->points[i].z;
            particle_cloud->points.push_back(point);
        }
        pcl::io::savePCDFileASCII("particles.pcd", *particle_cloud);
    }

    // Return result as 4x4 matrix
    T = tracker->getResult().toEigenMatrix().matrix();
    converged = true;
}

TrackerPCL::~TrackerPCL(){
    //delete tracker;
    //delete approximateVoxelFilter;
}
