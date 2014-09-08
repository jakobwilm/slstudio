#include <iostream>
#include <unistd.h>

#include "TrackerICP.h"
#include "TrackerNDT.h"
#include "TrackerICPFast.h"
#include "TrackerPCL.h"

#include <pcl/io/pcd_io.h>

#include <QTime>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

int main(){

#if 1
    // Construct artificial point clouds
    PointCloudPtr source(new pcl::PointCloud<pcl::PointXYZRGB>(480,640));
    for(unsigned int i=0; i<480; i++){
        for(unsigned int j=0; j<640; j++){
            float z = 200.0*sin(i/120.0)*cos(j/120.0) + 1000.0;
            source->at(i,j).x = i*z/1000.0;
            source->at(i,j).y = j*z/1000.0;
            source->at(i,j).z = z;
            source->at(i,j).r = source->at(i,j).g = source->at(i,j).b = 100;
        }
    }

    // Construct known transformation
    PointCloudPtr target(new pcl::PointCloud<pcl::PointXYZRGB>(*source));
    Eigen::Affine3f T = Eigen::Affine3f::Identity();
    T.translate(Eigen::Vector3f(0.2,-1.1,8.0));
    T.rotate(Eigen::AngleAxisf(0.02*M_PI, Eigen::Vector3f::UnitX()));
    T.rotate(Eigen::AngleAxisf(0.02*M_PI, Eigen::Vector3f::UnitY()));
    T.rotate(Eigen::AngleAxisf(0.02*M_PI, Eigen::Vector3f::UnitZ()));
    // Apply to source
    pcl::transformPointCloud(*source, *target, T);

    // Add independent noise to point clouds
    boost::normal_distribution<> nd(0.0, 1);
    boost::mt19937 rng;
    boost::variate_generator< boost::mt19937&, boost::normal_distribution<> > noise(rng, nd);
    for (size_t i = 0; i < source->points.size(); i++){
        //source->points[i].x += noise();
        //source->points[i].y += noise();
        source->points[i].z += noise();

        //target->points[i].x += noise();
        //target->points[i].y += noise();
        target->points[i].z += noise();
    }

    std::cout << "T: " << std::endl << T.matrix() << std::endl;

    // Make partial overlap
    source->is_dense = false;
    target->is_dense = false;
    for(unsigned int i=0; i<480; i++){
        for(unsigned int j=0; j<50; j++){
            source->at(i,j).x = NAN;
            source->at(i,j).y = NAN;
            source->at(i,j).z = NAN;
            target->at(i,639-j).x = NAN;
            target->at(i,639-j).y = NAN;
            target->at(i,639-j).z = NAN;
        }
    }

    pcl::io::savePCDFileBinary("source.pcd", *source);
    pcl::io::savePCDFileBinary("target.pcd", *target);

    Eigen::Matrix3f Kc = Eigen::Matrix3f::Identity();
    Kc(0,0) = 1000;
    Kc(1,1) = 1000;

#else

    PointCloudPtr source(new pcl::PointCloud<pcl::PointXYZRGB>());
    PointCloudPtr target(new pcl::PointCloud<pcl::PointXYZRGB>());

    // Load real sensor data
    pcl::io::loadPCDFile("/home/jakw/Code/Repos/SLStudio/build/test1.pcd", *source);
    pcl::io::loadPCDFile("test2.pcd", *target);

    // Load real camera matrix
    Eigen::Matrix3f Kc = Eigen::Matrix3f::Identity();
    Kc << 1.44241394e+03, 0., 3.30343628e+02, 0., 1.44099609e+03, 2.99075531e+02, 0., 0., 1.;

#endif

    // Set up tracker
    TrackerPCL tracker = TrackerPCL();
    tracker.setReference(target);

    tracker.setCameraMatrix(Kc);

    Eigen::Matrix4f Test;
    bool converged = false;

    QTime time;
    time.start();

    tracker.determineTransformation(source, Test, converged);

    std::cout << "Time: " << time.elapsed() << " ms" << std::endl;
    std::cout << "Test: " << std::endl << Test.matrix() << std::endl;

    PointCloudPtr result(new pcl::PointCloud<pcl::PointXYZRGB>(480,640));
    pcl::transformPointCloud(*source, *result, Test);

    // Save resulting point cloud
    pcl::io::savePCDFileBinary("result.pcd", *result);

}
