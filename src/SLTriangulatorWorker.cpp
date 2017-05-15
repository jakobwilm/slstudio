#include "SLTriangulatorWorker.h"

#include <QCoreApplication>
#include <QSettings>

#include <iostream>
#include <opencv2/opencv.hpp>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>

void SLTriangulatorWorker::setup(){

    // Initialize triangulator with calibration
    calibration = new CalibrationData;
    calibration->load("calibration.xml");
    triangulator = new Triangulator(*calibration);

    QSettings settings("SLStudio");
    writeToDisk = settings.value("writeToDisk/pointclouds",false).toBool();

}


void SLTriangulatorWorker::triangulatePointCloud(cv::Mat up, cv::Mat vp, cv::Mat mask, cv::Mat shading){

    // Recursively call self until latest event is hit
    busy = true;
    QCoreApplication::sendPostedEvents(this, QEvent::MetaCall);
    bool result = busy;
    busy = false;
    if(!result){
        std::cerr << "SLTriangulatorWorker: dropped phase image!" << std::endl;
        return;
    }

    time.restart();

    // Reconstruct point cloud
    cv::Mat pointCloud;
    triangulator->triangulate(up, vp, mask, shading, pointCloud);

    // Convert point cloud to PCL format
    PointCloudPtr pointCloudPCL(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Interprete as organized point cloud
    pointCloudPCL->width = pointCloud.cols;
    pointCloudPCL->height = pointCloud.rows;
    pointCloudPCL->is_dense = false;

    pointCloudPCL->points.resize(pointCloud.rows*pointCloud.cols);

    for(int row=0; row<pointCloud.rows; row++){
        int offset = row * pointCloudPCL->width;
        for(int col=0; col<pointCloud.cols; col++){
            const cv::Vec3f pnt = pointCloud.at<cv::Vec3f>(row,col);
            unsigned char shade = shading.at<unsigned char>(row,col);
            pcl::PointXYZRGB point;
            point.x = pnt[0]; point.y = pnt[1]; point.z = pnt[2];
            point.r = shade; point.g = shade; point.b = shade;
            pointCloudPCL->points[offset + col] = point;
        }
    }

      std::vector<cv::Mat> xyz;
      cv::split(pointCloud, xyz);
      cv::Mat depth;
      cv::normalize(xyz[2], depth, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      cv::bitwise_not( depth, depth ); //invert
      emit newDepthImage(depth);

//    // stack xyz data
//    std::vector<cv::Mat> pointCloudChannels;
//    pointCloudChannels.push_back(xyz[0]);
//    pointCloudChannels.push_back(xyz[1]);
//    pointCloudChannels.push_back(xyz[2]);

//    // 4 byte padding
//    pointCloudChannels.push_back(cv::Mat::zeros(pointCloud.size(), CV_32F));

//    // triple uchar color information
//    std::vector<cv::Mat> rgb;
//    rgb.push_back(shading);
//    rgb.push_back(shading);
//    rgb.push_back(shading);
//    rgb.push_back(cv::Mat::zeros(shading.size(), CV_8U));

//    cv::Mat rgb8UC4;
//    cv::merge(rgb, rgb8UC4);

//    cv::Mat rgb32F(rgb8UC4.size(), CV_32F, rgb8UC4.data);

//    pointCloudChannels.push_back(rgb32F);

//    // 12 bytes padding
//    pointCloudChannels.push_back(cv::Mat::zeros(pointCloud.size(), CV_32F));
//    pointCloudChannels.push_back(cv::Mat::zeros(pointCloud.size(), CV_32F));
//    pointCloudChannels.push_back(cv::Mat::zeros(pointCloud.size(), CV_32F));

//    // merge channels
//    cv::Mat pointCloudPadded;
//    cv::merge(pointCloudChannels, pointCloudPadded);

//    // memcpy everything
//    memcpy(&pointCloudPCL->points[0], pointCloudPadded.data, pointCloudPadded.rows*pointCloudPadded.cols*sizeof(pcl::PointXYZRGB));

//    // filtering
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filter;
//    filter.setMeanK(5);
//    filter.setStddevMulThresh(1.0);
//    filter.setInputCloud(pointCloudPCL);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudFiltered(new pcl::PointCloud<pcl::PointXYZRGB>);
//    filter.filter(*pointCloudFiltered);

    // Emit result
    emit newPointCloud(pointCloudPCL);

    std::cout << "Triangulator: " << time.elapsed() << "ms" << std::endl;

    if(writeToDisk){
        QString fileName = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmsszzz");
        fileName.append(".pcd");
        pcl::io::savePCDFileBinary(fileName.toStdString(), *pointCloudPCL);
    }

    //emit finished();
}

SLTriangulatorWorker::~SLTriangulatorWorker(){
    delete calibration;
    delete triangulator;

    std::cout<<"triangulatorWorker deleted\n"<<std::flush;
}
