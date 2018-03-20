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
    cv::Mat pointCloud0;
    triangulator->triangulate(up, vp, mask, shading, pointCloud0);

    // convert point cloud to depthImage
    cv::Mat depthImage(pointCloud0.rows, pointCloud0.cols, CV_32F); // convert pointCloud to depth iamge
    for(int row=0; row<pointCloud0.rows; row++){
        for(int col=0; col<pointCloud0.cols; col++){
            const cv::Vec3f pnt = pointCloud0.at<cv::Vec3f>(row,col);
            depthImage.at<float>(row, col) = pnt[2];
        }
    }

    // to better visualize the depth, apply colormap to the computed depth image
    cv::Mat mask2(mask);

    cv::Scalar mean, stddev;
    double mean_1, stddev_1;
    double min, max;

    std::cout << "Remove 3 sigma nosie:" << std::endl;
    int iter_n = 5; // maybe you want to redo this for multiple times
    for(int i=0; i < iter_n; i++)
    {   
        std::cout << "Iteration: " << i << std::endl;
        cv::meanStdDev(depthImage, mean, stddev, mask=mask2);
        mean_1 = mean[0];
        stddev_1 = stddev[0];
        std::cout << "  depthImage mean=" << mean_1 << ", stddev=" << stddev_1 << std::endl;

        double limit_h = mean_1 + 3 * stddev_1;
        double limit_l = mean_1 - 3 * stddev_1;

        cv::bitwise_and(cv::Mat(depthImage > limit_l), mask2, mask2);
        cv::bitwise_and(cv::Mat(depthImage < limit_h), mask2, mask2);
        
        cv::minMaxIdx(depthImage, &min, &max, NULL, NULL, mask2);
        std::cout << "  depthImage min=" << min << ", max=" << max << std::endl;
    }
    // cv::minMaxIdx(depthImage, &min, &max, NULL, NULL, mask2);
    // std::cout << "  depthImage min=" << min << ", max=" << max << std::endl;

    cv::Mat cmDepth, scaledDepth, depthView;
    convertScaleAbs( depthImage, scaledDepth, 255. / ( max - min ),  - 255.*min/(max - min));
    applyColorMap( scaledDepth, cmDepth, cv::COLORMAP_JET );

    cmDepth.copyTo(depthView, mask2);

    // get a new point cloud so that the noise point are masked.
    cv::Mat pointCloud = cv::Mat(up.size(), CV_32FC3, cv::Scalar(NAN, NAN, NAN));
    pointCloud0.copyTo(pointCloud, mask2);

    // Convert point cloud to PCL format
    std::cout << "Convert point clout to PCL format." << std::endl;
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

//    std::vector<cv::Mat> xyz;
//    cv::split(pointCloud, xyz);

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

    // filtering
    std::cout << "Start statistical outlier removal." << std::endl;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filter;
    filter.setMeanK(10);
    filter.setStddevMulThresh(1.0);
    filter.setInputCloud(pointCloudPCL);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudFiltered(new pcl::PointCloud<pcl::PointXYZRGB>);
    filter.filter(*pointCloudFiltered);

    // Emit result
    emit newDepthImage(depthView);
    emit newPointCloud(pointCloudFiltered);
    // emit newPointCloud(pointCloudPCL);

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
