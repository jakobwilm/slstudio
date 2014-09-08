#include "TrackerICPFast.h"

TrackerICPFast::TrackerICPFast(){

    lastTransformation = Eigen::Matrix4f::Identity();

}

void TrackerICPFast::setReference(PointCloudConstPtr _refPointCloud){

    refPointCloud = _refPointCloud;

    // Determine boundary of target point cloud (8 neighborhood number of NANS or edges > 4)
    boundary.create(refPointCloud->height, refPointCloud->height, cv::DataType<bool>::type);
    boundary.setTo(false);

    const unsigned int windowSize = 2;
    const unsigned int boundaryThreshold = 4;
    const float depthThreshold = 5.0;
    for(unsigned int i=0; i<refPointCloud->width; i++){
        for(unsigned int j=0; j<refPointCloud->height; j++){
            unsigned int nanCount = 0;
            for (unsigned int x_d = -windowSize/2; x_d <= windowSize/2; ++x_d){
                for (unsigned int y_d = -windowSize/2; y_d <= windowSize/2; ++y_d){
                    if (i + x_d >= 0 && i + x_d < refPointCloud->width &&
                        j + y_d >= 0 && j + y_d < refPointCloud->height){

                        bool nan = !pcl::isFinite(refPointCloud->at(i + x_d, j + y_d));
                        bool step = fabs(refPointCloud->at(i, j).z - refPointCloud->at(i + x_d, j + y_d).z) > depthThreshold;
                        if (nan || step)
                            nanCount++;
                    }
                }
            }
            if(nanCount > boundaryThreshold)
                boundary.at<bool>(i,j) = true;
        }
    }

}

void TrackerICPFast::determineTransformation(PointCloudConstPtr pointCloud, Eigen::Matrix4f &T, bool &converged){

    const unsigned int maxIterations = 50;

    for(unsigned int k=0; k<maxIterations; k++){




    }

}

TrackerICPFast::~TrackerICPFast(){

}
