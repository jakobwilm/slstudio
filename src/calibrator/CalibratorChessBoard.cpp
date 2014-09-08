#include "Calibrator.h"

Calibrator::Calibrator(unsigned int _screenResX, unsigned int _screenResY, bool _diamondPattern): screenResX(_screenResX), screenResY(_screenResY), diamondPattern(_diamondPattern){

    // Precompute projection image
    projectionImage = cv::Mat(screenResY, screenResX, CV_8UC3);

    // Number of checker corners
    unsigned int nx = 9, ny = 9;

    // Width of checker field in native units
    //  *For diamond array the native unit is a mirror diagonal
    //  *For rectangular arrays, the native unit is a pixel width
    float wx, wy;
    if(diamondPattern){
        wx = ((float)screenResX-0.5+0.001) / nx;
        wy = ((float)screenResY/2.0-0.5+0.001) / ny;
    } else {
        wx = ((float)screenResX-1.0) / nx;
        wy = ((float)screenResY-1.0) / ny;
    }

    // Construct checker board
    for(unsigned int i=0; i<screenResY; i++){
        for(unsigned int j=0; j<screenResX; j++){
            // Determine (up,vp) coordinates
            float up = j, vp = i;
            if(diamondPattern){
                up += ((i+1)%2)*0.5;
                vp /= 2;
            }
            // Determine if light or dark field
            int fieldX = up/wx;
            int fieldY = vp/wy;
            if((fieldX%2)^(fieldY%2))
                projectionImage.at<cv::Vec3b>(i,j) = cv::Vec3i(255,255,255);
            else
                projectionImage.at<cv::Vec3b>(i,j) = cv::Vec3i(0,0,0);

        }
    }

    // Determine (up, vp) coordinates of corners
    for(unsigned int j=0; j<ny; j++){
        for(unsigned int i=0; i<nx; i++){
            // Determine (up,vp) coordinates
            float up = i*wx, vp = j*wy;
            if(diamondPattern){
                up += ((j+1)%2)*0.5;
                vp /= 2;
            }
            // Save in feature point coordinate list
            cv::Vec2f qpi(up, vp);
            qp.push_back(qpi);
        }
    }
}

CalibrationData Calibrator::calibrate(){


}
