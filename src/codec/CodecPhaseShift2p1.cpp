#include "CodecPhaseShift2p1.h"
#include <math.h>

#include "pstools.h"
#include "cvtools.h"

#include <opencv2/imgproc/imgproc_c.h>

//#include <opencv2/video/tracking.hpp>

#ifndef M_PI
    #define M_PI 3.14159265359
#endif

// Encoder
EncoderPhaseShift2p1::EncoderPhaseShift2p1(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Encoder(_screenCols, _screenRows, _dir){

    // Set N
    N = 3;

    // Precompute encoded patterns
    const float pi = M_PI;

    // 0 deg
    cv::Mat patternI = pstools::computePhaseVector(screenCols, 0.0, screenCols);
    patterns.push_back(patternI.t());

    // 90 deg
    patternI = pstools::computePhaseVector(screenCols, pi/2.0, screenCols);
    patterns.push_back(patternI.t());

    // Flat image
    patternI.create(5, 5, CV_8UC3);
    patternI.setTo(0.5*255.0);
    patterns.push_back(patternI);
}

cv::Mat EncoderPhaseShift2p1::getEncodingPattern(unsigned int depth){
    return patterns[depth];
}

// Decoder
DecoderPhaseShift2p1::DecoderPhaseShift2p1(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Decoder(_screenCols, _screenRows){
    N = 3;
    frames.resize(N);

    lastShading = new cv::Mat_<float>();

}

void DecoderPhaseShift2p1::setFrame(unsigned int depth, cv::Mat frame){
    frames[depth] = frame;
}

void DecoderPhaseShift2p1::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading){

    const float pi = M_PI;

    cv::Mat_<float> I1(frames[0]);
    cv::Mat_<float> I2(frames[1]);
    cv::Mat_<float> I3(frames[2]);


//    cvtools::writeMat(I1, "I1.mat", "I1");
//    cvtools::writeMat(I2, "I2.mat", "I2");
//    cvtools::writeMat(I3, "I3.mat", "I3");

    if(lastShading->empty())
        *lastShading = I3;

//cvtools::writeMat(I1small, "I1small.mat", "I1small");
//cvtools::writeMat(*lastShading, "lastShading.mat", "lastShading");
//    cv::Point2d shift12 = cv::phaseCorrelate(I1, I2);
//    cv::Point2d shift23 = cv::phaseCorrelate(I2, I3);
//    cv::Point2d shift = 0.5*(shift12+shift23);

    // Hanning window used in phase correlations
    cv::Mat window;
    cv::createHanningWindow(window, lastShading->size(), CV_64F);

    // determine scale and rotation
    cv::Mat lastShading_logPolar(lastShading->size(), CV_32F);
    cv::Mat I3_logPolar(I3.size(), CV_32F);
    IplImage lastShading_ipl = *lastShading;
    IplImage I3_ipl = I3;
    IplImage lastShadingLogPolar_ipl = lastShading_logPolar;
    IplImage I3logPolar_ipl = I3_logPolar;
    cvLogPolar(&lastShading_ipl, &lastShadingLogPolar_ipl, CvPoint2D32f(lastShading_ipl.width/2, lastShading_ipl.height/2), 40.0, CV_INTER_LINEAR);
    cvLogPolar(&I3_ipl, &I3logPolar_ipl, CvPoint2D32f(I3_ipl.width/2, I3_ipl.height/2), 40.0, CV_INTER_LINEAR);

    cv::Point2d scaleRotation = cv::phaseCorrelate(lastShading_logPolar, I3_logPolar, window);
cv::imwrite("I3_logPolar.png", I3_logPolar);
    std::cout << "scaleRotation=" << scaleRotation << std::endl;

    cv::Point2d shift = cv::phaseCorrelate(*lastShading, I3, window);

    shiftHistory.push_back(shift);

    *lastShading = I3;

    // Shift input images according to global shift
    cv::Point2f center3(I3.cols/2.0, I3.rows/2.0);
    cv::Point2f center1(center3.x-0.667*shift.x, center3.y-0.667*shift.y);
    cv::Point2f center2(center3.x-0.333*shift.x, center3.y-0.333*shift.y);

    // Cannot process in-place when shift is positive
    cv::Mat_<float> I1Copy = I1.clone();
    cv::Mat_<float> I2Copy = I2.clone();

    cv::getRectSubPix(I1Copy, I1.size(), center1, I1);
    cv::getRectSubPix(I2Copy, I2.size(), center2, I2);

//    std::cout << "center3" << std::endl << center3 << std::endl;
//    std::cout << "center1" << std::endl << center1 << std::endl;
//    std::cout << "center2" << std::endl << center2 << std::endl;

    cv::phase(I1-I3, I2-I3, up);
    up *= screenCols/(2*pi);

//            cvtools::writeMat(I1, "I1c.mat", "I1c");
//            cvtools::writeMat(I2, "I2c.mat", "I2c");
//            cvtools::writeMat(I3, "I3c.mat", "I3c");

        //    cvtools::writeMat(up, "up.mat");

        //    cv::Mat upCopy = up.clone();
        //    cv::bilateralFilter(upCopy, up, 7, 500, 400);
    cv::GaussianBlur(up, up, cv::Size(0,0), 3, 3);

    cv::Mat mag;
    cv::magnitude(I1-I3, I2-I3, mag);

    shading = 2.0*frames[2];

        //    cvtools::writeMat(shading, "shading.mat");

            // Create mask from modulation image and erode
    mask.create(shading.size(), cv::DataType<bool>::type);
    mask.setTo(true);
    mask = (shading > 50) & (shading < 254);

        //    cv::Mat flow;
        //    cv::calcOpticalFlowSF(I1, I2, flow, 1, 3, 1);
        //    cv::Ptr<cv::DenseOpticalFlow> tvl1flow = cv::createOptFlow_DualTVL1();
        //    tvl1flow->calc(I1, I2, flow);
        //    cvtools::writeMat(flow, "flow.mat", "flow");


    std::cout << shift << std::endl;
    cv::Size frameSize = lastShading->size();
    cv::Point2d center(frameSize.width/2, frameSize.height/2);
    cv::line(shading, center, center+30*shift, cv::Scalar(255), 5);

    cv::Mat dx, dy;
    cv::Sobel(I3, dx, -1, 1, 0, 3);
    cv::Sobel(I3, dy, -1, 0, 1, 3);
    cv::Mat edgesShading = abs(dx) + abs(dy);

    cv::Sobel(up, dx, -1, 1, 0, 3);
    cv::Sobel(up, dy, -1, 0, 1, 3);
    cv::Mat edgesUp = abs(dx) + abs(dy);

    cv::Sobel(up, dx, -1, 1, 0, 3);
    cv::Sobel(up, dy, -1, 0, 1, 3);

    mask = mask & (edgesShading < 100) & (edgesUp < 130);

}

DecoderPhaseShift2p1::~DecoderPhaseShift2p1(){

    cv::FileStorage fs("shiftHistory.xml", cv::FileStorage::WRITE);
    if (!fs.isOpened())
        std::cerr << "Could not write shiftHistory.xml" << std::endl;

    fs << "shiftHistory" << cv::Mat(shiftHistory);
    fs.release();
}
