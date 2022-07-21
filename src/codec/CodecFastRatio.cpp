#include "CodecFastRatio.h"
#include <math.h>

#include "pstools.h"
#include "cvtools.h"

#ifndef M_PI
    #define M_PI 3.14159265359
#endif

// Encoder
EncoderFastRatio::EncoderFastRatio(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Encoder(_screenCols, _screenRows, _dir){

    // Set N
    N = 3;

    // Low
    cv::Mat patternI(1, 1, CV_8UC3);
    patternI.setTo(0.5*255);
    patterns.push_back(patternI.clone());

    // High
    patternI.setTo(255);
    patterns.push_back(patternI);

    // Linear Phase
    patternI = cv::Mat(1, screenCols, CV_8UC3);
    for(unsigned int i=0; i<screenCols; i++){
        // Amplitude of channels
        float amp = i*(255.0/(float)(screenCols-1));
        patternI.at<cv::Vec3b>(0, i) = cv::Vec3b(amp, amp, amp);
    }
    patterns.push_back(patternI);

}

cv::Mat EncoderFastRatio::getEncodingPattern(unsigned int depth) const{
    return patterns[depth];
}

// Decoder
DecoderFastRatio::DecoderFastRatio(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Decoder(_screenCols, _screenRows){
    N = 3;
    frames.resize(N);
}

void DecoderFastRatio::setFrame(unsigned int depth, cv::Mat frame){
    frames[depth] = frame;
}

void DecoderFastRatio::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading) const{


    cv::Mat_<float> I1(frames[0]);
    cv::Mat_<float> I2(frames[1]);
    cv::Mat_<float> I3(frames[2]);

//        cvtools::writeMat(I1, "I1.mat");
//        cvtools::writeMat(I2, "I2.mat");
//        cvtools::writeMat(I3, "I3.mat");

    up = (I3-I1)/(I2-I1);
    up = (up+1.0)/2.0;

//    cvtools::writeMat(frames[0], "frames[0].mat");
//    cvtools::writeMat(frames[1], "frames[1].mat");
//    cvtools::writeMat(frames[2], "frames[2].mat");

    up *= screenCols;

//    cv::Mat upCopy = up.clone();
//    cv::bilateralFilter(upCopy, up, 7, 500, 400);
    cv::GaussianBlur(up, up, cv::Size(0,0), 3, 3);

    shading = frames[1];

    // Create mask from modulation image and erode
    mask.create(shading.size(), cv::DataType<bool>::type);
//    mask.setTo(true);
    mask = (shading > 10000) & (shading < 65000) & (up <= screenCols) & (up >= 0);

//    cv::Mat edges;
//    cv::Sobel(up, edges, -1, 1, 1, 7);
//    edges = abs(edges) < 500;
//    cv::erode(edges, edges, cv::Mat());

//    mask = mask & edges;

}
