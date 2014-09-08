#include "CodecPhaseShift3DirectLight.h"
#include <math.h>

#include "cvtools.h"

#ifndef M_PI
    #define M_PI 3.14159265359
#endif

// Encoder
EncoderPhaseShift3DirectLight::EncoderPhaseShift3DirectLight(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Encoder(_screenCols, _screenRows, _dir){

    // Set N
    this->N = 3;

    // Precompute encoded patterns
    const float pi = M_PI;
    for(unsigned int i=0; i<N; i++){
        float phase = 2.0*pi/float(N) * (1.0 - (float)i);
        float pitch = screenCols;
        cv::Mat patternI(1, screenCols, CV_8UC3);
        patternI.setTo(0);
        for(int i=20; i<patternI.cols-20; i++){
            float amp = 0.5*(1+cos(2*pi*i/pitch + phase));
            patternI.at<cv::Vec3b>(0, i) = cv::Vec3b(255.0*amp,255.0*amp,255.0*amp);
        }
        patterns.push_back(patternI);
    }
}

cv::Mat EncoderPhaseShift3DirectLight::getEncodingPattern(unsigned int depth){
    return patterns[depth];
}

// Decoder
DecoderPhaseShift3DirectLight::DecoderPhaseShift3DirectLight(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Decoder(_screenCols, _screenRows){
    this->N = 3;
    frames.resize(N);
}

void DecoderPhaseShift3DirectLight::setFrame(unsigned int depth, cv::Mat frame){
    frames[depth] = frame;
}

void DecoderPhaseShift3DirectLight::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading){

    cv::Mat_<float> I1(frames[0]);
    cv::Mat_<float> I2(frames[1]);
    cv::Mat_<float> I3(frames[2]);

    const float pi = M_PI;

    // One function call approach
    cv::phase(2.0*I2-I1-I3, sqrt(3.0)*(I3-I1), up);
    up *= screenCols/(2.0*pi);

    cv::Mat modulation(shading.size(), CV_32F);
    modulation = 3.0*(I2-I3).mul(I2-I3) + (2.0*I1-I2-I3).mul(2.0*I1-I2-I3);
    cv::sqrt(modulation, modulation);
    modulation = 1.0/3.0 * modulation;
    modulation.convertTo(shading, CV_8U);

    // Create mask from modulation image and erode
    mask.create(modulation.size(), cv::DataType<bool>::type);
    mask = shading > 100;

//    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7));
//    cv::erode(mask, mask, element);

}
