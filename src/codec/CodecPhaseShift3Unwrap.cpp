#include "CodecPhaseShift3Unwrap.h"
#include <math.h>

#include "cvtools.h"
#include "pstools.h"
#include "phaseunwrap.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

static unsigned int nPhases = 4;

// Encoder

EncoderPhaseShift3Unwrap::EncoderPhaseShift3Unwrap(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Encoder(_screenCols, _screenRows, _dir){

    // Set N
    this->N = 3;

    // Precompute encoded patterns
    const float pi = M_PI;

    // Horizontally encoding patterns
    for(unsigned int i=0; i<3; i++){
        float phase = 2.0*pi/3.0 * i;
        float pitch = (float)screenCols/(float)nPhases;
        cv::Mat patternI(1,1,CV_8U);
        patternI = pstools::computePhaseVector(screenCols, phase, pitch);
        patterns.push_back(patternI.t());
    }

}

cv::Mat EncoderPhaseShift3Unwrap::getEncodingPattern(unsigned int depth) const{
    return patterns[depth];
}

// Decoder
DecoderPhaseShift3Unwrap::DecoderPhaseShift3Unwrap(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Decoder(_screenCols, _screenRows){
    this->N = 3;
    frames.resize(N);
}

void DecoderPhaseShift3Unwrap::setFrame(unsigned int depth, cv::Mat frame){
    frames[depth] = frame;
}

void DecoderPhaseShift3Unwrap::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading) const{

    const float pi = M_PI;

    // Calculate multiple phase image
    up = pstools::getPhase(frames[0], frames[1], frames[2]);

    // Calculate modulation
    shading = pstools::getMagnitude(frames[0], frames[1], frames[2]);

    // Create mask from modulation image
    mask = shading > 25;\

    // Unwrap multiple phase image
    cv::Mat quality = phaseunwrap::createqualitymap(up, mask);

    // Blurred quality map
    cv::GaussianBlur(quality, quality, cv::Size(0,0), 3, 3);

    //cvtools::imshow("quality", quality, 0, 0);
//cvtools::writeMat(quality, "quality.mat", "quality");

    std::vector<float> thresholds = phaseunwrap::computethresholds(quality, mask);

//    for(int i=0; i<3; i++)
//        std::cout << thresholds[i] << " ";
//    std::cout << std::endl;
//cvtools::writeMat(up, "up.mat", "up");
//    // Unwrap absolute phase
    phaseunwrap::unwrap(up, quality, mask, thresholds);
//cvtools::writeMat(up, "up.mat", "up");
//cvtools::writeMat(mask, "mask.mat", "mask");

    up += 3.0*2.0*pi;
    up *= screenCols/(2.0*pi*nPhases);

}
