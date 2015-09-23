#include "CodecPhaseShiftMicro.h"
#include <math.h>

#include "cvtools.h"
#include "pstools.h"


#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

static const unsigned int nFrequencies = 3;
static const float frequencies[] = {99, 100, 101};

// Encoder
EncoderPhaseShiftMicro::EncoderPhaseShiftMicro(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Encoder(_screenCols, _screenRows, _dir){

    // Set N
    N = nFrequencies+2;

    // Precompute encoded patterns
    const float pi = M_PI;

    // Main frequency encoding patterns
    for(unsigned int i=0; i<3; i++){
        float phase = 2.0*pi/3.0 * i;
        float pitch = frequencies[0];
        cv::Mat patternI(1,1,CV_8U);
        patternI = pstools::computePhaseVector(screenCols, phase, pitch);
        patternI = patternI.t();
        patterns.push_back(patternI);
    }

    // Additional frequency patterns
    for(unsigned int i=1; i<nFrequencies; i++){
        float phase = 0.0;
        float pitch = frequencies[i];
        cv::Mat patternI;
        patternI = pstools::computePhaseVector(screenCols, phase, pitch);
        patternI = patternI.t();
        patterns.push_back(patternI);
    }

}

cv::Mat EncoderPhaseShiftMicro::getEncodingPattern(unsigned int depth){
    return patterns[depth];
}

// Decoder
DecoderPhaseShiftMicro::DecoderPhaseShiftMicro(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Decoder(_screenCols, _screenRows, _dir){

    N = nFrequencies+2;

    frames.resize(N);
}

void DecoderPhaseShiftMicro::setFrame(unsigned int depth, cv::Mat frame){
    frames[depth] = frame;
}

void DecoderPhaseShiftMicro::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading){

    const float pi = M_PI;

    std::vector<cv::Mat> framesHorz(frames.begin(), frames.begin()+6);

    // Horizontal decoding
    up = pstools::getPhase(framesHorz[0], framesHorz[1], framesHorz[2]);
    shading = pstools::getMagnitude(frames[0], frames[1], frames[2]);

    mask = shading > 25;

    // NOTE: The rest is not implemented yet as it requires the rather intricate unwrapping technique of micro psi (see paper for details).

}
