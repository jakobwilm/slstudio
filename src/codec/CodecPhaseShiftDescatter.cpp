#include "CodecPhaseShiftDescatter.h"
#include <math.h>

#include "cvtools.h"
#include "pstools.h"


#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

static unsigned int nPhases = 16;

// Encoder


EncoderPhaseShiftDescatter::EncoderPhaseShiftDescatter(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Encoder(_screenCols, _screenRows, _dir){

    // Set N
    N = 6;

    // Precompute encoded patterns
    const float pi = M_PI;

    // Horizontally encoding patterns
    for(unsigned int i=0; i<3; i++){
        float phase = 2.0*pi/3.0 * i;
        float pitch = (float)screenCols/(float)nPhases;
        cv::Mat patternI(screenRows,screenCols,CV_8UC3);

        for(unsigned int r=0; r<screenRows; r++){
            for(unsigned int c=0; c<screenCols; c++){

                float amp = 0.5 + 0.5*cos(2*pi*c/pitch - phase)*cos(2*pi*r/200 - phase);

                patternI.at<cv::Vec3b>(r, c) = cv::Vec3b(255.0*amp,255.0*amp,255.0*amp);
            }

        }

        patternI = patternI;
        patterns.push_back(patternI);
    }

    // Phase cue patterns
    for(unsigned int i=0; i<3; i++){
        float phase = 2.0*pi/3.0 * i;
        float pitch = screenCols;
        cv::Mat patternI;
        patternI = pstools::computePhaseVector(screenCols, phase, pitch);
        patternI = patternI.t();
        patterns.push_back(patternI);
    }

}

cv::Mat EncoderPhaseShiftDescatter::getEncodingPattern(unsigned int depth){
    return patterns[depth];
}

// Decoder
DecoderPhaseShiftDescatter::DecoderPhaseShiftDescatter(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Decoder(_screenCols, _screenRows, _dir){

    N = 6;

    frames.resize(N);
}

void DecoderPhaseShiftDescatter::setFrame(unsigned int depth, cv::Mat frame){
    frames[depth] = frame;
}

static cv::Mat unwrap(const cv::Mat up, const cv::Mat upCue, unsigned int nPhases){

    const float pi = M_PI;

    // Determine number of jumps
    cv::Mat P = (upCue*nPhases-up)/(2*pi);

    // Round to integers
    P.convertTo(P, CV_8U);
    P.convertTo(P, CV_32F);

    // Add to phase
    cv::Mat upUnwrapped = up + P*2*pi;

    // Scale to range [0; 2pi]
    upUnwrapped *= 1.0/nPhases;

    return upUnwrapped;
}

void DecoderPhaseShiftDescatter::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading){

    const float pi = M_PI;

    std::vector<cv::Mat> framesHorz(frames.begin(), frames.begin()+6);

    // Horizontal decoding

    up = pstools::getPhase(framesHorz[0], framesHorz[1], framesHorz[2]);




    cv::Mat upCue = pstools::getPhase(framesHorz[3], framesHorz[4], framesHorz[5]);
    up = unwrap(up, upCue, nPhases);
    up *= screenCols/(2*pi);

    //cv::GaussianBlur(up, up, cv::Size(0,0), 3, 3);

    // Calculate modulation
    shading = pstools::getMagnitude(frames[0], frames[1], frames[2]);
//cvtools::writeMat(shading, "shading.mat");
    // Threshold modulation image for mask
    mask = shading > 25;
//cvtools::writeMat(mask, "mask.mat");
//    cv::Mat edges;
//    cv::Sobel(up, edges, -1, 1, 1, 7);
//    edges = abs(edges) < 200;

//    cv::Mat strel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
//    cv::dilate(edges, edges, strel);
//    strel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6,6));
//    cv::erode(edges, edges, cv::Mat());

    cv::Mat dx, dy;
    cv::Sobel(up, dx, -1, 1, 0, 3);
    cv::Sobel(up, dy, -1, 0, 1, 3);
    cv::Mat edges;
    cv::magnitude(dx, dy, edges);
//cvtools::writeMat(edges, "edges.mat", "edges");
    mask = mask & (edges < 200);
//cvtools::writeMat(mask, "mask.mat");

}
