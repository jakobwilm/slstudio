#include "CodecPhaseShiftModulated.h"
#include <math.h>

#include "cvtools.h"
#include "pstools.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

static unsigned int nPhases = 96;
static unsigned int Nx = 4;
static unsigned int Ny = 4;
static unsigned int Ncue = 8;
// Encoder


EncoderPhaseShiftModulated::EncoderPhaseShiftModulated(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Encoder(_screenCols, _screenRows, _dir){

    // Set N
    N = Nx*Ny+Ncue;

    // Precompute encoded patterns
    const float pi = M_PI;


    // Horizontally encoding patterns
    for(unsigned int x=0; x<Nx; x++){

        float phaseX = 2*pi/Nx * x;
        float pitch = (float)screenCols/(float)nPhases;

        for(unsigned int y=0; y<Ny; y++){

            float phaseY = 2*pi/Ny * y;

            cv::Mat patternI(screenRows,screenCols,CV_8UC3);

            for(int r=0; r<screenRows; r++){
                for(int c=0; c<screenCols; c++){

                    float amp = 0.5 + 0.5*(0.5 + 0.5*cos(2*pi*c/pitch - phaseX))*cos(2*pi*r/pitch - phaseY);

                    patternI.at<cv::Vec3b>(r, c) = cv::Vec3b(255.0*amp,255.0*amp,255.0*amp);
                }

            }

            patterns.push_back(patternI);
        }

    }

    // Phase cue patterns
    for(unsigned int i=0; i<Ncue; i++){
        float phase = 2.0*pi/Ncue * i;
        float pitch = screenCols;
        cv::Mat patternI;
        patternI = pstools::computePhaseVector(screenCols, phase, pitch);
        patternI = patternI.t();
        patterns.push_back(patternI);
    }

}

cv::Mat EncoderPhaseShiftModulated::getEncodingPattern(unsigned int depth){
    return patterns[depth];
}

// Decoder
DecoderPhaseShiftModulated::DecoderPhaseShiftModulated(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Decoder(_screenCols, _screenRows, _dir){

    N = Nx*Ny+Ncue;

    frames.resize(N);
}

void DecoderPhaseShiftModulated::setFrame(unsigned int depth, cv::Mat frame){
    frames[depth] = frame;
}

void DecoderPhaseShiftModulated::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading){

    const float pi = M_PI;

    std::vector<cv::Mat> framesX;

    // Decoding
    for(int x=0; x<Nx; x++){

        std::vector<cv::Mat> framesY(frames.begin()+x*Ny, frames.begin()+x*Ny+Ny);
        std::vector<cv::Mat> fIcomp = pstools::getDFTComponents(framesY);

        cv::Mat frameX;
        cv::magnitude(fIcomp[2], fIcomp[3], frameX);
        framesX.push_back(frameX);
    }

//    cv::Mat upX0 = pstools::getMagnitude(frames[0], frames[1], frames[2]);
//    cv::Mat upX1 = pstools::getMagnitude(frames[3], frames[4], frames[5]);
//    cv::Mat upX2 = pstools::getMagnitude(frames[6], frames[7], frames[8]);
////cvtools::writeMat(upX0, "upX0.mat", "upX0");
////cvtools::writeMat(upX1, "upX1.mat", "upX1");
////cvtools::writeMat(upX2, "upX2.mat", "upX2");
//    up = pstools::getPhase(upX0, upX1, upX2);
    std::vector<cv::Mat> fIcomp = pstools::getDFTComponents(framesX);
    cv::phase(fIcomp[2], -fIcomp[3], up);

    std::vector<cv::Mat> framesCue(frames.end()-Ncue, frames.end());
    fIcomp = pstools::getDFTComponents(framesCue);
    cv::Mat upCue;
    cv::phase(fIcomp[2], -fIcomp[3], upCue);

    up = pstools::unwrapWithCue(up, upCue, nPhases);
    up *= screenCols/(2*pi);

    //cv::GaussianBlur(up, up, cv::Size(0,0), 3, 3);

    // Calculate modulation
    cv::magnitude(fIcomp[2], fIcomp[3], shading);
    shading.convertTo(shading, CV_8U);
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
