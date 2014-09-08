#include "CodecPhaseShiftNStep.h"
#include <math.h>

#include "cvtools.h"
#include "pstools.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

static unsigned int nPhases = 20;
static unsigned int nSteps = 20;

// Encoder
EncoderPhaseShiftNStep::EncoderPhaseShiftNStep(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Encoder(_screenCols, _screenRows, _dir){

    // Set N
    N = nSteps+3;
    if(dir == CodecDirBoth)
        N *= 2;

    // Precompute encoded patterns
    const float pi = M_PI;

    if(dir & CodecDirHorizontal){
        // Horizontally encoding patterns
        for(unsigned int i=0; i<nSteps; i++){
            float phase = 2.0*pi/nSteps * i;
            float pitch = (float)screenCols/(float)nPhases;
            cv::Mat patternI(1,1,CV_8U);
            patternI = pstools::computePhaseVector(screenCols, phase, pitch);
            patterns.push_back(patternI.t());
        }

        // Phase cue patterns
        for(unsigned int i=0; i<3; i++){
            float phase = 2.0*pi/3.0 * i;
            float pitch = screenCols;
            cv::Mat patternI;
            patternI = pstools::computePhaseVector(screenCols, phase, pitch);
            patterns.push_back(patternI.t());
        }
    }
    if(dir & CodecDirVertical){
        // Precompute vertically encoding patterns
        for(unsigned int i=0; i<nSteps; i++){
            float phase = 2.0*pi/nSteps * i;
            float pitch = (float)screenRows/(float)nPhases;
            cv::Mat patternI;
            patternI = pstools::computePhaseVector(screenRows, phase, pitch);
            patterns.push_back(patternI);
        }

        // Precompute vertically phase cue patterns
        for(unsigned int i=0; i<3; i++){
            float phase = 2.0*pi/3.0 * i;
            float pitch = screenRows;
            cv::Mat patternI;
            patternI = pstools::computePhaseVector(screenRows, phase, pitch);
            patterns.push_back(patternI);
        }
    }
}

cv::Mat EncoderPhaseShiftNStep::getEncodingPattern(unsigned int depth){
    return patterns[depth];
}

// Decoder
DecoderPhaseShiftNStep::DecoderPhaseShiftNStep(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Decoder(_screenCols, _screenRows, _dir){

    // Set N
    N = nSteps+3;
    if(dir == CodecDirBoth)
        N *= 2;

    frames.resize(N);
}

void DecoderPhaseShiftNStep::setFrame(unsigned int depth, cv::Mat frame){
    frames[depth] = frame;
}

void DecoderPhaseShiftNStep::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading){

    const float pi = M_PI;

    std::vector<cv::Mat> fIcomp;

    if(dir & CodecDirHorizontal){
        std::vector<cv::Mat> framesHorz(frames.begin(), frames.begin()+nSteps);
        std::vector<cv::Mat> framesHorzCue(frames.begin()+nSteps, frames.begin()+nSteps+3);

        // Horizontal decoding
        fIcomp = pstools::getDFTComponents(framesHorz);
        cv::phase(fIcomp[2], -fIcomp[3], up);
        cv::Mat upCue = pstools::getPhase(framesHorzCue[0], framesHorzCue[1], framesHorzCue[2]);
        up = pstools::unwrapWithCue(up, upCue, nPhases);
        up *= screenCols/(2*pi);

        cv::GaussianBlur(up, up, cv::Size(0,0), 1, 1);

    }
    if(dir & CodecDirVertical){
        std::vector<cv::Mat> framesVert(frames.end()-nSteps-3, frames.end()-3);
        std::vector<cv::Mat> framesVertCue(frames.end()-3, frames.end());

        // Vertical decoding
        fIcomp = pstools::getDFTComponents(framesVert);
        cv::phase(fIcomp[2], -fIcomp[3], vp);
        cv::Mat vpCue = pstools::getPhase(framesVertCue[0], framesVertCue[1], framesVertCue[2]);
        vp = pstools::unwrapWithCue(vp, vpCue, nPhases);
        vp *= screenCols/(2*pi);
    }

    // Calculate modulation
    cv::Mat X0, X1, X2;
    cv::magnitude(fIcomp[0], fIcomp[1], X0);
    cv::magnitude(fIcomp[2], fIcomp[3], X1);
    cv::magnitude(fIcomp[4], fIcomp[5], X2);

    shading = 4.0/nSteps * X1;
    shading.convertTo(shading, CV_8U);

    // Threshold on energies
    mask = (X0 > nSteps*2048) & (X1 > nSteps*1024) & (X2 < nSteps*2048);

    // Threshold on gradient of phase
    cv::Mat edges;
    cv::Sobel(up, edges, -1, 1, 1, 7);

    edges = abs(edges) < 200;
    cv::Mat strel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
    cv::erode(edges, edges, strel);

    mask = mask & edges;

}
