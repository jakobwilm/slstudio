#include "CodecPhaseShiftModulated.h"
#include <math.h>

#include "cvtools.h"
#include "pstools.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

static unsigned int nPhaseX = 24;
static unsigned int nPhaseY = 16;
static unsigned int Nx = 3;
static unsigned int Ny = 3;
static unsigned int Ncue = 3;
// Encoder

#define USE_SINE_MODULATOR 1

EncoderPhaseShiftModulated::EncoderPhaseShiftModulated(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Encoder(_screenCols, _screenRows, _dir){

    // Set N
    //N = Ny * (Nx+Ncue);
    N = Ny * Nx + Ncue;

    // Precompute encoded patterns
    const float pi = M_PI;

    std::vector<cv::Mat> Lx; // Signal
    std::vector<cv::Mat> My; // Carriers

    for (unsigned int x = 0; x < Nx; x++) {
        float phaseX = 2 * pi / Nx * x;
        float pitch = (float)screenCols / (float)nPhaseX;
        cv::Mat lx(screenRows, screenCols, CV_32F);
        for (unsigned int r = 0; r < screenRows; r++) {
            for (unsigned int c = 0; c < screenCols; c++) {
                float phase = 2 * pi * c / pitch - phaseX;
                float amp = 0.5 + 0.5 * cos(phase);
                lx.at<float>(r, c) = amp;
            }
        }
        Lx.push_back(lx);
    }

#if USE_SINE_MODULATOR
    for (unsigned int y = 0; y < Ny; y++) {
        float phaseY = 2 * pi / Ny * y;
        //float pitch = (float)screenCols / (float)nPhaseY;
        float pitch = (float)screenRows / (float)nPhaseY;
        cv::Mat my(screenRows, screenCols, CV_32F);
        for (unsigned int r = 0; r < screenRows; r++) {
            for (unsigned int c = 0; c < screenCols; c++) {
                float phase = 2 * pi * r / pitch - phaseY;
                //float phase = 2 * pi * c / pitch - phaseY;
                float amp = 0.5 + 0.5 * cos(phase);
                my.at<float>(r, c) = amp;
            }
        }
        My.push_back(my);
    }
#else
    for (unsigned int y = 0; y < Ny; y++) {
        unsigned int period = screenCols / nPhaseY;
        //unsigned int period = screenRows / nPhaseY;
        unsigned int phase = period * y / Ny;
        cv::Mat my(screenRows, screenCols, CV_32F);
        for (unsigned int r = 0; r < screenRows; r++) {
            for (unsigned int c = 0; c < screenCols; c++) {
                float amp = (r + phase) % period > period / 2 ? 1 : 0;
                //float amp = (c + phase) % period > period / 2 ? 1 : 0;
                my.at<float>(r, c) = amp;
            }
        }
        My.push_back(my);
    }
#endif
    for (unsigned int x = 0; x < Lx.size(); x++) {
        for (unsigned int y = 0; y < My.size(); y++) {
            cv::Mat I;
            cv::multiply(Lx[x], My[y], I, 255, CV_8U);
            cv::cvtColor(I, I, CV_GRAY2RGB);
            patterns.push_back(I);
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
    return patterns[depth];    // Calculate modulation
}

// Decoder
DecoderPhaseShiftModulated::DecoderPhaseShiftModulated(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Decoder(_screenCols, _screenRows, _dir){

    //N = (Nx+Ncue) * Ny;
    N = Ny * Nx+Ncue;

    frames.resize(N);
}

void DecoderPhaseShiftModulated::setFrame(unsigned int depth, cv::Mat frame){
    frames[depth] = frame;
}

void DecoderPhaseShiftModulated::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading){

    const float pi = M_PI;

    std::vector<cv::Mat> framesX;

    // Decoding
#if USE_SINE_MODULATOR
    for(int x=Ny; x<=frames.size() - Ncue; x += Ny){

        std::vector<cv::Mat> framesY(frames.begin() + (x - Ny), frames.begin() + x);
        std::vector<cv::Mat> fIcomp = pstools::getDFTComponents(framesY);

        cv::Mat frameX;
        cv::magnitude(fIcomp[2], fIcomp[3], frameX);
        framesX.push_back(frameX);
    }
#else
    for(int x=Ny; x<=frames.size() - Ncue; x += Ny){

        std::vector<cv::Mat> framesY(frames.begin() + (x - Ny), frames.begin() + x);
        cv::Mat Imin = cv::min(framesY[0], framesY[1]);
        cv::Mat Imax = cv::max(framesY[0], framesY[1]);
        for (unsigned int i = 2; i < framesY.size(); i++) {
            Imin = cv::min(Imin, framesY[i]);
            Imax = cv::max(Imax, framesY[i]);
        }
        //std::vector<cv::Mat> fIcomp = pstools::getDFTComponents(framesY);

        //cv::Mat frameX;
        //cv::magnitude(fIcomp[2], fIcomp[3], frameX);
        cv::Mat Ld = Imax - Imin;
        framesX.push_back(Ld);
    }
#endif
    for (int i = 0; i < framesX.size(); i++) {
        cv::imwrite(cv::format("/tmp/%d.png", i), framesX[i]);
    }

//    cv::Mat upX0 = pstools::getMagnitude(frames[0], frames[1], frames[2]);
//    cv::Mat upX1 = pstools::getMagnitude(frames[3], frames[4], frames[5]);
//    cv::Mat upX2 = pstools::getMagnitude(frames[6], frames[7], frames[8]);
////cvtools::writeMat(upX0, "upX0.mat", "upX0");
////cvtools::writeMat(upX1, "upX1.mat", "upX1");
////cvtools::writeMat(upX2, "upX2.mat", "upX2");
//    up = pstools::getPhase(upX0, upX1, upX2);
    std::vector<cv::Mat> framesPhase(framesX.begin(), framesX.end());
    //std::vector<cv::Mat> framesPhase(framesX.begin(), framesX.end() - Ncue);
    std::vector<cv::Mat> fIcomp = pstools::getDFTComponents(framesPhase);
    cv::phase(fIcomp[2], -fIcomp[3], up);

    // Calculate modulation
    cv::magnitude(fIcomp[2], fIcomp[3], shading);
    shading.convertTo(shading, CV_8U, 2.0/Nx);

    //std::vector<cv::Mat> framesCue(framesX.end()-Ncue, framesX.end());
    std::vector<cv::Mat> framesCue(frames.end()-Ncue, frames.end());
    fIcomp = pstools::getDFTComponents(framesCue);
    cv::Mat upCue;
    cv::phase(fIcomp[2], -fIcomp[3], upCue);

    up = pstools::unwrapWithCue(up, upCue, nPhaseX);
    up *= screenCols/(2*pi);

    // Threshold modulation image for mask
    mask = shading > 20;
//cvtools::writeMat(mask, "mask.mat");
//    cv::Mat edges;
//    cv::Sobel(up, edges, -1, 1, 1, 7);
//    edges = abs(edges) < 200;


}
