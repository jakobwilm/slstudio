#include "CodecPhaseShiftMicro.h"
#include <math.h>
#include <iomanip>

#include "pstools.h"


#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

// Encoder
EncoderPhaseShiftMicro::EncoderPhaseShiftMicro(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir,
                                             const PhaseShiftMicroSettings &settings) : Encoder(_screenCols, _screenRows, _dir){
    CodecAlgorithmSettings requestedSettings;
    requestedSettings.micro = settings;
    validateCodecSettings(requestedSettings);
    const auto &frequencies = MicroPhasePitches;
    const auto F = frequencies.size();

    // Set N
    N = F + settings.steps - 1;

    // Precompute encoded patterns
    const float pi = M_PI;

    // Main frequency encoding patterns
    for(int i=0; i<settings.steps; i++){
        float phase = -2.0*pi/settings.steps * i;
        float pitch = frequencies[0];
        cv::Mat patternI(1,1,CV_8U);
        patternI = pstools::computePhaseVector(screenCols, phase, pitch);
        patternI = patternI.t();
        patterns.push_back(patternI);
    }

    // Additional frequency patterns
    for(unsigned int i=1; i<F; i++){
        float phase = 0.0;
        if(i%2)
            phase = pi;
        float pitch = frequencies[i];
        cv::Mat patternI;
        patternI = pstools::computePhaseVector(screenCols, phase, pitch);
        patternI = patternI.t();
        patterns.push_back(patternI);
    }

    #if 0
        for(unsigned int i=0; i<patterns.size(); i++){
            std::stringstream fileNameStream;
            fileNameStream << "pattern_" << std::setw(2) << std::setfill('0') << i << ".bmp";
            cv::imwrite(fileNameStream.str(), patterns[i]);
        }

    #endif
}

cv::Mat EncoderPhaseShiftMicro::getEncodingPattern(unsigned int depth) const{
    return patterns.at(depth);
}

// Decoder
DecoderPhaseShiftMicro::DecoderPhaseShiftMicro(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir,
                                             const PhaseShiftMicroSettings &requestedSettings)
    : Decoder(_screenCols, _screenRows, _dir), settings(requestedSettings){
    CodecAlgorithmSettings algorithmSettings;
    algorithmSettings.micro = settings;
    validateCodecSettings(algorithmSettings);

    N = MicroPhasePitches.size() + settings.steps - 1;
}

void DecoderPhaseShiftMicro::decodeFrames(const std::vector<cv::Mat> &frames, cv::Mat &up,
                                         cv::Mat &, cv::Mat &mask, cv::Mat &shading) {
    validateCodecFrames(frames, N);

    const float pi = M_PI;
    const auto &frequencies = MicroPhasePitches;
    const int F = static_cast<int>(frequencies.size());

    int rows = frames[0].rows;
    int cols = frames[0].cols;

    // Construct system of equations
    cv::Mat Mmicro(N, F+2, CV_32F, cv::Scalar(0.0));
    for (int shift = 0; shift < settings.steps; ++shift) {
        Mmicro.at<float>(shift, 1) = cos(2.0*pi/settings.steps * shift);
        Mmicro.at<float>(shift, 2) = -sin(2.0*pi/settings.steps * shift);
    }
    Mmicro.col(0).setTo(1.0);
    for(int i=0; i<F-1; i++)
        Mmicro.at<float>(settings.steps+i, 3+i) = i % 2 ? 1.0f : -1.0f;

    cv::Mat Rmicro(N, rows*cols, CV_32F);
    for(unsigned int i=0; i<N; i++){
        frames[i].reshape(0, 1).convertTo(Rmicro.row(i), CV_32F);
    }

    // Solve
    cv::Mat Ufact;
    if (!cv::solve(Mmicro, Rmicro, Ufact, settings.steps == 3 ? cv::DECOMP_LU : cv::DECOMP_SVD))
        throw std::runtime_error("Cannot solve the micro phase-shift system");

    // Shading
    cv::Mat amp;
    cv::magnitude(Ufact.row(1), Ufact.row(2), amp);
    shading = amp.reshape(0, rows);
    shading.convertTo(shading, CV_8U, 2.0);
    mask = shading > modulationThreshold(20);

    cv::Mat CosSin(F+1, rows*cols, CV_32F);
    for(unsigned int i=0; i<F+1; i++){
        cv::divide(Ufact.row(i+1), amp, CosSin.row(i));
    }


    // Reference CosSin values
    cv::Mat RefCosSin(F+1, screenCols, CV_32F);
    for(unsigned int i=0; i<screenCols; i++){
        RefCosSin.at<float>(0,i) = std::cos(2*pi*i/frequencies[0]);
        RefCosSin.at<float>(1,i) = std::sin(2*pi*i/frequencies[0]);
        for(unsigned int j=2; j<F+1; j++){
            RefCosSin.at<float>(j,i) = std::cos(2*pi*i/frequencies[j-1]);
        }
    }


    // Find best match value
    cv::Mat upCueMatch(1, rows*cols, CV_32F);
    for(int i=0; i<rows*cols; i++){
        if (!mask.at<unsigned char>(i / cols, i % cols)) {
            upCueMatch.at<float>(0, i) = 0;
            continue;
        }
        int bestMatch = -1;
        float bestDist = INFINITY;
        for(unsigned int j=0; j<screenCols; j++){
            float dist = 0.0;
            for(unsigned int k=0; k<F+1; k++){
                float diff = CosSin.at<float>(k, i)-RefCosSin.at<float>(k, j);
                dist += diff*diff;
            }
            if(dist < bestDist){
                bestDist = dist;
                bestMatch = j;
            }
        }
        upCueMatch.at<float>(0, i) = bestMatch;
    }

    cv::Mat upCue = upCueMatch.reshape(0, rows);

    upCue *= (2*pi)/screenCols;
    if (settings.steps == 3)
        up = pstools::getPhase(frames[0], frames[2], frames[1]);
    else {
        cv::Mat phase;
        cv::phase(Ufact.row(1), Ufact.row(2), phase);
        up = phase.reshape(0, rows);
    }
    up = pstools::unwrapWithCue(up, upCue, screenCols/frequencies[0]);
    up *= screenCols/(2*pi);
//up = upCue;

}
