#include "CodecPhaseShiftSpatialUnwrap.h"
#include <math.h>

#include "pstools.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

// Encoder

EncoderPhaseShiftSpatialUnwrap::EncoderPhaseShiftSpatialUnwrap(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir,
                                               const PhaseShiftSettings &settings) : Encoder(_screenCols, _screenRows, _dir){
    CodecAlgorithmSettings requestedSettings;
    requestedSettings.spatialUnwrap = settings;
    validateCodecSettings(requestedSettings);

    // Set N
    this->N = settings.steps;

    // Precompute encoded patterns
    const float pi = M_PI;

    // Horizontally encoding patterns
    for(unsigned int i=0; i<N; i++){
        float phase = 2.0*pi/N * i;
        float pitch = (float)screenCols/settings.periods;
        cv::Mat patternI(1,1,CV_8U);
        patternI = pstools::computePhaseVector(screenCols, phase, pitch);
        patterns.push_back(patternI.t());
    }

}

cv::Mat EncoderPhaseShiftSpatialUnwrap::getEncodingPattern(unsigned int depth) const{
    return patterns.at(depth);
}

// Decoder
DecoderPhaseShiftSpatialUnwrap::DecoderPhaseShiftSpatialUnwrap(unsigned int _screenCols, unsigned int _screenRows, CodecDir direction,
                                               const PhaseShiftSettings &requestedSettings, int centerFringeOrder)
    : Decoder(_screenCols, _screenRows, direction), settings(requestedSettings), centerFringeOrder(centerFringeOrder){
    CodecAlgorithmSettings algorithmSettings;
    algorithmSettings.spatialUnwrap = settings;
    validateCodecSettings(algorithmSettings);
    if (centerFringeOrder < 0 || centerFringeOrder >= settings.periods)
        throw std::invalid_argument("Center fringe order must be between zero and fringe periods minus one");
    this->N = settings.steps;
}

void DecoderPhaseShiftSpatialUnwrap::decodeFrames(const std::vector<cv::Mat> &frames, cv::Mat &up,
                                                 cv::Mat &, cv::Mat &mask, cv::Mat &shading) {
    validateCodecFrames(frames, N);

    const float pi = M_PI;

    // Calculate multiple phase image
    if (settings.steps == 3) {
        up = pstools::getPhase(frames[0], frames[1], frames[2]);
        shading = pstools::getMagnitude(frames[0], frames[1], frames[2]);
    } else {
        const auto components = pstools::getDFTComponents(frames);
        cv::phase(components[2], -components[3], up);
        cv::magnitude(components[2], components[3], shading);
        shading.convertTo(shading, CV_8U, 6.0 / settings.steps);
    }

    // Create mask from modulation image
    mask = shading > modulationThreshold(25);
    if (cv::countNonZero(mask) == 0) return;

    spatialUnwrapping(up, mask, centerFringeOrder);
    up *= screenCols/(2.0*pi*settings.periods);

}
