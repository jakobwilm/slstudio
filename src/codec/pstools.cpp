#include "pstools.h"

#include <cmath>

#ifndef M_PI
    #define M_PI 3.14159265359
#endif

namespace pstools{

// Cosine function vector (3-channel)
cv::Mat computePhaseVector(unsigned int length, float phase, float pitch){

    cv::Mat phaseVector(length, 1, CV_8UC3);

    const float pi = static_cast<float>(M_PI);

    // Loop through vector
    for(int i=0; i<phaseVector.rows; i++){
        // Amplitude of channels
        float amp = static_cast<float>(0.5*(1+cos(2*pi*i/pitch - phase)));
        phaseVector.at<cv::Vec3b>(i, 0) = cv::Vec3b::all(static_cast<unsigned char>(255.0*amp));
    }

    return phaseVector;
}

// Absolute phase from 3 frames
cv::Mat getPhase(const cv::Mat I1, const cv::Mat I2, const cv::Mat I3){

    cv::Mat_<float> I1_(I1);
    cv::Mat_<float> I2_(I2);
    cv::Mat_<float> I3_(I3);

    cv::Mat phase;

    // One call approach
    cv::phase(2.0*I1_-I3_-I2_, sqrt(3.0)*(I2_-I3_), phase);
    return phase;

}

// Absolute magnitude from 3 frames
cv::Mat getMagnitude(const cv::Mat I1, const cv::Mat I2, const cv::Mat I3){

    cv::Mat_<float> I1_(I1);
    cv::Mat_<float> I2_(I2);
    cv::Mat_<float> I3_(I3);

    cv::Mat magnitude;

    // One call approach
    cv::magnitude(2.0*I1_-I2_-I3_, sqrt(3.0)*(I2_-I3_), magnitude);
    magnitude.convertTo(magnitude, CV_8U);

    return magnitude;
}

void getPhaseAndMagnitude(const cv::Mat &first, const cv::Mat &second, const cv::Mat &third,
                          cv::Mat &phase, cv::Mat &magnitude){
    const cv::Mat_<float> firstFloat(first), secondFloat(second), thirdFloat(third);
    const cv::Mat cosine = 2.0 * firstFloat - thirdFloat - secondFloat;
    const cv::Mat sine = sqrt(3.0) * (secondFloat - thirdFloat);
    cv::cartToPolar(cosine, sine, magnitude, phase);
    magnitude.convertTo(magnitude, CV_8U);
}

// Absolute phase and magnitude from N frames
std::vector<cv::Mat> getDFTComponents(const std::vector<cv::Mat> frames){

    // Keep the DFT input single-channel to support both OpenCV 4 and 5 channel limits.
    const int count = static_cast<int>(frames.size());
    const int rows = frames[0].rows;
    const int pixels = rows * frames[0].cols;
    cv::Mat input(pixels, count, CV_32F);
    for (int index = 0; index < count; ++index)
        frames[index].reshape(1, pixels).convertTo(input.col(index), CV_32F);

    cv::Mat transformed;
    cv::dft(input, transformed, cv::DFT_ROWS | cv::DFT_COMPLEX_OUTPUT);
    std::vector<cv::Mat> realAndImaginary;
    cv::split(transformed, realAndImaginary);

    std::vector<cv::Mat> fIcomp;
    fIcomp.reserve(count * 2);
    for (int frequency = 0; frequency < count; ++frequency) {
        fIcomp.push_back(realAndImaginary[0].col(frequency).clone().reshape(0, rows));
        fIcomp.push_back(realAndImaginary[1].col(frequency).clone().reshape(0, rows));
    }

    return fIcomp;

}

// Phase unwrapping by means of a phase cue
cv::Mat unwrapWithCue(const cv::Mat up, const cv::Mat upCue, float nPhases){

    const float pi = static_cast<float>(M_PI);

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


}
