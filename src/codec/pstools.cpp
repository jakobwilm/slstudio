#include "pstools.h"

#ifndef M_PI
    #define M_PI 3.14159265359
#endif

namespace pstools{

// Cosine function vector (3-channel)
cv::Mat computePhaseVector(unsigned int length, float phase, float pitch){

    cv::Mat phaseVector(length, 1, CV_8UC3);

    const float pi = M_PI;

    // Loop through vector
    for(int i=0; i<phaseVector.rows; i++){
        // Amplitude of channels
        float amp = 0.5*(1+cos(2*pi*i/pitch - phase));
        phaseVector.at<cv::Vec3b>(i, 0) = cv::Vec3b(255.0*amp,255.0*amp,255.0*amp);
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

// Absolute phase and magnitude from N frames
std::vector<cv::Mat> getDFTComponents(const std::vector<cv::Mat> frames){

    unsigned int N = frames.size();

    std::vector<cv::Mat> framesReverse = frames;
    std::reverse(framesReverse.begin(), framesReverse.end());

    // DFT approach
    cv::Mat I;
    cv::merge(frames, I);
    unsigned int w = I.cols;
    unsigned int h = I.rows;
    I = I.reshape(1, h*w);
    I.convertTo(I, CV_32F);
    cv::Mat fI;
    cv::dft(I, fI, cv::DFT_ROWS + cv::DFT_COMPLEX_OUTPUT);
    fI = fI.reshape(N*2, h);

    std::vector<cv::Mat> fIcomp;
    cv::split(fI, fIcomp);

    return fIcomp;

}

// Phase unwrapping by means of a phase cue
cv::Mat unwrapWithCue(const cv::Mat up, const cv::Mat upCue, unsigned int nPhases){

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


}
