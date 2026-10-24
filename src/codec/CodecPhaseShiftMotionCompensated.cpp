#include "CodecPhaseShiftMotionCompensated.h"
#include <math.h>

#include "pstools.h"

#include <opencv2/imgproc/imgproc_c.h>

//#include <opencv2/video/tracking.hpp>

#ifndef M_PI
    #define M_PI 3.14159265359
#endif

// Encoder
EncoderPhaseShiftMotionCompensated::EncoderPhaseShiftMotionCompensated(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Encoder(_screenCols, _screenRows, _dir){

    // Set N
    N = 3;

    // Precompute encoded patterns
    const float pi = M_PI;

    // 0 deg
    cv::Mat patternI = pstools::computePhaseVector(screenCols, 0.0, screenCols);
    patterns.push_back(patternI.t());

    // 90 deg
    patternI = pstools::computePhaseVector(screenCols, pi/2.0, screenCols);
    patterns.push_back(patternI.t());

    // Flat image
    patternI.create(5, 5, CV_8UC3);
    patternI.setTo(0.5*255.0);
    patterns.push_back(patternI);
}

cv::Mat EncoderPhaseShiftMotionCompensated::getEncodingPattern(unsigned int depth) const{
    return patterns[depth];
}

// Decoder
DecoderPhaseShiftMotionCompensated::DecoderPhaseShiftMotionCompensated(unsigned int _screenCols, unsigned int _screenRows, CodecDir,
                                         bool compensate)
    : Decoder(_screenCols, _screenRows), motionCompensation(compensate) {
    N = 3;
}

void DecoderPhaseShiftMotionCompensated::setMotionCompensation(bool enabled) {
    if (motionCompensation != enabled) lastShading.release();
    motionCompensation = enabled;
}

// Motion compensation reference: Wilm, Jakob, et al.
// "Correction of motion artifacts for real-time structured light."
// Scandinavian Conference on Image Analysis.
// Cham: Springer International Publishing, 2015.
void DecoderPhaseShiftMotionCompensated::decodeFrames(const std::vector<cv::Mat> &frames, cv::Mat &up,
                                                      cv::Mat &, cv::Mat &mask, cv::Mat &shading) {
    validateCodecFrames(frames, N, false);

    const float pi = M_PI;

    cv::Mat_<float> I1(frames[0]);
    cv::Mat_<float> I2(frames[1]);
    cv::Mat_<float> I3(frames[2]);

    if (motionCompensation) {
        if (lastShading.empty() || lastShading.size() != I3.size()) lastShading = I3.clone();
        const cv::Point2f shift = cv::phaseCorrelate(lastShading, I3);
        lastShading = I3.clone();
        const cv::Point2f center(I3.cols / 2.0f, I3.rows / 2.0f);
        const cv::Point2f center1(center.x + 0.333 * shift.x, center.y + 0.333 * shift.y);
        const cv::Point2f center2(center.x - 0.333 * shift.x, center.y - 0.333 * shift.y);

        cv::Mat1f alignedFirst, alignedSecond;
        cv::getRectSubPix(I1, I1.size(), center1, alignedFirst);
        cv::getRectSubPix(I2, I2.size(), center2, alignedSecond);
        I1 = alignedFirst;
        I2 = alignedSecond;
    }

    cv::phase(I1-I3, I2-I3, up);
    up *= screenCols/(2*pi);

    cv::GaussianBlur(up, up, cv::Size(0,0), 3, 3);

    cv::Mat mag;
    cv::magnitude(I1-I3, I2-I3, mag);

    shading = 2.0*frames[2];


    // Create mask from modulation image and erode
    mask.create(shading.size(), cv::DataType<bool>::type);
    mask.setTo(true);
    mask = (shading > 80) & (shading < 254);

    // mask outlier-prone regions
    cv::Mat dx, dy;
    cv::Sobel(I3, dx, -1, 1, 0, 3);
    cv::Sobel(I3, dy, -1, 0, 1, 3);
    cv::Mat edgesShading = abs(dx) + abs(dy);

    cv::Sobel(up, dx, -1, 1, 0, 3);
    cv::Sobel(up, dy, -1, 0, 1, 3);
    cv::Mat edgesUp = abs(dx) + abs(dy);

    cv::Sobel(up, dx, -1, 1, 0, 3);
    cv::Sobel(up, dy, -1, 0, 1, 3);

    mask = mask & (edgesShading < 100) & (edgesUp < 130);

}
