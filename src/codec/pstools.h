#ifndef PSTOOLS_H
#define PSTOOLS_H

// Tools for phase shifting profilometry

#include <opencv2/core.hpp>
#include <vector>

namespace pstools {

    cv::Mat computePhaseVector(unsigned int length, float phase, float pitch);
    cv::Mat getPhase(const cv::Mat I1, const cv::Mat I2, const cv::Mat I3);
    cv::Mat getMagnitude(const cv::Mat I1, const cv::Mat I2, const cv::Mat I3);
    void getPhaseAndMagnitude(const cv::Mat &first, const cv::Mat &second, const cv::Mat &third,
                              cv::Mat &phase, cv::Mat &magnitude);
    std::vector<cv::Mat> getDFTComponents(const std::vector<cv::Mat> frames);
    cv::Mat unwrapWithCue(const cv::Mat up, const cv::Mat upCue, float nPhases);
}

#endif // PSTOOLS_H
