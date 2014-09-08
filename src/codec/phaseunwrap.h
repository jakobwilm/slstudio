#ifndef PHASEUNWRAP_H
#define PHASEUNWRAP_H

#include <opencv2/opencv.hpp>

namespace phaseunwrap {

    // Phase Unwrap according to Zhang's method
    // Ref: Zhang, S., Li, Xialoin, Yau, Shing-Tung; Multilevel quality-guided phase unwrapping..., Appl Opt 2007 vol. 46(1) pp. 50 -- 57
    // Implementation modified from http://code.google.com/p/structured-light/
    cv::Mat createqualitymap(const cv::Mat phase, const cv::Mat mask);
    std::vector<float> computethresholds(cv::Mat quality, const cv::Mat mask);
    void unwrap(cv::Mat phase, cv::Mat quality, cv::Mat mask, const std::vector<float> thresholds);

}

#endif // PHASEUNWRAP_H
