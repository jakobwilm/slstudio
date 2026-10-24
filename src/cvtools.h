#ifndef CVTOOLS_H
#define CVTOOLS_H

#include <opencv2/core.hpp>
#include <vector>

namespace cvtools {
bool findPartialCirclesGrid(const cv::Mat &im, std::vector<cv::Point2f> &q,
                            std::vector<cv::Point3f> &Q,
                            const float circleSpacing);
} // namespace cvtools

#endif // CVTOOLS_H
