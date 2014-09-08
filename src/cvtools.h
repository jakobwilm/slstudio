#ifndef CVTOOLS_H
#define CVTOOLS_H

#include <opencv2/opencv.hpp>

namespace cvtools{
    void initDistortMap(const cv::Matx33f cameraMatrix, const cv::Vec<float, 5> distCoeffs, const cv::Size size, cv::Mat &map1, cv::Mat &map2);
    cv::Mat diamondDownsample(cv::Mat &pattern);
    void imshow(const char *windowName, cv::Mat im, unsigned int x, unsigned int y);
    void imagesc(const char* windowName, cv::Mat im);
    cv::Mat histimage(cv::Mat histogram);
    void hist(const char *windowName, cv::Mat im, unsigned int x, unsigned int y);
    void writeMat( cv::Mat const& mat, const char* filename, const char* varName = "A", bool bgr2rgb = true);
}

#endif // CVTOOLS_H
