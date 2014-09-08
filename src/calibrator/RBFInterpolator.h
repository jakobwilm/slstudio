/* Radial Basis Function Interpolation.
 * Enables one to perform RBF interpolation for e.g. warping images or data interpolation.
 * Currently supports only Gaussian RBFs.
 * Currently supports only 2d knot points and associated 2d data values.
 */

#ifndef RBFINTERPOLATOR_H
#define RBFINTERPOLATOR_H

#include <opencv2/opencv.hpp>

enum tRBF {RBF_GAUSSIAN};

class RBFInterpolator{
    public:
        RBFInterpolator(tRBF _type = RBF_GAUSSIAN, float _regularizationK = 0.0) : type(_type), regularizationK(_regularizationK){}
        RBFInterpolator(std::vector<cv::Point2f> dataPoints, tRBF = RBF_GAUSSIAN, float _regularizationK = 0.0);
        void setDataPoints(const std::vector<cv::Point2f> x, const std::vector<cv::Point2f> f);
        cv::Point2f interpolate(const std::vector<cv::Point2f> x, const cv::Point2f xStar);
    private:
        tRBF type;
        float regularizationK;
        cv::Mat_<float> lambda;
};

#endif // RBFINTERPOLATOR_H
