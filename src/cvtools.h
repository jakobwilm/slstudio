#ifndef CVTOOLS_H
#define CVTOOLS_H

#include <opencv2/opencv.hpp>

namespace cvtools {
cv::Mat plotScatterXY(const std::vector<cv::Point2d> &P, const cv::Size size,
                      double &minX, double &maxX, double &minY);
cv::Vec3f applyHomTransform(const cv::Matx44f &T, const cv::Vec3f &p);
cv::Vec2f applyHomTransform(const cv::Matx33f &T, const cv::Vec2f &p);
std::vector<cv::Point2f> applyHomTransform(const cv::Matx33f &T,
                                           const std::vector<cv::Point2f> &ps);
std::vector<size_t>
findNearestNeighborsAngleSorted(const cv::KeyPoint queryPoint,
                                const std::vector<cv::KeyPoint> searchPoints,
                                const int N);
cv::Mat fitHomography(std::vector<cv::Point2f> q1, std::vector<cv::Point2f> q2,
                      float &ssd);
bool findPartialCirclesGrid(const cv::Mat &im, std::vector<cv::Point2f> &q,
                            std::vector<cv::Point3f> &Q,
                            const float circleSpacing);
void phaseCorrelate(const cv::Mat &im1, const cv::Mat &im2, float &scale,
                    float &angle, cv::Point2f &shift);
cv::Mat logPolar(const cv::Mat &image, float scale);
void initDistortMap(const cv::Matx33f cameraMatrix,
                    const cv::Vec<float, 5> distCoeffs, const cv::Size size,
                    cv::Mat &map1, cv::Mat &map2);
cv::Mat diamondDownsample(cv::Mat &pattern);
void imshow(const char *windowName, cv::Mat im, unsigned int x, unsigned int y);
void imagesc(const char *windowName, cv::Mat im);
cv::Mat histimage(cv::Mat histogram);
void hist(const char *windowName, cv::Mat im, unsigned int x, unsigned int y);
void writeMat(cv::Mat const &mat, const char *filename,
              const char *varName = "A", bool bgr2rgb = true);
} // namespace cvtools

#endif // CVTOOLS_H
