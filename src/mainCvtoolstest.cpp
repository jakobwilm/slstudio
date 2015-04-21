#include <iostream>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include "cvtools.h"

int main(){

    cv::Mat im = cv::imread("lena.png", CV_LOAD_IMAGE_GRAYSCALE);

    cv::Mat scaleRotationMatrix = cv::getRotationMatrix2D(cv::Point2f(im.cols/2.0, im.rows/2.0), 10.0, 1.0);

    cv::Mat imTransformed;
    cv::warpAffine(im, imTransformed, scaleRotationMatrix, im.size());

    cv::Mat translationMatrix = (cv::Mat_<float>(2,3) << 1, 0, 0, 0, 1, 0);
    cv::warpAffine(imTransformed, imTransformed, translationMatrix, imTransformed.size(), CV_INTER_LINEAR, cv::BORDER_CONSTANT, 0.0);

    cv::imwrite("im.png", im);
    cv::imwrite("imTransformed.png", imTransformed);

    // try to recover scale and rotation
    float scale, rotation;
    cv::Point2f shift;

    cvtools::phaseCorrelate(im, imTransformed, scale, rotation, shift);

    std::cout << scale << std::endl << rotation << std::endl << shift << std::endl;

//    cv::namedWindow("im1", CV_WINDOW_AUTOSIZE);
//    cv::imshow("im1", im1);
//    cv::waitKey();


    return 0;
}

