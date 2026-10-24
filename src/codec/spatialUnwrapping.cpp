#include "spatialUnwrapping.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#define VAL_UNWRAPPED -1.0

#ifndef M_PI
    #define M_PI 3.14159265359
#endif

// Modified from http://code.google.com/p/structured-light/.
namespace {

// Wrap differences of wrapped phase
float wrapphasedifference(float ph1, float ph2){

    const float pi = M_PI;

    // Estimates the true gradient by wrapping the differences of wrapped phase
    float innerDiff = (ph1 - ph2)/(2.0*pi);

    if(innerDiff > 0.5)
        return innerDiff-1.0;
    else if(innerDiff < -0.5)
        return innerDiff+1.0;
    else
        return innerDiff;

}

// Create quality map
cv::Mat createqualitymap(const cv::Mat phase, const cv::Mat mask){

    // Clear whole quality map
    cv::Mat quality(phase.size(), CV_32FC1);
    quality = cv::Scalar(0.0);

    // Go through each pixel in the phase map except the borders
    float up, down, left, right;
    for(int r = 1; r < phase.rows - 1; r++){
        for(int c = 1; c < phase.cols - 1; c++){

            // If this pixel should be processed
            if(mask.at<unsigned char>(r,c) != 0){

                // Compute wrapped phase differences
                up = std::fabs(wrapphasedifference(phase.at<float>(r,c), phase.at<float>(r-1,c)));
                down = std::fabs(wrapphasedifference(phase.at<float>(r+1,c), phase.at<float>(r,c)));
                left = std::fabs(wrapphasedifference(phase.at<float>(r,c), phase.at<float>(r,c-1)));
                right = std::fabs(wrapphasedifference(phase.at<float>(r,c+1), phase.at<float>(r,c)));

                // Quality is the highest of the four values
                quality.at<float>(r,c) = std::max(std::max(std::max(up,down),left),right);
            }
        }
    }

    return quality;
}

std::vector<float> computethresholds(cv::Mat quality, const cv::Mat mask){

    std::vector<float> thresholds(3);

    // Image size
    int numVals = 0;
    // Compute quality mean value
    float meanValue = 0.0;

    for(int r = 0; r < quality.rows; r++){
        for(int c = 0; c < quality.cols; c++){

            if(mask.at<unsigned char>(r,c) != 0){
                meanValue += quality.at<float>(r,c);
                numVals++;
            }
        }
    }
    if (!numVals) throw std::invalid_argument("Phase unwrapping requires valid samples");
    meanValue /= numVals;
    // Compute standard deviation
    float stdDev = 0.0f;
    for(int r = 0; r < quality.rows; r++){
        for(int c = 0; c < quality.cols; c++){

            if(mask.at<unsigned char>(r,c) != 0)
                stdDev += (quality.at<float>(r,c) - meanValue) * (quality.at<float>(r,c) - meanValue);
        }
    }
    stdDev = sqrt(stdDev / numVals);

    // Get thresholds
    // First threshold is equal to mean Value
    thresholds[0] = meanValue;
    for(unsigned int i = 1; i < thresholds.size(); i++){
        thresholds[i] = thresholds[0] + (1 << (i - 1)) * stdDev;
    }

    return thresholds;
}

float unwrappix(float currPhase, float nbrPhase){

    if(currPhase - nbrPhase > M_PI)
        return -1.0;
    else if(currPhase - nbrPhase < -M_PI)
        return 1.0;
    else
        return 0.0;

}

void unwrappatch(cv::Mat phase, cv::Mat quality, cv::Mat mask, cv::Mat phOffset, float threshold, cv::Point ptStart, int borderX, int borderY, int stepX, int stepY){

    // Prepare array for possible remainder pixels that has been left over for further processing
    int numRem = std::fabs((float)ptStart.x - borderX) * std::fabs((float)ptStart.y - borderY);
    std::vector<cv::Point2i> remainders;
    remainders.reserve(numRem);
    const auto inside = [&](int row, int col) {
        return row >= 0 && row < phase.rows && col >= 0 && col < phase.cols;
    };
    const auto unwrapped = [&](int row, int col) {
        return inside(row, col) && quality.at<float>(row, col) == VAL_UNWRAPPED;
    };
    const auto valid = [&](int row, int col) {
        return inside(row, col) && mask.at<unsigned char>(row, col) != 0;
    };

    // Go thorugh each row and column in this patch
    for(int r = ptStart.y; r != borderY-1; r += stepY){

        for(int c = ptStart.x; c != borderX-1; c += stepX){

            // If the quality is in the current level
            if(valid(r,c) && (quality.at<float>(r,c) <= threshold)){

                // Try to unwrap from first start facing pixel
                if(unwrapped(r,c-stepX)){

                    phOffset.at<float>(r,c) = phOffset.at<float>(r,c-stepX) + unwrappix(phase.at<float>(r,c), phase.at<float>(r,c-stepX));
                    quality.at<float>(r,c) = VAL_UNWRAPPED;

                } else {

                    // Try to unwrap from second start facing pixel
                    if(unwrapped(r-stepY,c)){

                        phOffset.at<float>(r,c) = phOffset.at<float>(r-stepY,c) + unwrappix(phase.at<float>(r,c), phase.at<float>(r-stepY,c));
                        quality.at<float>(r,c) = VAL_UNWRAPPED;

                    } else {
                        // If it wasn't able to unwrap from any start facing pixel, check border facing pixels and
                        // if there's valid one, add current pixel to the leftovers
                        if(valid(r,c+stepX) || valid(r+stepY,c))
                            remainders.push_back(cv::Point2i(c,r));
                    }
                }
            }
        }
    }

    // Now process remainder pixels that have been left over
    for(int i = static_cast<int>(remainders.size())-1; i >= 0; i--){

        // Get current pixel's position
        int r = remainders[i].y;
        int c = remainders[i].x;

        // Try to unwrap from the first border facing pixel
        if(unwrapped(r,c+stepX)){

            phOffset.at<float>(r,c) = phOffset.at<float>(r,c+stepX) + unwrappix(phase.at<float>(r,c), phase.at<float>(r,c+stepX));
            quality.at<float>(r,c) = VAL_UNWRAPPED;

        } else {

            // Try to unwrap from the second border facing pixel
            if(unwrapped(r+stepY,c)){

                phOffset.at<float>(r,c) = phOffset.at<float>(r+stepY,c) + unwrappix(phase.at<float>(r,c), phase.at<float>(r+stepY,c));
                quality.at<float>(r,c) = VAL_UNWRAPPED;

            } else {
                continue;
            }
        }
    }
}


void unwrapRelativePhase(cv::Mat phase, cv::Mat quality, cv::Mat mask, const std::vector<float> thresholds)
{
    // Start from the center pixel
    cv::Point2i ptStart = cv::Point2i(quality.cols/2, quality.rows/2);

    cv::Mat searchMask(quality.size(), CV_8UC1, cv::Scalar(0.0));
    const auto center = cv::Rect(ptStart.x-10, ptStart.y-10, 20, 20) & cv::Rect({}, quality.size());
    searchMask(center).setTo(255);
    cv::bitwise_and(searchMask, mask, searchMask);
    if (cv::countNonZero(searchMask) == 0) searchMask = mask;
    if (cv::countNonZero(searchMask) == 0)
        throw std::invalid_argument("Phase unwrapping requires valid samples");

    // Choose highest quality starting point
    cv::minMaxLoc(quality, NULL, NULL, &ptStart, NULL, searchMask);

    // Mat for storing offsets
    cv::Mat phOffset = cv::Mat(phase.size(), CV_32FC1, cv::Scalar(0.0));

    // Define starting point as unwrapped
    quality.at<float>(ptStart.y, ptStart.x) = VAL_UNWRAPPED;
    phOffset.at<float>(ptStart.y, ptStart.x) = 0;

    // Unwrap at all quality levels
    for(unsigned int i = 0; i < thresholds.size(); i++){
        unwrappatch(phase, quality, mask, phOffset, thresholds[i], ptStart, 0, 0, -1, -1);
        unwrappatch(phase, quality, mask, phOffset, thresholds[i], ptStart, 0, phase.rows+1, -1, 1);
        unwrappatch(phase, quality, mask, phOffset, thresholds[i], ptStart, phase.cols+1, 0, 1, -1);
        unwrappatch(phase, quality, mask, phOffset, thresholds[i], ptStart, phase.cols+1, phase.rows+1, 1, 1);
    }

    // Update mask to reflect which pixels were actually unwrapped
    mask = cv::Scalar(false);

    // Now unwrap all pixels
    for(int r = 0; r < phase.rows; r++){
        for(int c = 0; c < phase.cols; c++){

            // If the pixel has been unwrapped, actualize it's phase and also set mask for this pixel
            if(quality.at<float>(r,c) == VAL_UNWRAPPED){

                // Unwrap this pixel's phase
                phase.at<float>(r,c) += phOffset.at<float>(r,c)*2.0*M_PI;
                mask.at<unsigned char>(r,c) = 255;
            } else {
                //mask.at<float>(r,c) = false;
                // If this pixel was not unwrapped, set it's phase to 0
                //phase.at<float>(r,c) = 0;
            }
        }
    }
}


}

void spatialUnwrapping(cv::Mat &phase, cv::Mat &mask, int centerFringeOrder) {
    if (phase.empty() || phase.dims != 2 || phase.type() != CV_32FC1 ||
        mask.dims != 2 || mask.type() != CV_8UC1 || phase.size() != mask.size())
        throw std::invalid_argument("Spatial unwrapping requires matching float phase and Mono8 mask images");
    if (centerFringeOrder < 0)
        throw std::invalid_argument("Center fringe order must be nonnegative");
    cv::Mat quality = createqualitymap(phase, mask);
    cv::GaussianBlur(quality, quality, cv::Size(0, 0), 3, 3);
    const auto thresholds = computethresholds(quality, mask);
    unwrapRelativePhase(phase, quality, mask, thresholds);
    const float pi = M_PI;
    phase += centerFringeOrder * 2.0 * pi;
}
