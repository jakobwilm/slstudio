#include <iostream>
#include <unistd.h>
#include "Codec.h"
#include "CodecGrayCode.h"
#include "CodecPhaseShift.h"

#include "Projector.h"

#include <opencv2/opencv.hpp>

int main(){

    Projector projector(1);
    unsigned int screenResX, screenResY;
    projector.getScreenRes(&screenResX, &screenResY);
    //unsigned int screenResX = 608, screenResY = 684;

    bool diamondPattern = true;

    // Codec for horizontal direction
    CodecGrayCode codecHorz(screenResX, screenResY, diamondPattern);

    // Codec for vertical direction (slight misuse of code)
    CodecGrayCode codecVert(screenResY, screenResX, diamondPattern);

    int NHorz = codecHorz.getNPatterns();
    int NVert = codecVert.getNPatterns();

    cv::namedWindow("Pattern");

    for(int i=0; i<NHorz; i++){
        cv::Mat pattern = codecHorz.getEncodingPattern(i);
        pattern = cv::repeat(pattern, screenResY/pattern.rows, screenResX/pattern.cols);
        cv::imshow("Pattern", pattern);
        projector.displayTexture(pattern.ptr(), pattern.cols, pattern.rows);
        // Treat pattern as a pseudo captured frame
        std::vector<cv::Mat> channels;
        cv::split(pattern, channels);
        codecHorz.setFrame(i, channels[0]);
        std::cout << "Horizontal Pattern " << i+1 << "/" << NHorz << std::endl << std::flush;
        cv::waitKey(10);
    }
    for(int i=0; i<NVert; i++){
        cv::Mat pattern = codecVert.getEncodingPattern(i);
        pattern = pattern.t();
        pattern = cv::repeat(pattern, screenResY/pattern.rows, screenResX/pattern.cols);
        cv::imshow("Pattern", pattern);
        projector.displayTexture(pattern.ptr(), pattern.cols, pattern.rows);
        // Treat pattern as a pseudo captured frame
        std::vector<cv::Mat> channels;
        cv::split(pattern, channels);
        codecVert.setFrame(i, channels[0]);
        std::cout << "Horizontal Pattern " << i+1 << "/" << NVert << std::endl << std::flush;
        cv::waitKey(10);
    }

    // Reconstruct up image
    cv::Mat up, mask, shading;
    codecHorz.decodeFrames(up, mask, shading);

    // Reconstruct vp image
    cv::Mat vp;
    codecVert.decodeFrames(vp, mask, shading);

    cv::namedWindow("up");
    cv::imshow("up", up/screenResX);
    cv::namedWindow("vp");
    cv::imshow("vp", vp/screenResY);
    cv::namedWindow("mask");
    cv::imshow("mask", mask);
    cv::namedWindow("shading");
    cv::imshow("shading", shading);

    cv::waitKey(0);
}

