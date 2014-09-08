#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include <opencv2/opencv.hpp>
#include "CalibrationData.h"

using namespace std;

class Calibrator {
    public:
        Calibrator(unsigned int _screenResX, unsigned int _screenResY, bool _diamondPattern);
        cv::Mat getProjectionImage(){return projectionImage;}
        vector<cv::Vec2f> getProjectionCoordinates(){return qp;}
        void addFramePair(cv::Mat featureFrame, cv::Mat backGroundFrame){framePairs.push_back(make_pair(featureFrame, backGroundFrame));}
        void reset(){framePairs.clear();}
        CalibrationData calibrate();
    private:
        cv::Mat projectionImage;
        vector<cv::Vec2f> qp;
        bool diamondPattern;
        unsigned int screenResX, screenResY;
        vector< pair<cv::Mat, cv::Mat> > framePairs;

};

#endif // CALIBRATOR_H
