#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include <opencv2/opencv.hpp>
#include "CalibrationData.h"

using namespace std;

class CalibratorObserver {
public:
    virtual void newSequenceResult(cv::Mat img, unsigned int idx, bool success) = 0;
};

class Calibrator {
    public:
        Calibrator(unsigned int _screenCols, unsigned int _screenRows) : observer(0), screenCols(_screenCols), screenRows(_screenRows), N(0){}
        virtual unsigned int getNPatterns(){return N;}
        virtual cv::Mat getCalibrationPattern(unsigned int depth){return patterns[depth];}
        virtual void addFrameSequence(std::vector<cv::Mat> frameSeq){frameSeqs.push_back(frameSeq);}
        virtual void reset(){frameSeqs.clear();}
        virtual CalibrationData calibrate(const int checkerSize, const int checkerRows, const int checkerCols) = 0;
        virtual ~Calibrator(){}
        void setObserver(CalibratorObserver* newObserver){observer = newObserver;}
        
    protected:
        void newSequenceResult(cv::Mat img, unsigned int idx, bool success){
            if (observer)
                observer->newSequenceResult(img, idx, success);
        }
        
        CalibratorObserver* observer;
        unsigned int screenCols, screenRows;
        unsigned int N;
        vector<cv::Mat> patterns;
        vector< vector<cv::Mat> > frameSeqs;
};

#endif // CALIBRATOR_H
