#ifndef SLCameraWorker_H
#define SLCameraWorker_H

#include <QObject>
#include <opencv2/opencv.hpp>
#include "Camera.h"

class SLCameraWorker : public QObject{
    Q_OBJECT

    public:
        SLCameraWorker(): _isWorking(false), camera(NULL) {}
        bool isWorking(){return _isWorking;}
        ~SLCameraWorker();
    public slots:
        void setup(unsigned iNum, unsigned cNum);
        void doWork();
        void stopWorking(){_isWorking = false;}
    signals:
        void newFrameSeq(std::vector<cv::Mat> frameSeq);
        void finished();
    private:
        bool _isWorking;
        Camera *camera;
};

#endif // SLCameraWorker_H
