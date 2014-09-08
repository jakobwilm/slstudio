#ifndef CAMERAWORKER_H
#define CAMERAWORKER_H

#include <QObject>
#include <opencv2/opencv.hpp>
#include "Camera.h"

class CameraWorker : public QObject{
    Q_OBJECT

    public:
        CameraWorker(): _isWorking(false), camera(NULL) {}
        bool isWorking(){return _isWorking;}
        ~CameraWorker();
    public slots:
        void setup(unsigned iNum, unsigned cNum);
        void doWork();
        void stopWorking(){_isWorking = false;}
    signals:
        void newFrame(cv::Mat frame);
        void finished();
    private:
        bool _isWorking;
        Camera *camera;
};

#endif // CAMERAWORKER_H
