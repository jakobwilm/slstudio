#include "CameraWorker.h"
#include <QApplication>
#include <QtTest/QTest>

void CameraWorker::setup(unsigned iNum, unsigned cNum){

    // Initialize camera
    camera = Camera::NewCamera(iNum, cNum, triggerModeHardware);
}

void CameraWorker::doWork(){

    CameraFrame frame;
    unsigned int frameWidth, frameHeight;
    frameWidth = camera->getFrameWidth();
    frameHeight = camera->getFrameHeight();

    std::cout << "CameraWorker: opened camera " << frameWidth << " x " << frameHeight << std::endl << std::flush;


    camera->startCapture();

    _isWorking = true;
    int i = 0;
    while(_isWorking){

        frame = camera->getFrame();
        //frame = camera->lockFrame();
        cv::Mat frameCV(frame.height, frame.width, CV_8U, frame.memory);
        frameCV = frameCV.clone();

        // Emit new frame
        emit newFrame(frameCV);
        //std::cout << "frame" << i << std::endl << std::flush;

        i+=1;
        QApplication::processEvents();

   }

   camera->stopCapture();

    // Emit finished signal
    emit finished();
}

CameraWorker::~CameraWorker(){
    delete camera;

}
