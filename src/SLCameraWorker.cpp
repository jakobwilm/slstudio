#include "SLCameraWorker.h"

#include "Camera.h"
#include <QApplication>

void SLCameraWorker::setup(unsigned iNum, unsigned cNum){

    // Initialize camera
    camera = Camera::NewCamera(iNum, cNum);

    unsigned int frameWidth, frameHeight;
    camera->getFrameWidthHeight(&frameWidth, &frameHeight);

    std::cout << "SLCameraWorker: opened camera " << frameWidth << " x " << frameHeight << std::endl << std::flush;
}

void SLCameraWorker::doWork(){

    _isWorking = true;

    camera->startCapture();

    while(_isWorking){

        std::vector<cv::Mat> frameSeq;

        for(unsigned int i=0; i<3; i++){
            // Run camera in continous mode
            CameraFrame frame = camera->lockFrame();
            cv::Mat frameCV(frame.height, frame.width, CV_8U, frame.memory);
            frameCV = frameCV.clone();
            camera->unlockFrame();
            frameSeq.push_back(frameCV);
        }

        // Emit new frame sequence signal
        emit newFrameSeq(frameSeq);

        QApplication::processEvents();
    }

    camera->stopCapture();

    // Emit finished signal
    emit finished();
}

SLCameraWorker::~SLCameraWorker(){
    delete camera;
}
