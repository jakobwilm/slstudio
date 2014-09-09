#include "SLCameraVirtual.h"

#include <cstdio>
#include <QString>
#include <QSettings>

#include "CodecPhaseShift3.h"
#include "CodecPhaseShift4.h"
#include "CodecPhaseShift2x3.h"
#include "CodecPhaseShift3Unwrap.h"
#include "CodecGrayCode.h"

std::vector<CameraInfo> SLCameraVirtual::getCameraList(){

    CameraInfo info;
    info.vendor = "SLStudio";
    info.model = "Virtual Camera";
    info.busID = 0;

    std::vector<CameraInfo> ret;
    ret.push_back(info);

    return ret;
}

SLCameraVirtual::SLCameraVirtual(unsigned int, CameraTriggerMode triggerMode): Camera(triggerMode), frameWidth(640), frameHeight(480), counter(0){

    QSettings settings("SLStudio");

    CodecDir dir = (CodecDir)settings.value("pattern/direction", CodecDirHorizontal).toInt();
    if(dir == CodecDirNone)
        std::cerr << "SLDecoderWorker: invalid coding direction " << std::endl;

    QString patternMode = settings.value("pattern/mode", "CodecPhaseShift3").toString();
    if(patternMode == "CodecPhaseShift3")
        encoder = new EncoderPhaseShift3(frameWidth, frameHeight, dir);
    else if(patternMode == "CodecPhaseShift4")
        encoder = new EncoderPhaseShift4(frameWidth, frameHeight, dir);
    else if(patternMode == "CodecPhaseShift2x3")
        encoder = new EncoderPhaseShift2x3(frameWidth, frameHeight, dir);
    else if(patternMode == "CodecPhaseShift3Unwrap")
        encoder = new EncoderPhaseShift3Unwrap(frameWidth, frameHeight, dir);
    else if(patternMode == "CodecGrayCode")
        encoder = new EncoderGrayCode(frameWidth, frameHeight, dir);
    else
        std::cerr << "SLCameraVirtual: invalid pattern mode " << patternMode.toStdString() << std::endl;


    currentBuffer.create(frameHeight, frameWidth, CV_8U);

    std::cout << "SLCameraVirtual: Virtual Camera Started" << std::endl;
}

CameraSettings SLCameraVirtual::getCameraSettings(){

    CameraSettings settings;

    settings.shutter = 0.0;
    settings.gain = 0.0;

    return settings;

}

void SLCameraVirtual::startCapture(){
    capturing = true;
}

void SLCameraVirtual::stopCapture(){

    if(!capturing){
        std::cerr << "SLCameraVirtual: not capturing!" << std::endl;
        return;
    }
}


CameraFrame SLCameraVirtual::getFrame(){

    unsigned int depth = counter % encoder->getNPatterns();
    cv::Mat patternCV = encoder->getEncodingPattern(depth);

    // pick out first channel
    cv::Mat patternCVChannels[3];
    cv::split(patternCV, patternCVChannels);
    patternCV = patternCVChannels[0];

    // general repmat
    cv::Mat frameCV;
    frameCV = cv::repeat(patternCV, (frameHeight+patternCV.rows-1)/patternCV.rows, (frameWidth+patternCV.cols-1)/patternCV.cols);
    frameCV = frameCV(cv::Range(0, frameHeight), cv::Range(0, frameWidth));
    frameCV = frameCV.clone();

    // add noise
    frameCV.convertTo(frameCV, CV_32F);
    cv::Mat noise(frameCV.size(), frameCV.type());
    cv::randn(noise, 0, 3);
    frameCV += noise;
    frameCV.convertTo(currentBuffer, CV_8U);
    counter++;

 //cv::imwrite("frameCV.png", frameCV);

    // return as CameraFrame struct
    CameraFrame frame;
    frame.height = currentBuffer.rows;
    frame.width = currentBuffer.cols;
    frame.memory = currentBuffer.data;
    frame.timeStamp = counter;
    frame.sizeBytes = currentBuffer.rows*currentBuffer.cols;

    return frame;
}


size_t SLCameraVirtual::getFrameSizeBytes(){

    return frameWidth*frameHeight;
}

size_t SLCameraVirtual::getFrameWidth(){
    return frameWidth;
}

size_t SLCameraVirtual::getFrameHeight(){
    return frameHeight;
}


SLCameraVirtual::~SLCameraVirtual(){
    delete encoder;
}




