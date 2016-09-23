#include "SLScanWorker.h"

#include <QCoreApplication>
#include <QSettings>
#include <QtTest/QTest>
#include <QTime>

#include <iostream>

#include <opencv2/opencv.hpp>
#include "cvtools.h"

#include "CodecPhaseShift3.h"
#include "CodecPhaseShift3Unwrap.h"
#include "CodecPhaseShift4.h"
#include "CodecPhaseShift2x3.h"
#include "CodecPhaseShiftNStep.h"
#include "CodecPhaseShift3FastWrap.h"
#include "CodecPhaseShift2p1.h"
#include "CodecPhaseShiftDescatter.h"
#include "CodecPhaseShiftModulated.h"
#include "CodecPhaseShiftMicro.h"
#include "CodecFastRatio.h"
#include "CodecGrayCode.h"
#include "Projector.h"

#include "SLProjectorVirtual.h"
#include "SLCameraVirtual.h"
#include "SLPointCloudWidget.h"

void SLScanWorker::setup(){

    QSettings settings("SLStudio");

    // Read trigger configuration
    QString sTriggerMode = settings.value("trigger/mode", "Hardware").toString();
    if(sTriggerMode == "hardware")
        triggerMode = triggerModeHardware;
    else if(sTriggerMode == "software")
        triggerMode = triggerModeSoftware;
    else
        std::cerr << "SLScanWorker: invalid trigger mode " << sTriggerMode.toStdString() << std::endl;

    // Create camera
    int iNum = settings.value("camera/interfaceNumber", -1).toInt();
    int cNum = settings.value("camera/cameraNumber", -1).toInt();
    if(iNum != -1)
        camera = Camera::NewCamera(iNum,cNum,triggerMode);
    else
        camera = new SLCameraVirtual(cNum,triggerMode);

    // Set camera settings
    CameraSettings camSettings;
    camSettings.shutter = settings.value("camera/shutter", 16.666).toFloat();
    camSettings.gain = 0.0;
    camera->setCameraSettings(camSettings);

    // Initialize projector
    int screenNum = settings.value("projector/screenNumber", -1).toInt();
    if(screenNum == -1)
        projector = new SLProjectorVirtual(screenNum);
    else if(screenNum >= 0)
        projector = Projector::NewProjector(projectorTypeOpenGL, screenNum);
    else if(screenNum == -2)
        projector = Projector::NewProjector(projectorTypeLC3000);
    else if(screenNum == -3)
        projector = Projector::NewProjector(projectorTypeLC4500);
    else if(screenNum == -4)
        projector = Projector::NewProjector(projectorTypeQtGL);
    else
        std::cerr << "SLScanWorker: invalid projector id " << screenNum << std::endl;

    // Initialize encoder
    bool diamondPattern = settings.value("projector/diamondPattern", false).toBool();
    QString patternMode = settings.value("pattern/mode", "CodecPhaseShift3").toString();

    unsigned int screenResX, screenResY;
    projector->getScreenRes(&screenResX, &screenResY);

    // Unique number of rows and columns
    unsigned int screenCols, screenRows;
    if(diamondPattern){
        screenCols = 2*screenResX;
        screenRows = screenResY;
    } else {
        screenCols = screenResX;
        screenRows = screenResY;
    }

    CodecDir dir = (CodecDir)settings.value("pattern/direction", CodecDirHorizontal).toInt();
    if(dir == CodecDirNone)
        std::cerr << "SLScanWorker: invalid coding direction " << std::endl;

    if(patternMode == "CodecPhaseShift3")
        encoder = new EncoderPhaseShift3(screenCols, screenRows, dir);
    else if(patternMode == "CodecPhaseShift4")
        encoder = new EncoderPhaseShift4(screenCols, screenRows, dir);
    else if(patternMode == "CodecPhaseShift2x3")
        encoder = new EncoderPhaseShift2x3(screenCols, screenRows, dir);
    else if(patternMode == "CodecPhaseShift3Unwrap")
        encoder = new EncoderPhaseShift3Unwrap(screenCols, screenRows, dir);
    else if(patternMode == "CodecPhaseShiftNStep")
        encoder = new EncoderPhaseShiftNStep(screenCols, screenRows, dir);
    else if(patternMode == "CodecPhaseShift3FastWrap")
        encoder = new EncoderPhaseShift3FastWrap(screenCols, screenRows, dir);
    else if(patternMode == "CodecPhaseShift2p1")
        encoder = new EncoderPhaseShift2p1(screenCols, screenRows, dir);
    else if(patternMode == "CodecPhaseShiftDescatter")
        encoder = new EncoderPhaseShiftDescatter(screenCols, screenRows, dir);
    else if(patternMode == "CodecPhaseShiftModulated")
        encoder = new EncoderPhaseShiftModulated(screenCols, screenRows, dir);
    else if(patternMode == "CodecPhaseShiftMicro")
        encoder = new EncoderPhaseShiftMicro(screenCols, screenRows, dir);
    else if(patternMode == "CodecFastRatio")
        encoder = new EncoderFastRatio(screenCols, screenRows, dir);
    else if(patternMode == "CodecGrayCode")
        encoder = new EncoderGrayCode(screenCols, screenRows, dir);
    else
        std::cerr << "SLScanWorker: invalid pattern mode " << patternMode.toStdString() << std::endl;


    // Lens correction and upload patterns to projector/GPU
    CalibrationData calibration;
    calibration.load("calibration.xml");

    cv::Mat map1, map2;
    cv::Size mapSize = cv::Size(screenCols, screenRows);
    cvtools::initDistortMap(calibration.Kp, calibration.kp, mapSize, map1, map2);

    // Upload patterns to projector/GPU
    for(unsigned int i=0; i<encoder->getNPatterns(); i++){
        cv::Mat pattern = encoder->getEncodingPattern(i);

        // general repmat
        pattern = cv::repeat(pattern, screenRows/pattern.rows + 1, screenCols/pattern.cols + 1);
        pattern = pattern(cv::Range(0, screenRows), cv::Range(0, screenCols));

        // correct for lens distortion
        cv::remap(pattern, pattern, map1, map2, CV_INTER_CUBIC);

        if(diamondPattern)
            pattern=cvtools::diamondDownsample(pattern);

        if (!pattern.isContinuous()) {
            pattern = pattern.clone();
        }

        projector->setPattern(i, pattern.ptr(), pattern.cols, pattern.rows);

        //cv::imwrite(cv::format("pat_%d.png", i), pattern);
    }

//    // Upload patterns to projector/GPU without lens correction
//    for(unsigned int i=0; i<encoder->getNPatterns(); i++){
//        cv::Mat pattern = encoder->getEncodingPattern(i);
//        if(diamondPattern){
//            // general repmat
//            pattern = cv::repeat(pattern, screenRows/pattern.rows+1, screenCols/pattern.cols+1);
//            pattern = pattern(cv::Range(0, screenRows), cv::Range(0, screenCols));
//            pattern = cvtools::diamondDownsample(pattern);
//        }
//        projector->setPattern(i, pattern.ptr(), pattern.cols, pattern.rows);
//    }

    // Read aquisition mode
    QString sAquisition = settings.value("aquisition").toString();
    if(sAquisition == "continuous")
        aquisition = aquisitionContinuous;
    else if(sAquisition == "single")
        aquisition = aquisitionSingle;
    else
        std::cerr << "SLScanWorker: invalid aquisition mode " << sAquisition.toStdString() << std::endl;

    writeToDisk = settings.value("writeToDisk/frames", false).toBool();

}

void SLScanWorker::doWork(){

    // State variable
    isWorking = true;

    unsigned long k = 0;

    std::cout << "Starting capture!" << std::endl;
    camera->startCapture();

    unsigned int N = encoder->getNPatterns();

    QSettings settings("SLStudio");
    unsigned int shift = settings.value("trigger/shift", "0").toInt();
    unsigned int delay = settings.value("trigger/delay", "100").toInt();

    QTime time; time.start();

    // Processing loop
    do {

        std::vector<cv::Mat> frameSeq(N);
        bool success = true;

        time.restart();

        // Acquire patterns
        for(unsigned int i=0; i<N; i++){

            // Project coded pattern
            projector->displayPattern(i);

            if(triggerMode == triggerModeSoftware){
                // Wait one frame period to rotate projector frame buffer
                QTest::qSleep(delay);
            } else {
                // Wait a few milliseconds to allow camera to get ready
                QTest::qSleep(1);
            }
            CameraFrame frame;
            frame = camera->getFrame();


            if(!frame.memory){
                std::cerr << "SLScanWorker: missed frame!" << std::endl;
                success = false;
            }

            // Create 8 bit OpenCV matrix
            cv::Mat frameCV(frame.height, frame.width, CV_8U, frame.memory);
            frameCV = frameCV.clone();

            if(triggerMode == triggerModeHardware)
                frameSeq[(i+N-shift)%N] = frameCV;
            else
                frameSeq[i] = frameCV;

        }

        float sequenceTime = time.restart();
        cout << "Scan worker " << sequenceTime << "ms" << std::endl;

//        // Check for missed frames
//        if((triggerMode == triggerModeHardware) & (sequenceTime > N*18))
//            success = false;

//        if(!success){
//            std::cerr << "SLScanWorker: missed sequence!" << std::endl;
//            continue;
//        }

        // Write frames to disk if desired
        if(writeToDisk){
                for(int i=0; i<frameSeq.size(); i++){
                    QString filename = QString("frameSeq_%1.bmp").arg(i, 2, 10, QChar('0'));
                    cv::imwrite(filename.toStdString(), frameSeq[i]);

                }
        }

        // Pass frame sequence to decoder
        emit newFrameSeq(frameSeq);

        // Calculate and show histogram of sumimage
        float range[] = {0, 255};
        const float* histRange = {range};
        int histSize = 256;
        cv::Mat histogram;
        cv::Mat frameSeqArr[] = {frameSeq[0], frameSeq[1], frameSeq[2]};
        const int channels[] = {0,1,2};
        cv::calcHist(frameSeqArr, 3, channels, cv::Mat(), histogram, 1, &histSize, &histRange);
        //emit hist("Histogram", histogram, 100, 50);
        cv::Mat histogramImage = cvtools::histimage(histogram);
        emit showHistogram(histogramImage);

        // Increase iteration counter
        k += 1;

        // Process events to e.g. check for exit flag
        QCoreApplication::processEvents();

    } while (isWorking && (aquisition == aquisitionContinuous));

    if(triggerMode == triggerModeHardware)
        camera->stopCapture();

    // Emit message to e.g. initiate thread break down
    emit finished();
}

void SLScanWorker::stopWorking(){
    isWorking = false;
}

SLScanWorker::~SLScanWorker(){

    delete camera;
    delete projector;

}
