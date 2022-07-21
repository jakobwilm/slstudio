#include "ScanWorker.h"

#include "ProjectorFactory.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QSettings>

#include <iostream>

#include "cvtools.h"
#include <opencv2/opencv.hpp>

#include "CodecFactory.h"

#include "CameraFactory.h"
#include "CameraVirtual.h"

#include "PointCloudWidget.h"
#include "ProjectorVirtual.h"

void ScanWorker::setup() {

  QSettings settings("SLStudio");

  // Read trigger configuration
  QString sTriggerMode = settings.value("trigger/mode", "Hardware").toString();
  if (sTriggerMode == "hardware")
    triggerMode = triggerModeHardware;
  else if (sTriggerMode == "software")
    triggerMode = triggerModeSoftware;
  else
    std::cerr << "SLScanWorker: invalid trigger mode "
              << sTriggerMode.toStdString() << std::endl;

  // Create camera
  int iNum = settings.value("camera/interfaceNumber", -1).toInt();
  int cNum = settings.value("camera/cameraNumber", -1).toInt();
  if (iNum != -1)
    camera = CameraFactory::NewCamera(iNum, cNum, triggerMode);
  else
    camera = std::make_unique<CameraVirtual>(cNum, triggerMode);

  // Set camera settings
  CameraSettings camSettings;
  camSettings.shutter = settings.value("camera/shutter", 16.666).toFloat();
  camSettings.gain = 0.0;
  camera->setCameraSettings(camSettings);

  // Initialize projector
  int screenNum = settings.value("projector/screenNumber", -1).toInt();
  projector = ProjectorFactory::NewProjector(screenNum);

  if (projector == nullptr) {
    emit logMessage("SLCalibrationDialog: could not create projector.");
  }

  // Initialize encoder
  std::string codecName =
      settings.value("pattern/mode", "PhaseShift2x3").toString().toStdString();

  unsigned int screenResX, screenResY;
  projector->getScreenRes(&screenResX, &screenResY);

  CodecDir dir = static_cast<CodecDir>(
      settings.value("pattern/direction", CodecDirHorizontal).toInt());
  if (dir == CodecDirNone) {
    std::cerr << "ScanWorker: invalid coding direction " << std::endl;
  }

  if (Codecs.count(codecName) == 0) {
    std::cerr << "ScanWorker: invalid codec " << codecName << std::endl;
  }
  encoder = EncoderFactory::NewEncoder(Codecs.at(codecName), screenResX,
                                       screenResY, dir);

  if (!projector->requiresPatternUpload()) {
    // Lens correction and upload patterns to projector/GPU
    CalibrationData calibration;
    calibration.load("calibration.xml");

    cv::Mat map1, map2;
    if (settings.value("projector/correctLensDistortion", false).toBool()) {
      cv::Size mapSize = cv::Size(screenResX, screenResY);
      cvtools::initDistortMap(calibration.Kp, calibration.kp, mapSize, map1,
                              map2);
    }

    // Upload patterns to projector/GPU in full projector resolution
    std::vector<cv::Mat> patterns(encoder->getNPatterns());
    std::vector<const uchar *> patternPtrs(encoder->getNPatterns());

    for (unsigned int i = 0; i < encoder->getNPatterns(); i++) {
      patterns[i] = encoder->getEncodingPattern(i);

      // general repmat
      patterns[i] = cv::repeat(patterns[i], screenResY / patterns[i].rows + 1,
                               screenResX / patterns[i].cols + 1);
      patterns[i] =
          patterns[i](cv::Range(0, screenResY), cv::Range(0, screenResX));

      if (settings.value("projector/correctLensDistortion", false).toBool()) {
        cv::remap(patterns[i], patterns[i], map1, map2, cv::INTER_CUBIC);
      }

      patternPtrs[i] = patterns[i].data;
    }

    projector->setPatterns(patternPtrs, patterns[0].cols, patterns[0].rows);

    //    // Upload patterns to projector/GPU in compact resolution (texture)
    //    for(unsigned int i=0; i<encoder->getNPatterns(); i++){
    //        cv::Mat pattern = encoder->getEncodingPattern(i);
    //        if(diamondPattern){
    //            // general repmat
    //            pattern = cv::repeat(pattern, screenResY/pattern.rows+1,
    //            screenResX/pattern.cols+1); pattern = pattern(cv::Range(0,
    //            screenResY), cv::Range(0, screenResX)); pattern =
    //            cvtools::diamondDownsample(pattern);
    //        }
    //        projector->setPattern(i, pattern.ptr(), pattern.cols,
    //        pattern.rows);
    //    }
  }

  // Read aquisition mode
  QString sAquisition = settings.value("aquisition").toString();
  if (sAquisition == "continuous")
    aquisition = aquisitionContinuous;
  else if (sAquisition == "single")
    aquisition = aquisitionSingle;
  else
    std::cerr << "SLScanWorker: invalid aquisition mode "
              << sAquisition.toStdString() << std::endl;

  writeToDisk = settings.value("writeToDisk/frames", false).toBool();
}

void ScanWorker::doWork() {

  // State variable
  isWorking = true;

  emit logMessage("Starting capture!");
  camera->startCapture();

  unsigned int N = encoder->getNPatterns();

  QSettings settings("SLStudio");
  unsigned int shift = settings.value("trigger/shift", "0").toInt();
  unsigned int delay = settings.value("trigger/delay", "100").toInt();

  QElapsedTimer time;
  time.start();

  // Processing loop
  do {

    std::vector<cv::Mat> frameSeq(N);
    bool success = true;

    time.restart();

    // Acquire patterns
    for (unsigned int i = 0; i < N; i++) {

      // Project coded pattern
      projector->displayPattern(i);

      if (triggerMode == triggerModeSoftware) {
        // Wait one frame period to rotate projector frame buffer
        QThread::msleep(delay);
      } else {
        // Wait a few milliseconds to allow camera to get ready
        // QThread::msleep(1);
      }
      CameraFrame frame;
      frame = camera->getFrame();

      if (!frame.memory) {
        std::cerr << "SLScanWorker: missed frame!" << std::endl;
        success = false;
      }

      // If the camera provides a sequence start flag
      if (frame.flags != 0) {
        emit logMessage("reset");
        i = 0;
      }

      // Create 8 bit OpenCV matrix
      cv::Mat frameCV(frame.height, frame.width, CV_8U, frame.memory);
      frameCV = frameCV.clone();

      if (triggerMode == triggerModeHardware)
        frameSeq[(i + N - shift) % N] = frameCV;
      else
        frameSeq[i] = frameCV;
    }

    float sequenceTime = time.restart();
    emit logMessage(QString("Scan worker %1ms").arg(sequenceTime));

    //        // Check for missed frames
    //        if((triggerMode == triggerModeHardware) & (sequenceTime > N*18))
    //            success = false;

    if (!success) {
      std::cerr << "SLScanWorker: missed sequence!" << std::endl;
      continue;
    }

    // Write frames to disk if desired
    if (writeToDisk) {
      for (unsigned int i = 0; i < frameSeq.size(); i++) {
        QString filename = QString("frameSeq_%1.bmp").arg(i, 2, 10, QChar('0'));
        cv::imwrite(filename.toStdString(), frameSeq[i]);
      }
    }

    // Pass frame sequence to decoder
    emit newFrameSeq(frameSeq);

    // Calculate and show histogram of sumimage
    const std::vector<int> histSize{256};
    const std::vector<float> histRange{0.0, 256.0};
    cv::Mat histogram;
    std::vector<int> channels{0};
    //    std::iota(std::begin(channels), std::end(channels), 0);
    cv::calcHist(frameSeq, channels, cv::Mat(), histogram, histSize, histRange);
    cv::Mat histogramImage = cvtools::histimage(histogram);
    emit showHistogram(histogramImage);

    // Process events to e.g. check for exit flag
    QCoreApplication::processEvents();

  } while (isWorking && (aquisition == aquisitionContinuous));

  if (triggerMode == triggerModeHardware)
    camera->stopCapture();

  // Emit message to e.g. initiate thread break down
  emit finished();
}

void ScanWorker::stopWorking() { isWorking = false; }

ScanWorker::~ScanWorker() {}

bool ScanWorker::uploadPatterns(const Encoder *encoder, Projector *projector) {

  QSettings settings;

  unsigned int screenResX, screenResY;
  projector->getScreenRes(&screenResX, &screenResY);

  // Lens correction and upload patterns to projector/GPU
  CalibrationData calibration;
  calibration.load("calibration.xml");

  cv::Mat map1, map2;
  if (settings.value("projector/correctLensDistortion", false).toBool()) {
    cv::Size mapSize = cv::Size(screenResX, screenResY);
    cvtools::initDistortMap(calibration.Kp, calibration.kp, mapSize, map1,
                            map2);
  }

  // Upload patterns to projector/GPU in full projector resolution
  std::vector<cv::Mat> patterns(encoder->getNPatterns());
  std::vector<const uchar *> patternPtrs(encoder->getNPatterns());

  for (unsigned int i = 0; i < encoder->getNPatterns(); i++) {
    patterns[i] = encoder->getEncodingPattern(i);

    // general repmat
    patterns[i] = cv::repeat(patterns[i], screenResY / patterns[i].rows + 1,
                             screenResX / patterns[i].cols + 1);
    patterns[i] =
        patterns[i](cv::Range(0, screenResY), cv::Range(0, screenResX));

    if (settings.value("projector/correctLensDistortion", false).toBool()) {
      cv::remap(patterns[i], patterns[i], map1, map2, cv::INTER_CUBIC);
    }

    patternPtrs[i] = patterns[i].data;
  }

  projector->setPatterns(patternPtrs, patterns[0].cols, patterns[0].rows);

  return true;
}
