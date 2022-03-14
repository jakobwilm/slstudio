#include "SLCameraVirtual.h"

#include <QCollator>
#include <QFileDialog>
#include <QSettings>
#include <QStandardPaths>
#include <QString>
#include <cstdio>

#include "CodecFastRatio.h"
#include "CodecGrayCode.h"
#include "CodecPhaseShift2p1.h"
#include "CodecPhaseShift2x3.h"
#include "CodecPhaseShift3.h"
#include "CodecPhaseShift3FastWrap.h"
#include "CodecPhaseShift3Unwrap.h"
#include "CodecPhaseShift4.h"
#include "CodecPhaseShiftMicro.h"
#include "CodecPhaseShiftNStep.h"

std::vector<CameraInfo> SLCameraVirtual::getCameraList() {

  CameraInfo info;
  info.vendor = "SLStudio";
  info.model = "Virtual Camera";
  info.busID = 0;

  std::vector<CameraInfo> ret;
  ret.push_back(info);

  return ret;
}

SLCameraVirtual::SLCameraVirtual(unsigned int, CameraTriggerMode triggerMode)
    : Camera(triggerMode), counter(0) {}

CameraSettings SLCameraVirtual::getCameraSettings() {

  CameraSettings settings;

  settings.shutter = 0.0;
  settings.gain = 0.0;

  return settings;
}

void SLCameraVirtual::startCapture() {
  capturing = true;

  QSettings settings("SLStudio");
  QString path = settings.value("virtualCameraPath").toString();
  QDir dir(path);
  QStringList filenames = dir.entryList(QStringList("frameSeq_*.bmp"));
  QCollator collator;
  collator.setNumericMode(true);
  std::sort(filenames.begin(), filenames.end(), collator);

  patternFrames.clear();
  for (const auto &f : filenames) {
    std::cout << "loading " << QDir(path).filePath(f).toStdString()
              << std::endl;
    patternFrames.push_back(
        cv::imread(QDir(path).filePath(f).toStdString(), cv::IMREAD_GRAYSCALE));
  }

  if (patternFrames.empty()) {
    patternFrames.push_back(cv::Mat(1, 1, CV_8UC1));
  }
}

void SLCameraVirtual::stopCapture() {

  if (!capturing) {
    std::cerr << "SLCameraVirtual: not capturing!" << std::endl;
    return;
  }
}

CameraFrame SLCameraVirtual::getFrame() {

  cv::Mat frameCV = patternFrames[counter];
  counter = (counter + 1) % patternFrames.size();

  // return as CameraFrame struct
  CameraFrame frame;
  frame.height = frameCV.rows;
  frame.width = frameCV.cols;
  frame.memory = frameCV.data;
  frame.timeStamp = counter;
  frame.sizeBytes = frameCV.rows * frameCV.cols;

  return frame;
}

size_t SLCameraVirtual::getFrameSizeBytes() {
  return patternFrames[0].rows * patternFrames[0].cols;
}

size_t SLCameraVirtual::getFrameWidth() { return patternFrames[0].cols; }

size_t SLCameraVirtual::getFrameHeight() { return patternFrames[0].rows; }

SLCameraVirtual::~SLCameraVirtual() {}
