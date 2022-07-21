/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#pragma once

#include "Camera.h"
#include "Codec.h"

// Virtual Camera Implementation
class CameraVirtual : public Camera {
public:
  // Static methods
  static std::vector<CameraInfo> getCameraList();
  // Interface function
  CameraVirtual(unsigned int, CameraTriggerMode triggerMode);
  CameraSettings getCameraSettings();
  void setCameraSettings(CameraSettings) {}
  void startCapture();
  void stopCapture();
  CameraFrame getFrame();
  size_t getFrameSizeBytes();
  size_t getFrameWidth();
  size_t getFrameHeight();
  ~CameraVirtual();

private:
  std::vector<cv::Mat> patternFrames;
  unsigned long counter;
};
