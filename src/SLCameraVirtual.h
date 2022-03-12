/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#ifndef SLCAMERAVIRTUAL_H
#define SLCAMERAVIRTUAL_H

#include "Camera.h"
#include "Codec.h"

// Virtual Camera Implementation
class SLCameraVirtual : public Camera {
    public:
        // Static methods
        static std::vector<CameraInfo> getCameraList();
        // Interface function
        SLCameraVirtual(unsigned int, CameraTriggerMode triggerMode);
        CameraSettings getCameraSettings();
        void setCameraSettings(CameraSettings){}
        void startCapture();
        void stopCapture();
        CameraFrame getFrame();
        size_t getFrameSizeBytes();
        size_t getFrameWidth();
        size_t getFrameHeight();
        ~SLCameraVirtual();
    private:
        unsigned int frameWidth, frameHeight;
        Encoder *encoder;
        unsigned long counter;
        cv::Mat currentBuffer;
};

#endif
