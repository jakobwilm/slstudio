#ifndef CAMERAXIMEA_H
#define CAMERAXIMEA_H

#include "Camera.h"

// XIMEA specific type
typedef void* HANDLE;

class CameraXIMEA : public Camera {
    public:
        // Static methods
        static std::vector<CameraInfo> getCameraList();
        // Interface function
        CameraXIMEA(unsigned int camNum, CameraTriggerMode triggerMode);
        CameraSettings getCameraSettings();
        void setCameraSettings(CameraSettings);
        void startCapture();
        void stopCapture();
        CameraFrame getFrame();
        size_t getFrameSizeBytes();
        size_t getFrameWidth();
        size_t getFrameHeight();
        ~CameraXIMEA();
    private:
        HANDLE camera;
        int stat;
};

#endif // CAMERAXIMEA_H
