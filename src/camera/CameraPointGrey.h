#ifndef CameraPointGrey_H
#define CameraPointGrey_H

#include "Camera.h"
#include <FlyCapture2.h>

using namespace std;

class CameraPointGrey : public Camera {
    public:
        // Static methods
        static vector<CameraInfo> getCameraList();
        // Interface function
        CameraPointGrey(unsigned int camNum, CameraTriggerMode triggerMode);
        CameraSettings getCameraSettings();
        void setCameraSettings(CameraSettings);
        void startCapture();
        void stopCapture();
        CameraFrame getFrame();
        size_t getFrameSizeBytes();
        size_t getFrameWidth();
        size_t getFrameHeight();
        ~CameraPointGrey();
    private:
        FlyCapture2::Camera cam;
};

#endif
