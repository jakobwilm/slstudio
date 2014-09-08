#ifndef CAMERAIIDC_H
#define CAMERAIIDC_H

#include "Camera.h"
#include <dc1394/dc1394.h>

using namespace std;

class CameraIIDC : public Camera {
    public:
        // Static methods
        static vector<CameraInfo> getCameraList();
        // Interface function
        CameraIIDC(unsigned int camNum, CameraTriggerMode triggerMode);
        CameraSettings getCameraSettings();
        void setCameraSettings(CameraSettings);
        void startCapture();
        void stopCapture();
        CameraFrame getFrame();
        size_t getFrameSizeBytes();
        size_t getFrameWidth();
        size_t getFrameHeight();
        ~CameraIIDC();
    private:
        dc1394_t *context;
        dc1394camera_t *cam;
        dc1394video_mode_t video_mode;
        dc1394video_frame_t *currentFrame;
        void flushBuffer();
};

#endif
