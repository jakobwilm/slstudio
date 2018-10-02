#ifndef CAMERACV_H
#define CAMERACV_H

#include "Camera.h"
#include <opencv2/opencv.hpp>
// #include <dc1394/dc1394.h>

using namespace std;

class CameraCV : public Camera {
    public:
        // Static methods
        static vector<CameraInfo> getCameraList();

        // Interface function
        CameraCV(unsigned int camNum, CameraTriggerMode triggerMode);
        ~CameraCV();

        CameraSettings getCameraSettings();
        void setCameraSettings(CameraSettings);
        void startCapture();
        void stopCapture();
        CameraFrame getFrame();
        size_t getFrameSizeBytes();
        size_t getFrameWidth();
        size_t getFrameHeight();
    private:
        // dc1394_t *context;
        // dc1394camera_t *cam;
        // dc1394video_mode_t video_mode;
        // dc1394video_frame_t *currentFrame;
        size_t _width;
        size_t _height;
        cv::Mat  cameraFrame;
        cv::VideoCapture* camera;
};

#endif
