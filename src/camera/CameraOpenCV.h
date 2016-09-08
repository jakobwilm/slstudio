/*
 *  Author: Benjamin Langmann (b.langmann@gmx.de)
 *  Date: 2016
 */

#ifndef CAMERAOPENCV
#define CAMERAOPENCV

#include "Camera.h"
#include <opencv2/opencv.hpp>

using namespace std;

class CameraOpenCV : public Camera {
    public:
        // Static methods
        static vector<CameraInfo> getCameraList();
        // Interface function
        CameraOpenCV(unsigned int camNum);
        CameraSettings getCameraSettings();
        void setCameraSettings(CameraSettings);
        void startCapture();
        void stopCapture();
        CameraFrame getFrame();
        size_t getFrameSizeBytes();
        size_t getFrameWidth();
        size_t getFrameHeight();
        ~CameraOpenCV();

    private:
        size_t m_grabTimeNS;
        size_t m_devNum;
        size_t m_bytes;
        cv::VideoCapture m_videoCap;
};

#endif // CAMERAOPENCV

