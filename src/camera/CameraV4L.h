/*
 *  Author: Benjamin Langmann (b.langmann@gmx.de)
 *  Date: 2016
 */

#ifndef CAMERAV4L
#define CAMERAV4L

#include "Camera.h"

#include <opencv2/opencv.hpp>

using namespace std;

struct buffer {
    char   *start;
    size_t  length;
};

class CameraV4L : public Camera {
    public:
        // Static methods
        static vector<CameraInfo> getCameraList();
        // Interface function
        CameraV4L(unsigned int camNum);
        CameraSettings getCameraSettings();
        void setCameraSettings(CameraSettings);
        void startCapture();
        void stopCapture();
        CameraFrame getFrame();
        size_t getFrameSizeBytes();
        size_t getFrameWidth();
        size_t getFrameHeight();
        ~CameraV4L();

    private:
        size_t m_bytes, m_width, m_height;
        buffer* m_buffers;
        size_t m_numBuffers;
        int m_fd;
        int m_io;
        bool opened;
        cv::Mat m_lastImage;
};

#endif // CAMERAV4L

