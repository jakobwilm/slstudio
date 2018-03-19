#ifndef CAMERAV4L2_H
#define CAMERAV4L2_H

#include "Camera.h"
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
// #include <dc1394/dc1394.h>

using namespace std;

#define TRUE  1
#define FALSE 0

#define  IMAGEWIDTH    640
#define  IMAGEHEIGHT   480

#define EXPOSURE_MODE_SETTING       V4L2_EXPOSURE_MANUAL  //1-> V4L2_EXPOSURE_MANUAL 3 -> V4L2_EXPOSURE_APERTURE_PRIORITY
// #define EXPOSURE_ABSOLUTE_VALUE     300

class CameraV4L2 : public Camera {
    public:
        // Static methods
        static vector<CameraInfo> getCameraList();

        // Interface function
        CameraV4L2(unsigned int camNum, CameraTriggerMode triggerMode);
        ~CameraV4L2();

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
        int fd;
        struct   v4l2_capability   cap;
        struct v4l2_fmtdesc fmtdesc;
        struct v4l2_format fmt,fmtack;
        struct v4l2_streamparm setfps;  
        struct v4l2_requestbuffers req;
        struct v4l2_buffer buf;
        enum v4l2_buf_type buf_type;
        struct v4l2_queryctrl queryctrl;
        struct v4l2_control control;

        // unsigned char frame_buffer[IMAGEWIDTH*IMAGEHEIGHT*3];
        unsigned char grey_buffer[IMAGEWIDTH*IMAGEHEIGHT];
        
        int camera_on();
        int camera_off();
        int get_ctrl(unsigned long cid);
        int set_ctrl(unsigned long cid, int value);
        int yuyv_2_grey();
        int flush_buffer();

};

#endif
