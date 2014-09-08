#pragma once
#pragma warning(disable:4996)
#ifndef CAMERAIDSIMAGING_H
#define CAMERAIDSIMAGING_H

// IDS specific type
#ifdef WIN32
    typedef unsigned long HIDS;
    typedef void * HANDLE;
#else
    typedef unsigned int HIDS;
#endif

#include "Camera.h"

class CameraIDSImaging : public Camera {
    public:
        // Static methods
        static std::vector<CameraInfo> getCameraList();
        // Interface function
        CameraIDSImaging(unsigned int camNum, CameraTriggerMode triggerMode);
        CameraSettings getCameraSettings();
        void setCameraSettings(CameraSettings);
        void startCapture();
        void stopCapture();
        CameraFrame getFrame();
        size_t getFrameSizeBytes();
        size_t getFrameWidth();
        size_t getFrameHeight();
        ~CameraIDSImaging();
    private:
        HIDS camera;
        unsigned int frameWidth, frameHeight;
        char* frameMemory; // char pointers
        int memoryID;
        #ifdef WIN32
            HANDLE hEvent;
        #endif
};

#endif
