#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <vector>

#include "slcamera_export.h"

struct CameraFrame {
    unsigned char *memory;
    unsigned int width;
    unsigned int height;
    unsigned int sizeBytes;
    unsigned int timeStamp;
    CameraFrame(): memory(NULL), width(0), height(0), sizeBytes(0), timeStamp(0){}
};

struct CameraSettings {
    float gain;
    float shutter; // [ms]
    CameraSettings(): gain(0.0), shutter(0.0) {}
};

struct CameraInfo {
    std::string vendor;
    std::string model;
    unsigned int busID;
};

enum CameraTriggerMode {
    triggerModeHardware,
    triggerModeSoftware
};

// Camera factory methods and abstract base class for camera implementations
class SLCAMERA_EXPORT Camera {

    public:
        // Static "camera factory" methods
        static std::vector< std::vector<CameraInfo> > GetInterfaceCameraList();
        static Camera* NewCamera(unsigned int interfaceNum, unsigned int camNum, CameraTriggerMode triggerMode);
        // Interface function
        Camera(CameraTriggerMode _triggerMode) : capturing(false), triggerMode(_triggerMode){}
        virtual void startCapture() = 0;
        bool isCapturing(){return capturing;}
        virtual void stopCapture() = 0;
        virtual CameraFrame getFrame() = 0;
        virtual size_t getFrameSizeBytes() = 0;
        virtual size_t getFrameWidth() = 0;
        virtual size_t getFrameHeight() = 0;
        virtual CameraSettings getCameraSettings() = 0;
        virtual void setCameraSettings(CameraSettings) = 0;
        virtual ~Camera(){}

    protected:
        bool capturing;
        CameraTriggerMode triggerMode;
};

#endif
