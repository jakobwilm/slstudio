#include "CameraPointGrey.h"
#include <cstring>

void PrintError(FlyCapture2::Error error){
    error.PrintErrorTrace();
}

vector<CameraInfo> CameraPointGrey::getCameraList(){
    
    FlyCapture2::Error error;

    FlyCapture2::BusManager busManager;
    unsigned int numCameras;
    error = busManager.GetNumOfCameras(&numCameras);

    vector<CameraInfo> ret;

    if (error != FlyCapture2::PGRERROR_OK){
        PrintError(error);
        return ret;
    }

    for (unsigned int i=0; i < numCameras; i++){
        FlyCapture2::PGRGuid guid;
        error = busManager.GetCameraFromIndex(i, &guid);
        if (error != FlyCapture2::PGRERROR_OK)
            PrintError(error);

        // Connect to camera
        FlyCapture2::Camera cam;
        error = cam.Connect(&guid);
        if (error != FlyCapture2::PGRERROR_OK)
            PrintError( error );

        // Get the camera information
        FlyCapture2::CameraInfo camInfo;
        error = cam.GetCameraInfo(&camInfo);
        if (error != FlyCapture2::PGRERROR_OK)
            PrintError( error );

        CameraInfo camera;
        camera.busID = camInfo.nodeNumber;
        camera.model = camInfo.modelName;
        camera.vendor = "Point Grey Research";

        ret.push_back(camera);
    }

    return ret;
}

CameraPointGrey::CameraPointGrey(unsigned int camNum, CameraTriggerMode triggerMode) : Camera(triggerMode){

    FlyCapture2::Error error;

    // Connect to camera
    FlyCapture2::BusManager busManager;
    FlyCapture2::PGRGuid camGuid;
    busManager.GetCameraFromIndex(camNum, &camGuid);
    error = cam.Connect(&camGuid);
    if (error != FlyCapture2::PGRERROR_OK)
        PrintError(error);

    // Configure video mode and frame rate
    FlyCapture2::VideoMode videoMode = FlyCapture2::VIDEOMODE_640x480Y16;
    FlyCapture2::FrameRate frameRate = FlyCapture2::FRAMERATE_15;
    cam.SetVideoModeAndFrameRate(videoMode, frameRate);

    // Get the camera information
    FlyCapture2::CameraInfo camInfo;
    error = cam.GetCameraInfo(&camInfo);
    if (error != FlyCapture2::PGRERROR_OK)
        PrintError(error);

    std::cout << camInfo.vendorName << "  " << camInfo.modelName << "  " << camInfo.serialNumber << std::endl;

    // Set reasonable default settings
    CameraSettings settings;
    //settings.shutter = 8.33;
    settings.shutter = 33.33;
    settings.gain = 0.0;
    this->setCameraSettings(settings);

    // Start isochronous image transfer
    error = cam.StartCapture();
    if (error != FlyCapture2::PGRERROR_OK)
        PrintError(error);

    return;
}

CameraSettings CameraPointGrey::getCameraSettings(){

    FlyCapture2::Property property;

    // Get settings:
    CameraSettings settings;

    property.type = FlyCapture2::SHUTTER;
    cam.GetProperty(&property);
    settings.shutter = property.absValue;

    property.type = FlyCapture2::GAIN;
    cam.GetProperty(&property);
    settings.gain = property.absValue;

    return settings;
}

void CameraPointGrey::setCameraSettings(CameraSettings settings){

    FlyCapture2::Property property;
    property.onOff = true;
    property.absControl = true;

    property.type = FlyCapture2::SHUTTER;
    property.absValue = settings.shutter;
    cam.SetProperty(&property);

    property.type = FlyCapture2::GAIN;
    property.absValue = settings.gain;
    cam.SetProperty(&property);

}

void CameraPointGrey::startCapture(){

    FlyCapture2::Error error;\

    CameraSettings settings = this->getCameraSettings();
    std::cout << "\tShutter: " << settings.shutter << "ms" << std::endl;
    std::cout << "\tGain: " << settings.gain << "dB" << std::endl;

    if(triggerMode == triggerModeHardware){
        // Configure for hardware trigger
        FlyCapture2::TriggerMode triggerMode;
        triggerMode.onOff = true;
        triggerMode.polarity = 0;
        triggerMode.source = 0;
        triggerMode.mode = 14;
        error = cam.SetTriggerMode(&triggerMode);
        if (error != FlyCapture2::PGRERROR_OK)
            PrintError(error);

        error = cam.StartCapture();
        if (error != FlyCapture2::PGRERROR_OK)
            PrintError(error);

    } else if(triggerMode == triggerModeSoftware){
        // Configure software trigger
        FlyCapture2::TriggerMode triggerMode;
        triggerMode.onOff = true;
        triggerMode.polarity = 0;
        triggerMode.source = 7; // software
        triggerMode.mode = 0;
        error = cam.SetTriggerMode(&triggerMode);
        if (error != FlyCapture2::PGRERROR_OK)
            PrintError(error);
    }

    // Set the trigger timeout to 1000 ms
    FlyCapture2::FC2Config config;
    config.grabTimeout = 1000;
    error = cam.SetConfiguration(&config);
    if (error != FlyCapture2::PGRERROR_OK)
        PrintError(error);

    capturing = true;
}

void CameraPointGrey::stopCapture(){

    FlyCapture2::Error error = cam.StopCapture();
    if (error != FlyCapture2::PGRERROR_OK)
        PrintError(error);

    capturing = false;
}

CameraFrame CameraPointGrey::getFrame(){

    FlyCapture2::Error error;

    if(triggerMode == triggerModeSoftware){
        // Fire software trigger
        // broadcasting not supported on some platforms
        cam.FireSoftwareTrigger(false);
    }

    // Retrieve the image
    FlyCapture2::Image rawImage;
    error = cam.RetrieveBuffer(&rawImage);
    if (error != FlyCapture2::PGRERROR_OK)
        PrintError(error);

    CameraFrame frame;

    frame.timeStamp = rawImage.GetTimeStamp().cycleCount;
    frame.height = rawImage.GetRows();
    frame.width = rawImage.GetCols();
    frame.memory = rawImage.GetData();

    return frame;
}


size_t CameraPointGrey::getFrameSizeBytes(){
    
    return 0;
}

size_t CameraPointGrey::getFrameWidth(){

    // How do we poll this from the camera?
    return 640;

}

size_t CameraPointGrey::getFrameHeight(){

    // How do we poll this from the camera?
    return 480;

}


CameraPointGrey::~CameraPointGrey(){

    if(capturing && triggerMode == triggerModeHardware){
        // Stop camera transmission
        cam.StopCapture();
    }

    // Gracefulle destruct the camera
    cam.Disconnect();

}



