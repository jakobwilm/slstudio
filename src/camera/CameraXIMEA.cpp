#include "CameraXIMEA.h"
#include <cstdio>

// Note: library headers conflict with IDS imaging headers
#include <xiApi.h>

#define HandleResult(res,place) if (res!=XI_OK) {printf("CameraXIMEA: Error at %s (%d)\n",place,res); fflush(stdout);}

std::vector<CameraInfo> CameraXIMEA::getCameraList(){

    XI_RETURN stat = XI_OK;
    DWORD numCams;
    stat = xiGetNumberDevices(&numCams);
    HandleResult(stat, "xiGetNumberDevices");

    std::vector<CameraInfo> ret(numCams);
    for(unsigned int i=0; i<numCams; i++){
        CameraInfo info;
        info.vendor = "Ximea";
        char name[20];
        xiGetDeviceInfoString(i, XI_PRM_DEVICE_NAME, name, 20);
        info.model = name;
        info.busID = i;
        ret[i] = info;
    }
    return ret;
}

CameraXIMEA::CameraXIMEA(unsigned int camNum, CameraTriggerMode triggerMode) : Camera(triggerMode), camera(NULL){

    // Set debugging level
    xiSetParamInt(0, XI_PRM_DEBUG_LEVEL, XI_DL_FATAL);

    // Disable auto bandwidth determination (takes some seconds in initialization)
    xiSetParamInt(0, XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_OFF);

    // Retrieve a handle to the camera device
    stat = xiOpenDevice(camNum, &camera);
    HandleResult(stat,"xiOpenDevice");

    // Configure unsafe buffers (prevents old buffers, memory leak)
    xiSetParamInt(camera, XI_PRM_BUFFER_POLICY, XI_BP_UNSAFE);

//    // Output frame signal
//    xiSetParamInt(camera, XI_PRM_GPO_SELECTOR, 1);
//    xiSetParamInt(camera, XI_PRM_GPO_MODE, XI_GPO_ON);

    // Configure buffer size
//    stat = xiSetParamInt(camera, XI_PRM_ACQ_BUFFER_SIZE, 128*1024);
//    HandleResult(stat,"xiSetParam (XI_PRM_ACQ_BUFFER_SIZE)");
//    stat = xiSetParamInt(camera, XI_PRM_BUFFERS_QUEUE_SIZE, 10);
//    HandleResult(stat,"xiSetParam (XI_PRM_BUFFERS_QUEUE_SIZE)");

    // Configure queue mode (0 = next frame in queue, 1 = most recent frame)
    stat = xiSetParamInt(camera, XI_PRM_RECENT_FRAME, 0);
    HandleResult(stat,"xiSetParam (XI_PRM_RECENT_FRAME)");

    // Configure image type
    stat = xiSetParamInt(camera, XI_PRM_IMAGE_DATA_FORMAT, XI_MONO8);
    HandleResult(stat,"xiSetParam (XI_PRM_IMAGE_DATA_FORMAT)");

    // Configure input pin 1 as trigger input
    xiSetParamInt(camera, XI_PRM_GPI_SELECTOR, 1);
    stat = xiSetParamInt(camera, XI_PRM_GPI_MODE, XI_GPI_TRIGGER);
    HandleResult(stat,"xiSetParam (XI_PRM_GPI_MODE)");

//    // Downsample to half size
//    stat = xiSetParamInt(camera, XI_PRM_DOWNSAMPLING_TYPE, XI_SKIPPING);
//    HandleResult(stat,"xiSetParam (XI_PRM_DOWNSAMPLING_TYPE)");
//    stat = xiSetParamInt(camera, XI_PRM_DOWNSAMPLING, 2);
//    HandleResult(stat,"xiSetParam (XI_PRM_DOWNSAMPLING)");

//    // Configure frame rate
//    stat = xiSetParamFloat(camera, XI_PRM_FRAMERATE, 10);
//    HandleResult(stat,"xiSetParam (XI_PRM_FRAMERATE)");

    // Define ROI
    stat = xiSetParamInt(camera, XI_PRM_WIDTH, 640);
    HandleResult(stat,"xiSetParam (XI_PRM_WIDTH)");
    stat = xiSetParamInt(camera, XI_PRM_HEIGHT, 512);
    HandleResult(stat,"xiSetParam (XI_PRM_HEIGHT)");
    stat = xiSetParamInt(camera, XI_PRM_OFFSET_X, 320);
    HandleResult(stat,"xiSetParam (XI_PRM_OFFSET_X)");
    stat = xiSetParamInt(camera, XI_PRM_OFFSET_Y, 256);
    HandleResult(stat,"xiSetParam (XI_PRM_OFFSET_Y)");

    // Setting reasonable default settings
    xiSetParamFloat(camera, XI_PRM_GAMMAY, 1.0);
    xiSetParamInt(camera, XI_PRM_EXPOSURE, 16666); //us
    xiSetParamFloat(camera, XI_PRM_GAIN, 0);

}

CameraSettings CameraXIMEA::getCameraSettings(){

    CameraSettings settings;

    int shutter;
    xiGetParamInt(camera, XI_PRM_EXPOSURE, &shutter);
    settings.shutter = shutter/1000.0; // from us to ms
    xiGetParamFloat(camera, XI_PRM_GAIN, &settings.gain);

    return settings;
}

void CameraXIMEA::setCameraSettings(CameraSettings settings){

    // Set shutter (in us)
    xiSetParamInt(camera, XI_PRM_EXPOSURE, settings.shutter*1000);
    // Set gain (in dB)
    xiSetParamFloat(camera, XI_PRM_GAIN, settings.gain);

    std::cout << "Setting camera parameters:" << std::endl
              << "Shutter: " << settings.shutter << " ms" << std::endl
              << "Gain: " << settings.gain << " dB" << std::endl;
}

void CameraXIMEA::startCapture(){

    if(triggerMode == triggerModeHardware){
        // Configure for hardware trigger
        stat = xiSetParamInt(camera, XI_PRM_TRG_SOURCE, XI_TRG_EDGE_RISING);
        HandleResult(stat,"xiSetParam (XI_PRM_TRG_SOURCE)");
    } else if(triggerMode == triggerModeSoftware){
        // Configure for software trigger (for getSingleFrame())
        stat = xiSetParamInt(camera, XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE);
        HandleResult(stat,"xiSetParam (XI_PRM_TRG_SOURCE)");
    }

    // Start aquistion
    stat = xiStartAcquisition(camera);
    HandleResult(stat,"xiStartAcquisition");

    capturing = true;

}

void CameraXIMEA::stopCapture(){

    if(!capturing){
        std::cerr << "CameraXIMEA: not capturing!" << std::endl;
        return;
    }

}

CameraFrame CameraXIMEA::getFrame(){

    // Create single image buffer
    XI_IMG image;
    image.size = SIZE_XI_IMG_V2; // must be initialized
    image.bp = NULL;
    image.bp_size = 0;

    if(triggerMode == triggerModeSoftware){
        // Fire software trigger
        stat = xiSetParamInt(camera, XI_PRM_TRG_SOFTWARE, 0);
        HandleResult(stat,"xiSetParam (XI_PRM_TRG_SOFTWARE)");

        // Retrieve image from camera
        stat = xiGetImage(camera, 1000, &image);
        HandleResult(stat,"xiGetImage");
    } else {

        // Retrieve image from camera
        stat = xiGetImage(camera, 50, &image);
        HandleResult(stat,"xiGetImage");
    }

    // Empty buffer
    while(xiGetImage(camera, 1, &image) == XI_OK){
        std::cerr << "drop!" << std::endl;
        continue;
    }

    CameraFrame frame;
    frame.height = image.height;
    frame.width = image.width;
    frame.memory = (unsigned char*)image.bp;
    frame.timeStamp = image.tsUSec;
    frame.sizeBytes = image.bp_size;

    return frame;
}


size_t CameraXIMEA::getFrameSizeBytes(){
    return 0;
}

size_t CameraXIMEA::getFrameWidth(){
    int w;
    xiGetParamInt(camera, XI_PRM_WIDTH, &w);

    return w;
}

size_t CameraXIMEA::getFrameHeight(){
    int h;
    xiGetParamInt(camera, XI_PRM_HEIGHT, &h);

    return h;
}

CameraXIMEA::~CameraXIMEA(){

    if(capturing){
        // Stop acquisition
        stat = xiStopAcquisition(camera);
        HandleResult(stat,"xiStopAcquisition");
    }

    // Close device
    xiCloseDevice(camera);
}


