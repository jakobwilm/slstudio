#include "CameraIDSImaging.h"
#include <string.h>

// Note: library header conflicts, and should only be included in cpp files
#include <uEye.h>

std::vector<CameraInfo> CameraIDSImaging::getCameraList(){

    int err;
    int pnNumCams;
    err = is_GetNumberOfCameras(&pnNumCams);
    if(err == IS_NO_SUCCESS){
        std::cerr << "Could not get number of cameras!" << std::endl;
        std::vector<CameraInfo> ret;
        return ret;
    }

    // Allocate the required camera list size
    PUEYE_CAMERA_LIST m_pCamList;
    m_pCamList = (PUEYE_CAMERA_LIST)new char[sizeof(DWORD) + pnNumCams * sizeof(UEYE_CAMERA_INFO)];
    m_pCamList->dwCount = pnNumCams;

    // Get CameraList
    err = is_GetCameraList(m_pCamList);

    std::vector<CameraInfo> ret(pnNumCams);
    for(int i=0; i<pnNumCams; i++){
        CameraInfo info;
        info.vendor = "IDS Imaging";
        info.model = m_pCamList->uci[i].Model;
        info.busID = m_pCamList->uci[i].dwCameraID;
        ret[i] = info;
    }
    return ret;
}

CameraIDSImaging::CameraIDSImaging(unsigned int camNum, CameraTriggerMode triggerMode):  Camera(triggerMode), frameWidth(0), frameHeight(0), frameMemory(NULL), memoryID(0){

    // Init Camera
    camera = (HIDS) camNum + 1;

    //open camera with null pointer, this defaults to DIB mode
    int ret = is_InitCamera(&camera, NULL);
    if(ret != IS_SUCCESS)
        std::cerr << "CameraIDSImaging: Could not open camera!" << std::endl << std::flush;

#ifndef _WIN32
    // Error reporting
    int err = is_SetErrorReport(camera, IS_ENABLE_ERR_REP);
    if (err != IS_SUCCESS)
        std::cerr << "CameraIDSImaging Error: Failed to set error reporting!";
#endif

    int image_down_scale=2;

    // Use 2x scaler to double attainable framerate
    is_SetSensorScaler(camera, IS_ENABLE_SENSOR_SCALER, image_down_scale);
    SENSORINFO sInfo;
    is_GetSensorInfo (camera, &sInfo);
    frameWidth = sInfo.nMaxWidth/image_down_scale;
    frameHeight = sInfo.nMaxHeight/image_down_scale;

//    // Set AOI to subframe
//    IS_RECT rectAOI;
//    is_AOI(camera, IS_AOI_IMAGE_GET_AOI, (void*)&rectAOI, sizeof(rectAOI));
//    // Decrease AOI a little to improve readout speed
//    rectAOI.s32Width /= 2;
//    rectAOI.s32Height /= 2;
//    rectAOI.s32Height -= 12;
//    rectAOI.s32X = rectAOI.s32Width/2;
//    rectAOI.s32Y = rectAOI.s32Height/2;
//    is_AOI(camera, IS_AOI_IMAGE_SET_AOI, (void*)&rectAOI, sizeof(rectAOI));
//    frameWidth = rectAOI.s32Width;
//    frameHeight = rectAOI.s32Height;


    // Set up 8bit monochrome color depth
    is_SetColorMode(camera, IS_CM_MONO8);
    int bitsPerPixel = 8;

//    // Memory initialization
//    is_ClearSequence(camera);
//    for(unsigned int i=0; i<1; i++){
//        frameMemory[i] = 0;
//        memoryID[i] = 0;
//        is_AllocImageMem(camera, frameWidth, frameHeight, bitsPerPixel, &frameMemory[i], &memoryID[i]);
//        is_AddToSequence(camera, frameMemory[i], memoryID[i]);
//    }

    // Set display mode - unecessary with InitCamera with NULL pointer to display
    is_SetDisplayMode(camera, IS_SET_DM_DIB);

    // Configure FIFO queue
    //is_InitImageQueue(camera, 0);

    // Single frame memory
    is_AllocImageMem(camera, frameWidth, frameHeight, bitsPerPixel, &frameMemory, &memoryID);
    is_SetImageMem(camera, frameMemory, memoryID);

    // Set max available pixel clock
    unsigned int pixelClockRange[3];
    is_PixelClock(camera, IS_PIXELCLOCK_CMD_GET_RANGE, (void*)pixelClockRange, sizeof(pixelClockRange));
    unsigned int nMax = pixelClockRange[1];
    is_PixelClock(camera, IS_PIXELCLOCK_CMD_SET,(void*)&nMax, sizeof(nMax));

    // Set low framerate to enable long exposure setting.
    // Note: framerate setting has no other effect in trigger mode.
    is_SetFrameRate(camera, 1.0, NULL);

    // Use global shutter
    //    is_SetGlobalShutter(camera, IS_SET_GLOBAL_SHUTTER_ON);
    int shutterMode = IS_DEVICE_FEATURE_CAP_SHUTTER_MODE_GLOBAL;
    is_DeviceFeature(camera, IS_DEVICE_FEATURE_CMD_SET_SHUTTER_MODE, (void*)&shutterMode, sizeof(shutterMode));

    // Enable gain boost
    is_SetGainBoost(camera, IS_SET_GAINBOOST_OFF);

    // Disable hardware gamma
    is_SetHardwareGamma(camera, IS_SET_HW_GAMMA_OFF);

    // Choose starting settings
    CameraSettings settings;
    settings.shutter = 16.666/2;
    settings.gain = 0;
    setCameraSettings(settings);

}

CameraSettings CameraIDSImaging::getCameraSettings(){

    // Get settings:
    CameraSettings settings;

    // Get settings:
    double shutter = settings.shutter;
    is_Exposure(camera, IS_EXPOSURE_CMD_GET_EXPOSURE, &shutter, sizeof(shutter));
    settings.shutter = shutter;
    settings.gain = is_SetHardwareGain(camera, IS_GET_MASTER_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);

    return settings;
}

void CameraIDSImaging::setCameraSettings(CameraSettings settings){

    // Set settings:
    double shutter = settings.shutter;
    is_Exposure(camera, IS_EXPOSURE_CMD_SET_EXPOSURE, &shutter, sizeof(shutter));
    is_SetHardwareGain(camera, settings.gain, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);

}

void CameraIDSImaging::startCapture(){

    if(capturing){
        std::cerr << "CameraIDSImaging: already capturing!" << std::endl;
        return;
    }

    if(triggerMode == triggerModeHardware){

        // Configure for hardware triggered mode
        is_SetExternalTrigger(camera, IS_SET_TRIGGER_LO_HI);

        // Timeout for marking a trigger event as failed
        is_SetTimeout(camera, IS_TRIGGER_TIMEOUT, 10000);

        // Create the Windows event
        #ifdef WIN32
            hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
            //Enable frame event, start image capture and wait for event
            is_InitEvent(camera, hEvent, IS_SET_EVENT_FRAME);
        #endif

        // Enable the frame event
        is_EnableEvent(camera, IS_SET_EVENT_FRAME);

        // Begin transmission
        is_CaptureVideo(camera, IS_DONT_WAIT);
        if(is_CaptureVideo(camera, IS_GET_LIVE) == TRUE)
            capturing = true;
        else
            std::cerr << "CameraIDSImaging Error: could not start capture!" << std::endl;

    } else if(triggerMode == triggerModeSoftware) {

        // Configure for no trigger mode as default
        is_SetExternalTrigger(camera, IS_SET_TRIGGER_SOFTWARE);
    }

    // Print current settings
    std::cout << "Camera IDS Imaging" << std::endl;
    double shutter;
    is_Exposure(camera, IS_EXPOSURE_CMD_GET_EXPOSURE, &shutter, sizeof(shutter));
    std::cout << "Shutter: " << shutter << " ms" << std::endl;
    std::cout << "Gain: " << is_SetHardwareGain(camera, IS_GET_MASTER_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER) << " %" << std::endl;

    capturing = true;

}

void CameraIDSImaging::stopCapture(){

    if(!capturing){
        std::cerr << "CameraIDSImaging: not capturing!" << std::endl;
        return;
    }

    // Disable the frame event
    is_DisableEvent(camera, IS_SET_EVENT_FRAME);

    // Exit Windows event
    #ifdef WIN32
        is_ExitEvent(camera, IS_SET_EVENT_FRAME);
        CloseHandle(hEvent);
    #endif

    // Configure for default software triggered mode
    is_SetExternalTrigger(camera, IS_SET_TRIGGER_SOFTWARE);

    // Stop live video
    int err = is_StopLiveVideo(camera, IS_FORCE_VIDEO_STOP);
    if(err != IS_SUCCESS)
        std::cerr << "CameraIDSImaging Error: could not stop capture!" << std::endl;
    else
        capturing = false;

}

CameraFrame CameraIDSImaging::getFrame(){

    CameraFrame frame;

    if(!capturing){
        std::cerr << "CameraIDSImaging: capturing. Call startCapture() before lockFrame()" << std::endl;
        return frame;
    }

    if(triggerMode == triggerModeHardware){

        #ifdef __LINUX__
            int stat = is_WaitEvent(camera, IS_SET_EVENT_FRAME, 1000);
            if(stat == IS_TIMED_OUT){
                std::cerr << "CameraIDSImaging: Frame timeout!" << std::endl << std::flush;
    //            is_ForceTrigger(camera);
            }
        #elif WIN32
            DWORD dwRet = WaitForSingleObject(hEvent, 1000);
            if (dwRet == WAIT_TIMEOUT){
                std::cerr << "CameraIDSImaging: Frame timeout!" << std::endl << std::flush;
            }
        #endif

        //is_GetActiveImageMem(camera, &frameMemory, &memoryID);
    //    is_FreezeVideo(camera, IS_WAIT);
    } else {
        is_GetActiveImageMem(camera, &frameMemory, &memoryID);
        is_FreezeVideo(camera, IS_WAIT);
    }

    UEYEIMAGEINFO imageInfo;
    is_GetImageInfo(camera, memoryID, &imageInfo, sizeof(imageInfo));

    frame.memory = (unsigned char*)frameMemory;
    frame.width = imageInfo.dwImageWidth;
    frame.height = imageInfo.dwImageHeight;
    frame.timeStamp = imageInfo.u64TimestampDevice;
    frame.sizeBytes = imageInfo.dwImageHeight * imageInfo.dwImageWidth;
    return frame;
}

size_t CameraIDSImaging::getFrameSizeBytes(){

    if (!capturing) {
        std::cerr << "ERROR: Cannot get frame size before capturing." << std::endl;
        return 0;
    }

    UEYEIMAGEINFO imageInfo;
    is_GetImageInfo(camera, memoryID, &imageInfo, sizeof(imageInfo));

    return imageInfo.dwImageHeight * imageInfo.dwImageWidth;
}

unsigned int CameraIDSImaging::getFrameWidth(){

    //    Get image buffer info (only works after capture)
    //    UEYEIMAGEINFO imageInfo;
    //    is_GetImageInfo(camera, currentFrameID, &imageInfo, sizeof(imageInfo));
    return frameWidth;
}

unsigned int CameraIDSImaging::getFrameHeight(){

    return frameHeight;
}

CameraIDSImaging::~CameraIDSImaging(){

    std::cout<<"Closing camera\n"<<std::flush;

    if(capturing)
        stopCapture();

    is_FreeImageMem(camera, frameMemory, memoryID);

    // Exit and free memories
    // Note: IDS defines an OpenGLContext too, which might collide with our own class
    int err = is_ExitCamera(camera);
    if(err != IS_SUCCESS)
        std::cerr << "CameraIDSImaging: Could not exit camera!" << std::endl << std::flush;

    std::cout<<"Camera closed\n"<<std::flush;
}


