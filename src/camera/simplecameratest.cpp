#include <ueye.h>
#include <iostream>

using namespace std;

int main(int argc, char *argv[]){

    // Init Camera
    HIDS camera = (HIDS) 0 + 1;
    is_InitCamera(&camera, NULL);

    // Error reporting
    is_SetErrorReport(camera, IS_ENABLE_ERR_REP);

    //    // Set display mode
    //    is_SetDisplayMode(camera, IS_SET_DM_DIB);

    // Use 2x scaler to double attainable framerate
    is_SetSensorScaler(camera, IS_ENABLE_SENSOR_SCALER, 2.0);

    // Get current AOI info
    IS_RECT rectAOI;
    is_AOI(camera, IS_AOI_IMAGE_GET_AOI, (void*)&rectAOI, sizeof(rectAOI));
    // Decrease AOI a little to improve readout speed
    rectAOI.s32X = 6; rectAOI.s32Width -= 12;
    is_AOI(camera, IS_AOI_IMAGE_SET_AOI, (void*)&rectAOI, sizeof(rectAOI));
    unsigned int frameWidth = rectAOI.s32Width;
    unsigned int frameHeight = rectAOI.s32Height;

    // Set up 8bit monochrome color depth
    is_SetColorMode(camera, IS_CM_MONO8);
    int bitsPerPixel = 8;

    char* frameMemory[10]; // 10 char pointers
    int memoryID[10];

    // Memory initialization
    is_ClearSequence(camera);
    for(unsigned int i=0; i<10; i++){
        is_AllocImageMem(camera, frameWidth, frameHeight, bitsPerPixel, &frameMemory[i], &memoryID[i]);
        is_AddToSequence(camera, frameMemory[i], memoryID[i]);
    }
    // Configure FIFO queue
//    is_InitImageQueue(camera, 0);

//        // Single frame memory
//        is_AllocImageMem(camera, frameWidth, frameHeight, bitsPerPixel, &currentFrameMemory, &currentFrameID);
//        is_SetImageMem(camera, currentFrameMemory, currentFrameID);

    // Set max available pixel clock
    unsigned int pixelClockRange[3];
    is_PixelClock(camera, IS_PIXELCLOCK_CMD_GET_RANGE, (void*)pixelClockRange, sizeof(pixelClockRange));
    unsigned int nMax = pixelClockRange[1];
    is_PixelClock(camera, IS_PIXELCLOCK_CMD_SET,(void*)&nMax, sizeof(nMax));

    // Set low framerate to enable long exposure setting.
    // Note: framerate setting has no other effect in trigger mode.
    is_SetFrameRate(camera, 30.0, NULL);

    //    // Use global shutter
    //    is_SetGlobalShutter(camera, IS_SET_GLOBAL_SHUTTER_ON);
//    int shutterMode = IS_DEVICE_FEATURE_CAP_SHUTTER_MODE_GLOBAL;
//    is_DeviceFeature(camera, IS_DEVICE_FEATURE_CMD_SET_SHUTTER_MODE, (void*)&shutterMode, sizeof(shutterMode));

    // Enable gain boost
    is_SetGainBoost(camera, IS_SET_GAINBOOST_ON);

    // Disable hardware gamma
    is_SetHardwareGamma(camera, IS_SET_HW_GAMMA_OFF);

    // Print current settings
    cout << "Camera IDS Imaging" << endl;
    double shutter;
    is_Exposure(camera, IS_EXPOSURE_CMD_GET_EXPOSURE, &shutter, sizeof(shutter));
    cout << "Shutter: " << shutter << " ms" << endl;
    cout << "Gamma: " << (float)is_SetGamma(camera, IS_GET_GAMMA)/100.0 << endl;
    cout << "Gain: " << is_SetHardwareGain(camera, IS_GET_MASTER_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER) << " %" << endl;

    // Configure for hardware triggered mode
    is_SetExternalTrigger(camera, IS_SET_TRIGGER_OFF);

    // Timeout for marking a trigger event as failed
    //is_SetTimeout(camera, IS_TRIGGER_TIMEOUT, 10000);

    // Begin transmission
    is_CaptureVideo(camera, IS_DONT_WAIT);

    is_EnableEvent(camera, IS_SET_EVENT_FRAME);

    // Capture a couple of frames
    char* currentFrameMemory;
    int currentFrameID;
    for(unsigned int i=0; i<100; i++){
//        is_WaitForNextImage(camera, 10000, &currentFrameMemory, &currentFrameID);
        is_WaitEvent(camera, IS_SET_EVENT_FRAME, 1000);
        is_GetActSeqBuf(camera, NULL, NULL, &currentFrameMemory);
        is_LockSeqBuf(camera, IS_IGNORE_PARAMETER, currentFrameMemory);
        is_UnlockSeqBuf(camera, IS_IGNORE_PARAMETER, currentFrameMemory);
        cout << "Caputured frame " << i << endl << flush;
    }

    is_StopLiveVideo(camera, IS_WAIT);

//        is_ExitImageQueue(camera);
//    // Gracefully close the camera
//    is_ClearSequence(camera);

//    unsigned int ringBufferSize = 10;
//    for(unsigned int i=0; i<ringBufferSize; i++){
//        is_FreeImageMem(camera, frameMemory[i], memoryID[i]);
//    }

    // Exit and free memories
    is_ExitCamera(camera);

    return 0;
}



