#include "CameraIIDC.h"
#include <cstring>

vector<CameraInfo> CameraIIDC::getCameraList(){
    
    dc1394_t *context = dc1394_new();
    
    dc1394camera_list_t *camera_list;
    dc1394error_t err;
    err = dc1394_camera_enumerate(context, &camera_list);
    DC1394_WRN(err,"libdc1394: Failed to enumerate cameras!");

    vector<CameraInfo> ret;
    
    for (unsigned int i=0; i<camera_list->num; i++) {
        CameraInfo info;
        dc1394camera_t *cam;
        cam = dc1394_camera_new(context, camera_list->ids[i].guid);

        //info.vendor = std::string(cam->vendor ? cam->vendor : "");
        info.vendor = "IIDC";
        info.model = string(cam->model ? cam->model : "");
        info.busID = (unsigned int)cam->guid;
        
        dc1394_camera_free(cam);
        ret.push_back(info);
    }
    
    dc1394_camera_free_list(camera_list);
    dc1394_free(context);
    
    return ret;
}

CameraIIDC::CameraIIDC(unsigned int camNum, CameraTriggerMode triggerMode) : Camera(triggerMode) {

    context = dc1394_new();
    
    dc1394camera_list_t *camera_list;
    dc1394error_t err;
    err = dc1394_camera_enumerate(context, &camera_list);
    
    if(camNum+1 > camera_list->num){
        cerr << "libdc1394: Selected camera is not available!";
        return;
    }
    
    cam = NULL;

    cam = dc1394_camera_new(context, camera_list->ids[camNum].guid);
    if(!cam){
        cerr << "libdc1394: Could not open camera!";
        return;
    }

    dc1394_camera_free_list(camera_list);

//    // Get video modes
//    dc1394video_modes_t video_modes;
//    err=dc1394_video_get_supported_modes(cam, &video_modes);
//    DC1394_WRN(err,"Can't get video modes");
//    // Select highest res mode
//    int i;
//    dc1394color_coding_t coding;
//    for (i=video_modes.num-1;i>=0;i--) {
//        if (!dc1394_is_video_mode_scalable(video_modes.modes[i])) {
//            dc1394_get_color_coding_from_video_mode(cam, video_modes.modes[i], &coding);
//            DC1394_WRN(err,"libdc1394: Could not get color coding");
//            if (coding==DC1394_COLOR_CODING_MONO8) {
//                video_mode=video_modes.modes[i];
//                break;
//            }
//        }
//    }
//    if (i < 0) {
//        cerr << "libdc1394: Could not get a valid gray scale mode!";
//        return;
//    }

    // Select format 7 mode 0
    video_mode = DC1394_VIDEO_MODE_FORMAT7_0;

    // Set bit depth
    err = dc1394_format7_set_color_coding(cam, video_mode, DC1394_COLOR_CODING_MONO8);
    if (err!=DC1394_SUCCESS)
        cerr << "libdc1394: Could not set video mode or color coding!";

//    // Set video mode
//    err=dc1394_video_set_mode(cam, video_mode);
//    if (err!=DC1394_SUCCESS)
//        cerr << "libdc1394: Could not set video mode!";

    // Setup capture
    err=dc1394_video_set_operation_mode(cam, DC1394_OPERATION_MODE_1394B);
    if (err!=DC1394_SUCCESS)
        cerr << "libdc1394: Could not set operation mode!";

    err=dc1394_video_set_iso_speed(cam, DC1394_ISO_SPEED_800);
    if (err!=DC1394_SUCCESS)
        cerr << "libdc1394: Could not set iso speed!";

    // Size of ringbuffer (Note: on OS X 10.9, size must be greater than 1)
#ifdef __APPLE__
    unsigned int bufferSize = 2;
#else
    unsigned int bufferSize = 1;
#endif

    err=dc1394_capture_setup(cam, bufferSize, DC1394_CAPTURE_FLAGS_DEFAULT);
    if (err!=DC1394_SUCCESS)
        cerr << "libdc1394: Could not set up camera!";

    // Disable auto exposure mode
    dc1394_feature_set_power(cam, DC1394_FEATURE_EXPOSURE, DC1394_OFF);

    // Disable gamma mode
    dc1394_feature_set_power(cam, DC1394_FEATURE_GAMMA, DC1394_OFF);

    // Disable sharpness mode
    dc1394_feature_set_power(cam, DC1394_FEATURE_SHARPNESS, DC1394_OFF);

    // Disable frame-rate mode
    dc1394_feature_set_power(cam, DC1394_FEATURE_FRAME_RATE, DC1394_OFF);

    // Set manual settings
    dc1394_feature_set_mode(cam, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL);
    dc1394_feature_set_mode(cam, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_MANUAL);


    // Set reasonable default settings
    CameraSettings settings;
    //settings.shutter = 8.333;
    settings.shutter = 16.66;
    settings.gain = 0.0;
    this->setCameraSettings(settings);

    // Print camera features and settings
    dc1394featureset_t features;
    err=dc1394_feature_get_all(cam, &features);
    DC1394_WRN(err, "libdc1394: Could not get feature set.");
    dc1394_feature_print_all(&features, stdout);
    fflush(stdout);

    return;
}
    

unsigned int getSetting(dc1394camera_t *cam, dc1394feature_t setting){
    dc1394error_t err;
    unsigned int value;
    err = dc1394_feature_get_value(cam, setting, &value);
    if(err != DC1394_SUCCESS)
        cerr << "Could not get setting!" << endl;

    return value;
}

CameraSettings CameraIIDC::getCameraSettings(){

    // Get settings:
    CameraSettings settings;
    settings.gain = getSetting(cam, DC1394_FEATURE_GAIN);
    settings.shutter = getSetting(cam, DC1394_FEATURE_SHUTTER);

    return settings;
}

void setSetting(dc1394camera_t *cam, dc1394feature_t setting, float value){
    dc1394error_t err;
    float min, max;
    dc1394_feature_get_absolute_boundaries(cam, setting, &min, &max);
    if(value < min || value > max){
        std::cerr << "CameraIIDC: cannot set value. Out of permissable range." << std::endl;
    } else {
        err = dc1394_feature_set_absolute_control(cam, setting, DC1394_ON);
        if(err != DC1394_SUCCESS)
            cerr << "Could not enable absolute control!" << endl;
        err = dc1394_feature_set_absolute_value(cam, setting, value);
        if(err != DC1394_SUCCESS)
            cerr << "Could not set absolute value!" << endl;
    }
}

void CameraIIDC::setCameraSettings(CameraSettings settings){

    // Set settings:
    setSetting(cam, DC1394_FEATURE_GAIN, settings.gain);
    setSetting(cam, DC1394_FEATURE_SHUTTER, settings.shutter/1000.0); // [ms]
}


void CameraIIDC::startCapture(){

    dc1394error_t err;

    // Print camera information
    dc1394_camera_print_info(cam, stdout);
    fflush(stdout);

    if(triggerMode == triggerModeHardware){

        // Set hardware trigger
        err=dc1394_external_trigger_set_power(cam, DC1394_ON);
        if (err!=DC1394_SUCCESS)
            cerr << "libdc1394: Could not set external trigger on!";

        err=dc1394_external_trigger_set_source(cam, DC1394_TRIGGER_SOURCE_0);
        if (err!=DC1394_SUCCESS)
            cerr << "libdc1394: Could not set external trigger source!";

        err=dc1394_external_trigger_set_mode(cam, DC1394_TRIGGER_MODE_14);
        if (err!=DC1394_SUCCESS)
            cerr << "libdc1394: Could not set external trigger mode!";

        err=dc1394_external_trigger_set_polarity(cam, DC1394_TRIGGER_ACTIVE_HIGH);
        if (err!=DC1394_SUCCESS)
            cerr << "libdc1394: Could not set external trigger polarity!";

    } else if(triggerMode == triggerModeSoftware) {

        // Disable hardware trigger
        err=dc1394_external_trigger_set_power(cam, DC1394_OFF);
        if (err!=DC1394_SUCCESS)
            cerr << "libdc1394: Could not set external trigger off!";

    }

    // Begin transmission
	err = dc1394_video_set_transmission(cam, DC1394_ON);
	if (err!=DC1394_SUCCESS)
	{
		cerr << "ERROR: Could not begin transmission!" << endl;
	}    

    capturing = true;
}

void CameraIIDC::stopCapture(){

    dc1394error_t err;

    err = dc1394_video_set_transmission(cam, DC1394_OFF);
    err = dc1394_capture_stop(cam);

//    // Disable trigger
//    err=dc1394_external_trigger_set_power(cam, DC1394_OFF);
//    if (err!=DC1394_SUCCESS)
//        cerr << "libdc1394: Could not set external trigger on!";

    capturing = false;
}

CameraFrame CameraIIDC::getFrame(){

    CameraFrame frame;

    if (!capturing) {
        cerr << "ERROR: Not capturing on camera. Call startCapture() before lockFrame()." << endl;
        return frame;
    }

    dc1394error_t err;
    
    if(triggerMode == triggerModeSoftware){

        if (cam->one_shot_capable != DC1394_TRUE){
            cerr << "ERROR: Camera is not one_shot_capable." << endl;
            return frame;
        }

        dc1394error_t err;

        // Flush the ring buffer
        flushBuffer();

        // One-shot trigger
        err == dc1394_video_set_one_shot(cam, DC1394_ON);

    }

    // Get frame from ring buffer:
    err = dc1394_capture_dequeue(cam, DC1394_CAPTURE_POLICY_WAIT, &currentFrame);
    if (err!=DC1394_SUCCESS){
        cerr << "ERROR: Could not capture a frame." << endl;
        return frame;
    }

    // Return the frame to the ring buffer:
    dc1394_capture_enqueue(cam, currentFrame);
    currentFrame = NULL;

    // Copy frame address and properties
    frame.memory = currentFrame->image;
    frame.width = currentFrame->size[0];
    frame.height = currentFrame->size[1];
    frame.sizeBytes = currentFrame->image_bytes;

    return frame;
}

void CameraIIDC::flushBuffer(){
    // This function is courtesy of ofxVideoGrabber/Libdc1394Grabber
    bool bufferEmpty = false;

    while (!bufferEmpty){
        if(dc1394_capture_dequeue(cam, DC1394_CAPTURE_POLICY_POLL, &currentFrame) == DC1394_SUCCESS){
            if(currentFrame != NULL){
                dc1394_capture_enqueue(cam, currentFrame);
            } else {
                bufferEmpty = true;
            }
        } else {
            bufferEmpty = true;
        }
    }
}


size_t CameraIIDC::getFrameSizeBytes(){
    if (!capturing) {
        cerr << "ERROR: Cannot get frame size before capturing. Call startCapture() before getFrameSizeBytes()." << endl;
        return 0;
    }

//    dc1394video_frame_t *frame = NULL;
//    dc1394error_t err;
    
//	// Get frame from ring buffer:
//    err = dc1394_capture_dequeue(cam, DC1394_CAPTURE_POLICY_WAIT, &frame);
//    err = dc1394_capture_enqueue(cam, frame);
    
//    return frame->image_bytes;
    uint64_t total_bytes;
    dc1394_format7_get_total_bytes(cam, video_mode, &total_bytes);

    return total_bytes;
}

size_t CameraIIDC::getFrameWidth(){
    uint32_t _width, _height;
    dc1394_get_image_size_from_video_mode(cam, video_mode, &_width, &_height);
    return _width;
}


size_t CameraIIDC::getFrameHeight(){
    uint32_t _width, _height;
    dc1394_get_image_size_from_video_mode(cam, video_mode, &_width, &_height);
    return _height;
}


CameraIIDC::~CameraIIDC(){
    // Stop camera transmission
    if(capturing)
        stopCapture();
    else
        dc1394_capture_stop(cam);

    // Gracefulle destruct the camera
    dc1394_camera_free(cam);
    dc1394_free(context);
}



