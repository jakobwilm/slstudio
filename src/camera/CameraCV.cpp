#include "CameraCV.h"
#include <cstring>

vector<CameraInfo> CameraCV::getCameraList(){
    
    unsigned int cNum = 10; // default camera
    // dc1394_t *context = dc1394_new();
    
    // dc1394camera_list_t *camera_list;
    // dc1394error_t err;
    // err = dc1394_camera_enumerate(context, &camera_list);
    // DC1394_WRN(err,"libdc1394: Failed to enumerate cameras!");

    vector<CameraInfo> ret;
    
    for (unsigned int i=0; i <= cNum; i++) {
        CameraInfo info;
        cv::VideoCapture camera;

        camera.open(i);

        if (!camera.isOpened())
            continue;

        // dc1394camera_t *cam;
        // cam = dc1394_camera_new(context, camera_list->ids[i].guid);

        //info.vendor = std::string(cam->vendor ? cam->vendor : "");
        info.vendor = "OPENCV";
        info.model = "video" + to_string(i);
        info.busID = (unsigned int)i;
        
        camera.release();
        ret.push_back(info);
    }
    
    // dc1394_camera_free_list(camera_list);
    // dc1394_free(context);
    
    return ret;
}

CameraCV::CameraCV(unsigned int camNum, CameraTriggerMode triggerMode) : Camera(triggerMode) {
    camera = new cv::VideoCapture(camNum);
    
    if(!camera->isOpened()){
        cerr << "OPENCV: failed to open selected camera!\n";
        return;
    }

    _width = static_cast<size_t>(camera->get(CV_CAP_PROP_FRAME_WIDTH));
    _height = static_cast<size_t>(camera->get(CV_CAP_PROP_FRAME_HEIGHT));

    // Set reasonable default settings
    CameraSettings settings;

    settings.shutter = 0.0;
    settings.gain = 0.0;
    this->setCameraSettings(settings);

    // Print camera features and settings
    fflush(stdout);

    return;
}
    

CameraSettings CameraCV::getCameraSettings(){

    // Get settings:
    CameraSettings settings;
    settings.gain = camera->get(CV_CAP_PROP_GAIN);
    // settings.shutter = getSetting(cam, DC1394_FEATURE_SHUTTER);

    return settings;
}

void CameraCV::setCameraSettings(CameraSettings settings){

    // Set settings:
    // setSetting(cam, DC1394_FEATURE_GAIN, settings.gain);
    // setSetting(cam, DC1394_FEATURE_SHUTTER, settings.shutter/1000.0); // [ms]
}


void CameraCV::startCapture(){

 //    dc1394error_t err;

 //    // Print camera information
 //    dc1394_camera_print_info(cam, stdout);
 //    fflush(stdout);

 //    if(triggerMode == triggerModeHardware){

 //        // Set hardware trigger
 //        err=dc1394_external_trigger_set_power(cam, DC1394_ON);
 //        if (err!=DC1394_SUCCESS)
 //            cerr << "libdc1394: Could not set external trigger on!";

 //        err=dc1394_external_trigger_set_source(cam, DC1394_TRIGGER_SOURCE_0);
 //        if (err!=DC1394_SUCCESS)
 //            cerr << "libdc1394: Could not set external trigger source!";

 //        err=dc1394_external_trigger_set_mode(cam, DC1394_TRIGGER_MODE_14);
 //        if (err!=DC1394_SUCCESS)
 //            cerr << "libdc1394: Could not set external trigger mode!";

 //        err=dc1394_external_trigger_set_polarity(cam, DC1394_TRIGGER_ACTIVE_HIGH);
 //        if (err!=DC1394_SUCCESS)
 //            cerr << "libdc1394: Could not set external trigger polarity!";

 //    } else if(triggerMode == triggerModeSoftware) {

 //        // Disable hardware trigger
 //        err=dc1394_external_trigger_set_power(cam, DC1394_OFF);
 //        if (err!=DC1394_SUCCESS)
 //            cerr << "libdc1394: Could not set external trigger off!";

 //    }

 //    // Begin transmission
	// err = dc1394_video_set_transmission(cam, DC1394_ON);
	// if (err!=DC1394_SUCCESS)
	// {
	// 	cerr << "ERROR: Could not begin transmission!" << endl;
	// }    

    capturing = true;
}

void CameraCV::stopCapture(){

//     dc1394error_t err;

//     err = dc1394_video_set_transmission(cam, DC1394_OFF);
//     err = dc1394_capture_stop(cam);

// //    // Disable trigger
// //    err=dc1394_external_trigger_set_power(cam, DC1394_OFF);
//     if (err!=DC1394_SUCCESS)
//         cerr << "libdc1394: Could not stop capture!";

    capturing = false;
}

CameraFrame CameraCV::getFrame(){

    CameraFrame frame;

    if (!capturing) {
        cerr << "ERROR: Not capturing on camera. Call startCapture() before lockFrame()." << endl;
        return frame;
    }

    // dc1394error_t err;
    
    // if(triggerMode == triggerModeSoftware){

    //     if (cam->one_shot_capable != DC1394_TRUE){
    //         cerr << "ERROR: Camera is not one_shot_capable." << endl;
    //         return frame;
    //     }

    //     dc1394error_t err;

    //     // Flush the ring buffer
    //     flushBuffer();

    //     // One-shot trigger
    //     err == dc1394_video_set_one_shot(cam, DC1394_ON);

    //     if (err!=DC1394_SUCCESS){
    //         cerr << "ERROR: Could not enable one-shot buffer." << endl;
    //     }
    // }

    // Get frame from ring buffer:
    // err = dc1394_capture_dequeue(cam, DC1394_CAPTURE_POLICY_WAIT, &currentFrame);
    // if (err!=DC1394_SUCCESS){
    //     cerr << "ERROR: Could not capture a frame." << endl;
    //     return frame;
    // }

    // // Return the frame to the ring buffer:
    // dc1394_capture_enqueue(cam, currentFrame);
    // currentFrame = NULL;
    cv::Mat colorFrame;
    camera->read(colorFrame);

    cv::cvtColor(colorFrame, cameraFrame, CV_BGR2GRAY);

    // Copy frame address and properties
    frame.memory = cameraFrame.data;
    frame.width = cameraFrame.cols;
    frame.height = cameraFrame.rows;
    frame.type = cameraFrame.type();
    frame.sizeBytes = cameraFrame.total() * cameraFrame.elemSize();

    return frame;
}

// void CameraCV::flushBuffer(){
//     // This function is courtesy of ofxVideoGrabber/Libdc1394Grabber
//     bool bufferEmpty = false;

//     while (!bufferEmpty){
//         if(dc1394_capture_dequeue(cam, DC1394_CAPTURE_POLICY_POLL, &currentFrame) == DC1394_SUCCESS){
//             if(currentFrame != NULL){
//                 dc1394_capture_enqueue(cam, currentFrame);
//             } else {
//                 bufferEmpty = true;
//             }
//         } else {
//             bufferEmpty = true;
//         }
//     }
// }


size_t CameraCV::getFrameSizeBytes(){
    // if (!capturing) {
    //     cerr << "ERROR: Cannot get frame size before capturing. Call startCapture() before getFrameSizeBytes()." << endl;
    //     return 0;
    // }

//    dc1394video_frame_t *frame = NULL;
//    dc1394error_t err;
    
//	// Get frame from ring buffer:
//    err = dc1394_capture_dequeue(cam, DC1394_CAPTURE_POLICY_WAIT, &frame);
//    err = dc1394_capture_enqueue(cam, frame);
    
//    return frame->image_bytes;
    // uint64_t total_bytes;
    // dc1394_format7_get_total_bytes(cam, video_mode, &total_bytes);

    return 0;
    // return total_bytes;
}

size_t CameraCV::getFrameWidth(){
    // uint32_t _width, _height;
    // dc1394_get_image_size_from_video_mode(cam, video_mode, &_width, &_height);

    // int _width = static_cast<int>(camera->get(CV_CAP_PROP_FRAME_WIDTH));
    
    return _width;
}


size_t CameraCV::getFrameHeight(){
    // uint32_t _width, _height;
    // dc1394_get_image_size_from_video_mode(cam, video_mode, &_width, &_height);

    // int _height = static_cast<int>(stream1->get(CV_CAP_PROP_FRAME_HEIGHT));

    return _height;
}


CameraCV::~CameraCV(){
    // Stop camera transmission
    if(capturing)
        stopCapture();

    // Gracefulle destruct the camera
    if (camera != NULL)
    {
        camera->release();
        delete camera;
        camera = NULL;
    }
}



