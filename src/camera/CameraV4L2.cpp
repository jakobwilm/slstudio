#include "CameraV4L2.h"

#include <cstdlib>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <cstdio>
#include <fcntl.h>
#include <memory>
#include <cstring>
#include <iostream>
#include <sys/ioctl.h>

using namespace std;

// static   int      fd;
// static   struct   v4l2_capability   cap;
// struct v4l2_fmtdesc fmtdesc;
// struct v4l2_format fmt,fmtack;
// struct v4l2_streamparm setfps;  
// struct v4l2_requestbuffers req;
// struct v4l2_buffer buf;
// enum v4l2_buf_type buf_type;

// // unsigned char frame_buffer[IMAGEWIDTH*IMAGEHEIGHT*3];
// // unsigned char grey_buffer[IMAGEWIDTH*IMAGEHEIGHT];


// struct v4l2_queryctrl queryctrl;
// struct v4l2_querymenu querymenu;

// struct v4l2_control control;

struct buffer
{
    void * start;
    unsigned int length;
} * buffers;

vector<CameraInfo> CameraV4L2::getCameraList(){
    
    unsigned int cNum = 1; // default camera

    vector<CameraInfo> ret;
    
    int fd;
    struct   v4l2_capability   cap;
    for (unsigned int i=0; i <= cNum; i++) { 
        CameraInfo info;
        string FILE_VIDEO = "/dev/video" + to_string(i);
        cv::VideoCapture camera;

        if ((fd = open(FILE_VIDEO.c_str(), O_RDWR)) == -1)
            continue;

        if ((ioctl(fd, VIDIOC_QUERYCAP, &cap)) == -1 )
        {
            cerr << "V4L2: unable to query device: " << FILE_VIDEO << "\n";
            continue;
        }

        //info.vendor = std::string(cam->vendor ? cam->vendor : "");
        info.vendor = string((char*)cap.card);
        info.model = "video" + to_string(i);
        info.busID = (unsigned int)i;
        
        close(fd);
        ret.push_back(info);
    }
    
    // dc1394_camera_free_list(camera_list);
    // dc1394_free(context);
    
    return ret;
}

CameraV4L2::CameraV4L2(unsigned int camNum, CameraTriggerMode triggerMode) : Camera(triggerMode) {

    string FILE_VIDEO = "/dev/video" + to_string(camNum);
    fd = open(FILE_VIDEO.c_str(), O_RDWR);
    
    if( fd == -1 ){
        cerr << "V4L2: failed to open selected camera!\n";
        return;
    }

    //emu all support fmt
    fmtdesc.index=0;
    fmtdesc.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    printf("Support format:\n");
    while(ioctl(fd,VIDIOC_ENUM_FMT,&fmtdesc)!=-1)
    {
        printf("\t%d.%s\n",fmtdesc.index+1,fmtdesc.description);
        fmtdesc.index++;
    }

    //set fmt
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.height = IMAGEHEIGHT;
    fmt.fmt.pix.width = IMAGEWIDTH;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    
    if(ioctl(fd, VIDIOC_S_FMT, &fmt) == -1)
    {
        perror("VIDIOC_S_FMT");
        printf("Unable to set format\n");
        return;
    }

    if(ioctl(fd, VIDIOC_G_FMT, &fmt) == -1)
    {
        perror("VIDIOC_G_FMT");
        printf("Unable to get format\n");
        return;
    }
    else 
    {
         printf("fmt.type:\t\t%d\n",fmt.type);
         printf("pix.pixelformat:\t%c%c%c%c\n",fmt.fmt.pix.pixelformat & 0xFF, (fmt.fmt.pix.pixelformat >> 8) & 0xFF,(fmt.fmt.pix.pixelformat >> 16) & 0xFF, (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
         printf("pix.height:\t\t%d\n",fmt.fmt.pix.height);
         printf("pix.width:\t\t%d\n",fmt.fmt.pix.width);
         printf("pix.field:\t\t%d\n",fmt.fmt.pix.field);
    }

    // query and emu control
    memset(&queryctrl, 0, sizeof(queryctrl));
    queryctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
    while (0 == ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl)) {
        if (!(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)) {
            printf("Control %s\n", queryctrl.name);
        }

        queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }
    if (errno != EINVAL) {
        perror("VIDIOC_QUERYCTRL");
        return;
    }

    // Set reasonable default settings
    CameraSettings settings;

    settings.shutter = 16.666;
    settings.gain = 0.0;
    this->setCameraSettings(settings);

    this->getCameraSettings();

    // Print camera features and settings
    fflush(stdout);

    return;
}

int CameraV4L2::get_ctrl(unsigned long cid)
{
    memset(&queryctrl, 0, sizeof(queryctrl));
    queryctrl.id = cid;

    if (-1 == ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl))
    {
        if (errno != EINVAL) {
            perror("VIDIOC_QUERYCTRL");
        }
        else{
            printf("CID is not supported1.\n");
        }
        return -1;
    }
    else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
    {
        printf("CID is not supported2.\n");
        return -1;
    }
    else{
        printf("default CID value is %d\n", queryctrl.default_value);
        memset(&control, 0, sizeof(control));
        control.id = cid;
        if (-1 == ioctl(fd, VIDIOC_G_CTRL, &control))
        {
            perror("VIDIOC_G_CTRL");
            return -1;
        }
    }

    return 0;
}
    

CameraSettings CameraV4L2::getCameraSettings(){

    // Get settings:
    CameraSettings settings;

    // get gain control 
    memset(&control, 0, sizeof(control));  
    if ( 0 != get_ctrl(V4L2_CID_GAIN) )
    {
        printf("Error: failed to get GAIN value.\n");
        settings.gain = 0.0;
    } 
    else
    {
        printf("Gain value is %d\n", control.value);
        settings.gain = (float) control.value;
    }

    // get shutter
    memset(&control, 0, sizeof(control));  
    if ( 0 != get_ctrl(V4L2_CID_EXPOSURE_ABSOLUTE) )
    {
        printf("Error: failed to get shutter value.\n");
        settings.shutter = -1.0;
    } 
    else
    {
        printf("shutter value is %d\n", control.value);
        settings.gain = (float) control.value;
    }

    return settings;
}

int CameraV4L2::set_ctrl(unsigned long cid, int value)
{
    memset(&control, 0, sizeof(control));
    control.id = cid;
    control.value = value;

    if (-1 == ioctl(fd, VIDIOC_S_CTRL, &control))
    {
       perror("VIDIOC_S_CTRL");
       return -1;
    }

    return 0;
}

void CameraV4L2::setCameraSettings(CameraSettings settings){

    int gain = (int)settings.gain;
    int shutter = (int)settings.shutter;

    // set gain
    if ( 0 == set_ctrl(V4L2_CID_GAIN, gain) )
    {
        printf("set gain value to %d\n", gain);
    }
    else
    {
        printf("set gain value to %d failed. \n", gain);
    }

    // set shutter
    if ( 0 == set_ctrl(V4L2_CID_EXPOSURE_AUTO, EXPOSURE_MODE_SETTING))
    {
        if ( 0 ==   set_ctrl(V4L2_CID_EXPOSURE_ABSOLUTE, shutter))
        {
            printf("set exposure(shutter) value to %d. \n", shutter);
        }
        else
        {
            printf("set exposure(shutter) value to %d failed. \n", shutter); 
        }
    }
    else
    {
        printf("set exposure mode failed.");
    }
    // setSetting(cam, DC1394_FEATURE_GAIN, settings.gain);
    // setSetting(cam, DC1394_FEATURE_SHUTTER, settings.shutter/1000.0); // [ms]
}

int CameraV4L2::camera_on()
{
    // unsigned int n_buffers;

    // //request for 4 buffers 
    // req.count = 1;
    // req.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    // req.memory=V4L2_MEMORY_MMAP;
    // if(ioctl(fd,VIDIOC_REQBUFS,&req)==-1)
    // {
    //     perror("VIDIOC_REQBUFS");
    //     printf("request for buffers error\n");
    //     return -1;
    // }

    // //mmap for buffers
    // buffers = (buffer*)malloc(req.count*sizeof (*buffers));
    // if (!buffers) 
    // {
    //     printf ("Out of memory\n");
    //     return -1;
    // }
    
    // for (n_buffers = 0; n_buffers < req.count; n_buffers++) 
    // {
    //     buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    //     buf.memory = V4L2_MEMORY_MMAP;
    //     buf.index = n_buffers;
    //     //query buffers
    //     if (ioctl (fd, VIDIOC_QUERYBUF, &buf) == -1)
    //     {
    //         printf("query buffer error\n");
    //         return -1;
    //     }

    //     buffers[n_buffers].length = buf.length;
    //     //map
    //     buffers[n_buffers].start = mmap(NULL,buf.length,PROT_READ |PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    //     if (buffers[n_buffers].start == MAP_FAILED)
    //     {
    //         printf("buffer map error\n");
    //         return -1;
    //     }
    // }

    //queue
    for (unsigned int n_buffers = 0; n_buffers < req.count; n_buffers++)
    {
        buf.index = n_buffers;
        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1)
        {
            perror("VIDIOC_QBUF");
            return -1;
        }
    } 
    
    buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl (fd, VIDIOC_STREAMON, &buf_type);        
    
    printf("stream on OK\n");

    return 0;

}


void CameraV4L2::startCapture(){

    // if (0 != camera_on())
    // {
    //     capturing = false;
    //     return;
    // }

    unsigned int n_buffers;

    //request for 4 buffers 
    req.count = 1;
    req.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory=V4L2_MEMORY_MMAP;
    if(ioctl(fd,VIDIOC_REQBUFS,&req)==-1)
    {
        perror("VIDIOC_REQBUFS");
        printf("request for buffers error\n");
        return;
    }

    //mmap for buffers
    buffers = (buffer*)malloc(req.count*sizeof (*buffers));
    if (!buffers) 
    {
        printf ("Out of memory\n");
        return;
    }
    
    for (n_buffers = 0; n_buffers < req.count; n_buffers++) 
    {
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;
        //query buffers
        if (ioctl (fd, VIDIOC_QUERYBUF, &buf) == -1)
        {
            printf("query buffer error\n");
            return;
        }

        buffers[n_buffers].length = buf.length;
        //map
        buffers[n_buffers].start = mmap(NULL,buf.length,PROT_READ |PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if (buffers[n_buffers].start == MAP_FAILED)
        {
            printf("buffer map error\n");
            return;
        }
    }

    if (0 != camera_on())
    {
        return;
    }

    capturing = true;
}

int CameraV4L2::camera_off()
{

    // for(unsigned int i = 0;i < req.count; i++)
    // {
    //     munmap(buffers[i].start, buffers[i].length);
    // }

    buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMOFF, &buf_type);

    printf("stream off OK\n");

    return 0;    

}

void CameraV4L2::stopCapture(){

//     dc1394error_t err;

//     err = dc1394_video_set_transmission(cam, DC1394_OFF);
//     err = dc1394_capture_stop(cam);

// //    // Disable trigger
// //    err=dc1394_external_trigger_set_power(cam, DC1394_OFF);
//     if (err!=DC1394_SUCCESS)
//         cerr << "libdc1394: Could not stop capture!";

  
    for(unsigned int i = 0;i < req.count; i++)
    {
        munmap(buffers[i].start, buffers[i].length);
    }

    camera_off();

    capturing = false;
}

int CameraV4L2::flush_buffer(){
    // This function is courtesy of ofxVideoGrabber/Libdc1394Grabber
    // bool bufferEmpty = false;

    // while (!bufferEmpty){
    //     if(dc1394_capture_dequeue(cam, DC1394_CAPTURE_POLICY_POLL, &currentFrame) == DC1394_SUCCESS){
    //         if(currentFrame != NULL){
    //             dc1394_capture_enqueue(cam, currentFrame);
    //         } else {
    //             bufferEmpty = true;
    //         }
    //     } else {
    //         bufferEmpty = true;
    //     }
    // }

    for (unsigned int n_buffers = 0; n_buffers < req.count; n_buffers++)
    {
        buf.index = n_buffers;
        if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1)
        {
            perror("VIDIOC_QBUF");
        }

        if (ioctl(fd, VIDIOC_QBUF, &buf) ==  -1)
        {
            perror("VIDIOC_QBUF");
        }
    }

    return 0;
}

CameraFrame CameraV4L2::getFrame(){

    CameraFrame frame;

    if (!capturing) {
        cerr << "ERROR: Not capturing on camera. Call startCapture() before lockFrame()." << endl;
        return frame;
    }
    // camera_off();
    // camera_on();

    flush_buffer();

    if (-1 == ioctl(fd, VIDIOC_DQBUF, &buf))
        perror("VIDIOC_DQBUF");
    
    printf("buf index = %d\n", buf.index);
    
    yuyv_2_grey();

    // Copy frame address and properties
    frame.memory = grey_buffer;
    frame.width  = IMAGEWIDTH;
    frame.height = IMAGEHEIGHT;
    frame.type   = CV_8U;
    frame.sizeBytes = IMAGEHEIGHT * IMAGEWIDTH;

    if (ioctl(fd, VIDIOC_QBUF, &buf) == -1)
    {
        perror("VIDIOC_QBUF");
    }

    // camera_off();

    return frame;
}

int CameraV4L2::yuyv_2_grey(void)
{
    int           i,j;
    unsigned char y1,y2,u,v;
    char * pointer;
    
    pointer = (char*)buffers[buf.index].start;

    memset(grey_buffer, 0, IMAGEWIDTH * IMAGEHEIGHT);
       
    for(i=0;i < IMAGEHEIGHT;i++)
    {
        for(j=0;j<IMAGEWIDTH/2;j++)
        {
            y1 = *( pointer + (i*IMAGEWIDTH / 2+j)*4);
            u  = *( pointer + (i*IMAGEWIDTH / 2+j)*4 + 1);
            y2 = *( pointer + (i*IMAGEWIDTH / 2+j)*4 + 2);
            v  = *( pointer + (i*IMAGEWIDTH / 2+j)*4 + 3);
                                
            *(grey_buffer + ((i)*IMAGEWIDTH / 2+j)*2    ) = (unsigned char)y1;
            *(grey_buffer + ((i)*IMAGEWIDTH / 2+j)*2 + 1) = (unsigned char)y2;
        }
    }

    // memset(buffers[buf.index].start, 0 , buffers[buf.index].length);
    // printf("change to GREY OK \n");
    return 0;
}

// void CameraV4L2::flushBuffer(){
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


size_t CameraV4L2::getFrameSizeBytes(){
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

size_t CameraV4L2::getFrameWidth(){
    // uint32_t _width, _height;
    // dc1394_get_image_size_from_video_mode(cam, video_mode, &_width, &_height);

    // int _width = static_cast<int>(camera->get(CV_CAP_PROP_FRAME_WIDTH));
    
    return fmt.fmt.pix.width;
}


size_t CameraV4L2::getFrameHeight(){
    // uint32_t _width, _height;
    // dc1394_get_image_size_from_video_mode(cam, video_mode, &_width, &_height);

    // int _height = static_cast<int>(stream1->get(CV_CAP_PROP_FRAME_HEIGHT));

    return fmt.fmt.pix.height;
}


CameraV4L2::~CameraV4L2(){
    // Stop camera transmission
    if(capturing)
        stopCapture();

    // Gracefulle destruct the camera
    // if (camera != NULL)
    // {
    //     camera->release();
    //     delete camera;
    //     camera = NULL;
    // }
    if (fd != -1)
        close(fd);
}



