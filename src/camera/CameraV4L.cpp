/*
 *  Author: Benjamin Langmann (b.langmann@gmx.de)
 *  Date: 2016
 */

#include "CameraV4L.h"

#include <string>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

enum io_method {
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
};

static int xioctl(int fh, int request, void *arg)
{
    int r;

    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);

    return r;
}

static int open_device(const char* dev_name)
{
    struct stat st;

    if (-1 == stat(dev_name, &st)) {
        fprintf(stderr, "Cannot identify '%s': %d, %s\n", dev_name, errno, strerror(errno));
        return -1;
    }

    if (!S_ISCHR(st.st_mode)) {
        fprintf(stderr, "%s is no device\n", dev_name);
        return -1;
    }

    int fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == fd) {
        fprintf(stderr, "Cannot open '%s': %d, %s\n", dev_name, errno, strerror(errno));
        return -1;
    }
    return fd;
}

static bool uninit_device(int io, struct buffer *buffers, size_t n_buffers)
{
        unsigned int i;

        switch (io) {
        case IO_METHOD_READ:
                delete[] buffers[0].start;
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i)
                        if (-1 == munmap(buffers[i].start, buffers[i].length))
                                return false;
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i)
                        delete[] buffers[i].start;
                break;
        }

        delete[] buffers;
        return true;
}

static bool init_read(struct buffer **buffers, unsigned int buffer_size)
{
        *buffers = new buffer[1];

        if (!*buffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        (*buffers)[0].length = buffer_size;
        (*buffers)[0].start = new char[buffer_size];

        if (!(*buffers)[0].start) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }
        return true;
}

static bool init_mmap(int fd, struct buffer **buffers, int n_buffers, unsigned int buffer_size)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = n_buffers;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            cerr << "Camera does not support memory mapping" << endl;
            return false;
        } else {
            cerr << "Error initializing device (VIDIOC_REQBUFS)" << endl;
            return false;
        }
    }

    if (req.count != n_buffers) {
        cerr << "Insufficient buffer memory" << endl;
        return false;
    }

    *buffers = new buffer[n_buffers];

    if (!buffers) {
            fprintf(stderr, "Out of memory\n");
            exit(EXIT_FAILURE);
    }

    for (int i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;

        if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf)) {
            cerr << "Error initializing device (VIDIOC_QUERYBUF)" << endl;
            return false;
        }

        (*buffers)[n_buffers].length = buf.length;
        (*buffers)[n_buffers].start = (char*)
                mmap(NULL /* start anywhere */,
                      buf.length,
                      PROT_READ | PROT_WRITE /* required */,
                      MAP_SHARED /* recommended */,
                      fd, buf.m.offset);

        if (MAP_FAILED == (*buffers)[n_buffers].start) {
            cerr << "Error initializing device (mmap)" << endl;
            return false;
        }
    }
    return true;
}


static bool init_userp(int fd, struct buffer **buffers, int n_buffers, unsigned int buffer_size)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count  = n_buffers;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
            if (EINVAL == errno) {
                cerr << "Camera does not support user pointer i/o" << endl;
                return false;
            } else {
                cerr << "Error initializing device (VIDIOC_REQBUFS)" << endl;
                return false;
            }
        }

        *buffers = new buffer[n_buffers];

        if (!buffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (int i = 0; i < n_buffers; ++i) {
                (*buffers)[i].length = buffer_size;
                (*buffers)[i].start = new char[buffer_size];

                if (!(*buffers)[i].start) {
                        fprintf(stderr, "Out of memory\n");
                        exit(EXIT_FAILURE);
                }
        }
        return true;
}


static bool init_device(int fd, const char* dev_name, int io, buffer** buffers, size_t n_buffers, struct v4l2_format& fmt)
{
        struct v4l2_capability cap;
        struct v4l2_cropcap cropcap;
        struct v4l2_crop crop;

        unsigned int min;

        if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s is no V4L2 device\n",
                                 dev_name);
                        return false;
                } else {
                        return false;
                }
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
                fprintf(stderr, "%s is no video capture device\n",
                         dev_name);
                return false;
        }

        switch (io) {
        case IO_METHOD_READ:
                if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
                        fprintf(stderr, "%s does not support read i/o\n",
                                 dev_name);
                        return false;
                }
                break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
                if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
                        fprintf(stderr, "%s does not support streaming i/o\n",
                                 dev_name);
                        return false;
                }
                break;
        }


        /* Select video input, video standard and tune here. */

        CLEAR(cropcap);

        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
                crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                crop.c = cropcap.defrect; /* reset to default */

                if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
                        switch (errno) {
                        case EINVAL:
                                /* Cropping not supported. */
                                break;
                        default:
                                /* Errors ignored. */
                                break;
                        }
                }
        } else {
                /* Errors ignored. */
        }

        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (0) {
                CLEAR(fmt);
                fmt.fmt.pix.width       = 640;
                fmt.fmt.pix.height      = 480;
                fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
                fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

                if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                {
                        cerr << "Error initializing device (VIDIOC_S_FMT)" << endl;
                        return false;
                }
                /* Note VIDIOC_S_FMT may change width and height. */
        } else {
                /* Preserve original settings as set by v4l2-ctl for example */
                if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
                {
                        cerr << "Error initializing device (VIDIOC_G_FMT)" << endl;
                        return false;
                }
        }

        /* Buggy driver paranoia. */
        min = fmt.fmt.pix.width * 2;
        if (fmt.fmt.pix.bytesperline < min)
                fmt.fmt.pix.bytesperline = min;
        min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
        if (fmt.fmt.pix.sizeimage < min)
                fmt.fmt.pix.sizeimage = min;

        switch (io) {
        case IO_METHOD_READ:
                return init_read(buffers, fmt.fmt.pix.sizeimage);
                break;

        case IO_METHOD_MMAP:
                init_mmap(fd, buffers, n_buffers, fmt.fmt.pix.sizeimage);
                break;

        case IO_METHOD_USERPTR:
                return init_userp(fd, buffers, n_buffers, fmt.fmt.pix.sizeimage);
                break;
        }
        return true;
}

static buffer read_frame(int io, int fd, buffer* buffers, size_t n_buffers)
{
    struct v4l2_buffer buf;
    unsigned int i;
    buffer ret_buffer;
    ret_buffer.length = 0;
    ret_buffer.start = NULL;

    switch (io) {
    case IO_METHOD_READ:
            if (-1 == read(fd, buffers[0].start, buffers[0].length)) {
                    switch (errno) {
                    case EAGAIN:
                            return ret_buffer;

                    case EIO:
                            /* Could ignore EIO, see spec. */

                            /* fall through */

                    default:
                        cerr << "Error reading frame (read)" << endl;
                        return ret_buffer;
                    }
            }
            ret_buffer = buffers[0];
            break;

    case IO_METHOD_MMAP:
            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
                    switch (errno) {
                    case EAGAIN:
                            return ret_buffer;

                    case EIO:
                            /* Could ignore EIO, see spec. */

                            /* fall through */

                    default:
                        cerr << "Error reading frame (VIDIOC_DQBUF)" << endl;
                        return ret_buffer;
                    }
            }

            assert(buf.index < n_buffers);
            ret_buffer = buffers[buf.index];

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) {
                cerr << "Error reading frame (VIDIOC_QBUF)" << endl;
                return ret_buffer;
            }
            break;

    case IO_METHOD_USERPTR:
            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;

            int ret;
            do {
                ret = xioctl(fd, VIDIOC_DQBUF, &buf);
                if (ret == -1 && errno != EAGAIN)
                    break;
            } while (ret == -1);

            if (-1 == ret) {
                 cerr << "Error reading frame (VIDIOC_DQBUF)" << endl;
                 return ret_buffer;
            }

            for (i = 0; i < n_buffers; ++i)
                    if (buf.m.userptr == (unsigned long)buffers[i].start
                        && buf.length == buffers[i].length)
                            break;

            assert(i < n_buffers);

            ret_buffer = buffers[i];

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) {
                cerr << "Error reading frame (VIDIOC_QBUF)" << endl;
                return ret_buffer;
            }
            break;
    }

    return ret_buffer;
}

vector<CameraInfo> CameraV4L::getCameraList()
{
    vector<CameraInfo> ret;

    for (unsigned int i=0; i<16; i++) {
        
        boost::filesystem::path devName(std::string("/dev/video") +
                                        boost::lexical_cast<std::string>(i));

        if (boost::filesystem::exists(devName)) {
            CameraInfo info;
            info.vendor = "V4L";
            info.model = devName.string();
            info.busID = i;
            ret.push_back(info);
        }
    }

    return ret;
}

CameraV4L::CameraV4L(unsigned int camNum) : Camera(triggerModeSoftware)
{
    m_numBuffers = 1;
    m_buffers = NULL;
    m_io = IO_METHOD_USERPTR; //IO_METHOD_READ;
    opened = false;

    std::ostringstream devName;
    devName << "/dev/video";
    devName << camNum;
    m_fd = open_device(devName.str().c_str());

    if (m_fd == -1) {
        cerr << "Error opening V4L camera: " << devName.str() << endl;
        return;
    }

    struct v4l2_format fmt;
    if (!init_device(m_fd, devName.str().c_str(), m_io, &m_buffers, m_numBuffers, fmt)) {
        cerr << "Error inialializing V4L camera: " << devName.str() << endl;
        return;
    }

    m_width = fmt.fmt.pix.width;
    m_height = fmt.fmt.pix.height;
    m_bytes = fmt.fmt.pix.sizeimage;
    opened = true;
}

CameraSettings CameraV4L::getCameraSettings()
{
    CameraSettings settings;

    return settings;
}

static void setV4LCtrl(int fd, unsigned int id, int value, const char* ctrl_id)
{
    struct v4l2_control ctrl;
    CLEAR(ctrl);
    ctrl.id = id;
    ctrl.value = value;
    if (-1 == ioctl(fd, VIDIOC_S_CTRL, &ctrl)) {
        cerr << "Error initializing device (VIDIOC_S_CTRL, id: " << ctrl_id << ")" << endl;
    }
}

void CameraV4L::setCameraSettings(CameraSettings settings)
{
    setV4LCtrl(m_fd, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, "V4L2_CID_EXPOSURE_AUTO");
    setV4LCtrl(m_fd, V4L2_CID_EXPOSURE_ABSOLUTE, settings.shutter * 10.f, "V4L2_CID_EXPOSURE_ABSOLUTE");
    setV4LCtrl(m_fd, V4L2_CID_AUTO_WHITE_BALANCE, 0, "V4L2_CID_AUTO_WHITE_BALANCE");
    setV4LCtrl(m_fd, V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE, V4L2_WHITE_BALANCE_MANUAL, "V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE");
    setV4LCtrl(m_fd, V4L2_CID_ISO_SENSITIVITY, V4L2_ISO_SENSITIVITY_MANUAL, "V4L2_CID_ISO_SENSITIVITY");
    setV4LCtrl(m_fd, V4L2_CID_3A_LOCK, 1, "V4L2_CID_3A_LOCK");
    setV4LCtrl(m_fd, V4L2_CID_AUTOGAIN, 0, "V4L2_CID_AUTOGAIN");
    setV4LCtrl(m_fd, V4L2_CID_POWER_LINE_FREQUENCY, V4L2_CID_POWER_LINE_FREQUENCY_DISABLED, "V4L2_CID_POWER_LINE_FREQUENCY");
    setV4LCtrl(m_fd, V4L2_CID_HUE_AUTO, 0, "V4L2_CID_HUE_AUTO");
    setV4LCtrl(m_fd, V4L2_CID_AUTOBRIGHTNESS, 0, "V4L2_CID_AUTOBRIGHTNESS");
    setV4LCtrl(m_fd, V4L2_CID_FOCUS_AUTO, 0, "V4L2_CID_FOCUS_AUTO");
    setV4LCtrl(m_fd, V4L2_CID_BACKLIGHT_COMPENSATION, 0, "V4L2_CID_BACKLIGHT_COMPENSATION");
    setV4LCtrl(m_fd, V4L2_CID_FOCUS_ABSOLUTE, 0, "V4L2_CID_FOCUS_ABSOLUTE");
    setV4LCtrl(m_fd, V4L2_CID_BRIGHTNESS, 30, "V4L2_CID_BRIGHTNESS");
}

void CameraV4L::startCapture()
{
    if (!opened) {
        cerr << "ERROR: CameraV4L not opened, cannot start capture!" << endl;
        return;
    }

    unsigned int i;
    enum v4l2_buf_type type;

    switch (m_io) {
    case IO_METHOD_READ:
            /* Nothing to do. */
            break;

    case IO_METHOD_MMAP:
            for (i = 0; i < m_numBuffers; ++i) {
                    struct v4l2_buffer buf;

                    CLEAR(buf);
                    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                    buf.memory = V4L2_MEMORY_MMAP;
                    buf.index = i;

                    if (-1 == xioctl(m_fd, VIDIOC_QBUF, &buf)) {
                        cerr << "Error initializing device (VIDIOC_QBUF)" << endl;
                        return;
                    }
            }
            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (-1 == xioctl(m_fd, VIDIOC_STREAMON, &type))
            {
                cerr << "Error initializing device (VIDIOC_STREAMON)" << endl;
                return;
            }
            break;

    case IO_METHOD_USERPTR:
            for (i = 0; i < m_numBuffers; ++i) {
                struct v4l2_buffer buf;

                CLEAR(buf);
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_USERPTR;
                buf.index = i;
                buf.m.userptr = (unsigned long)m_buffers[i].start;
                buf.length = m_buffers[i].length;

                if (-1 == xioctl(m_fd, VIDIOC_QBUF, &buf)) {
                    cerr << "Error initializing device (VIDIOC_QBUF)" << endl;
                    return;
                }
            }
            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (-1 == xioctl(m_fd, VIDIOC_STREAMON, &type)) {
                cerr << "Error initializing device (VIDIOC_STREAMON)" << endl;
                return;
            }
            break;
    }
    capturing = true;
}

void CameraV4L::stopCapture()
{
    if (!capturing)
    {
        std::cerr << "CameraV4L: not capturing!" << std::endl;
        return;
    }
    enum v4l2_buf_type type;

    switch (m_io) {
    case IO_METHOD_READ:
            /* Nothing to do. */
            break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(m_fd, VIDIOC_STREAMOFF, &type))
        {
            cerr << "Error deinitializing device (VIDIOC_STREAMOFF)" << endl;
        }
        break;
    }

    capturing = false;
}

CameraFrame CameraV4L::getFrame(){

    CameraFrame frame;

    if (!capturing) {
        cerr << "ERROR: Not capturing on camera. Call startCapture() before lockFrame()." << endl;
        return frame;
    }

    buffer buf;

    for (int i = 0; i < 2; ++i) {
        buf = read_frame(m_io, m_fd, m_buffers, m_numBuffers);

        if (!buf.start)
        {
            cerr << "ERROR: Did not get a image from the camera." << endl;
            return frame;
        }
    }

    cv::Mat cvframe(m_height, m_width, CV_8UC2,  buf.start);
    cv::cvtColor(cvframe, m_lastImage, cv::COLOR_YUV2GRAY_YUYV);

    // Copy frame address and properties
    frame.memory = m_lastImage.data;
    frame.width = m_lastImage.cols;
    frame.height = m_lastImage.rows;
    frame.sizeBytes = frame.height * m_lastImage.step;

    return frame;
}

size_t CameraV4L::getFrameSizeBytes()
{
    if (!capturing) {
        cerr << "ERROR: Cannot get frame size before capturing. Call startCapture() before getFrameSizeBytes()." << endl;
        return 0;
    }

    return m_bytes;
}

size_t CameraV4L::getFrameWidth()
{
    return m_width;
}


size_t CameraV4L::getFrameHeight()
{
    return m_height;
}

CameraV4L::~CameraV4L()
{
    if (!opened)
        return;
    if (capturing)
        stopCapture();

    uninit_device(m_io, m_buffers, m_numBuffers);

    if (close(m_fd) == -1)
    {
        cerr << "Error closing device." << endl;
    }
}

