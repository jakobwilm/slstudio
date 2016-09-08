/*
 *  Author: Benjamin Langmann (b.langmann@gmx.de)
 *  Date: 2016
 */

#include "CameraOpenCV.h"

#include <string>
#include <sstream>
#include <boost/timer/timer.hpp>

vector<CameraInfo> CameraOpenCV::getCameraList()
{
    vector<CameraInfo> ret;

    for (unsigned int i=0; i<2; i++) {
        CameraInfo info;
        info.vendor = "OpenCV";
        std::ostringstream s;
        s << i;
        info.model = s.str();
        info.busID = i;
        ret.push_back(info);
    }

    return ret;
}

CameraOpenCV::CameraOpenCV(unsigned int camNum) : Camera(triggerModeSoftware),
    m_bytes(0)
{
    m_devNum = camNum;

    CameraSettings settings;
    setCameraSettings(settings);
}

CameraSettings CameraOpenCV::getCameraSettings()
{
    CameraSettings settings;

    return settings;
}


void CameraOpenCV::setCameraSettings(CameraSettings settings)
{
}

void CameraOpenCV::startCapture()
{
    m_videoCap.open(m_devNum);
    std::cout << "Exposure: " << m_videoCap.get(CV_CAP_PROP_EXPOSURE) << std::endl;
    m_videoCap.set(CV_CAP_PROP_EXPOSURE, 10);
    std::cout << "Exposure: " << m_videoCap.get(CV_CAP_PROP_EXPOSURE) << std::endl;

    capturing = m_videoCap.isOpened();

    if (capturing)
    {
        cv::Mat frame, out;
        m_videoCap >> frame;
        m_grabTimeNS = 0;
        for (int i = 0; i < 30; ++i)
        {
            boost::timer::cpu_timer timer;
            m_videoCap >> frame;
            m_grabTimeNS += timer.elapsed().wall;
        }
        m_grabTimeNS /= 30;
        cout << "Measured grab time: " << m_grabTimeNS << endl;
        cv::cvtColor(frame, out, cv::COLOR_BGR2GRAY);
        m_bytes = out.step * out.rows;
    }
}

void CameraOpenCV::stopCapture()
{
    m_videoCap.release();
    capturing = false;
    m_bytes = 0;
}

CameraFrame CameraOpenCV::getFrame(){

    CameraFrame frame;

    if (!capturing) {
        cerr << "ERROR: Not capturing on camera. Call startCapture() before lockFrame()." << endl;
        return frame;
    }

    cv::Mat cvframe, out;
    for (int i = 0; i < 30; ++i)
    {
        boost::timer::cpu_timer timer;
        m_videoCap.grab();
        boost::timer::nanosecond_type t = timer.elapsed().wall;
        if (t >= 0.3 * m_grabTimeNS) break;
        //cout << "Time to grab frame to short (" << t << "), retaking image ..." << endl;
    }
    bool ret = m_videoCap.retrieve(cvframe);

    if (!ret || cvframe.empty())
    {
        cerr << "ERROR: Did not get a image from the camera." << endl;
        return frame;
    }

    cv::cvtColor(cvframe, out, cv::COLOR_BGR2GRAY);
    // Copy frame address and properties
    frame.memory = out.data;
    frame.width = out.cols;
    frame.height = out.rows;
    frame.sizeBytes = frame.height * out.step;

    return frame;
}

size_t CameraOpenCV::getFrameSizeBytes()
{
    if (!capturing) {
        cerr << "ERROR: Cannot get frame size before capturing. Call startCapture() before getFrameSizeBytes()." << endl;
        return 0;
    }

    return m_bytes;
}

size_t CameraOpenCV::getFrameWidth()
{
    return m_videoCap.get(CV_CAP_PROP_FRAME_WIDTH);
}


size_t CameraOpenCV::getFrameHeight()
{
    return m_videoCap.get(CV_CAP_PROP_FRAME_HEIGHT);
}

CameraOpenCV::~CameraOpenCV()
{
}

