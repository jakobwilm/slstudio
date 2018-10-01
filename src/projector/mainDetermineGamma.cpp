
#include <iostream>
#include <sstream>
#include <fstream>
//#include <unistd.h>
#include <stdio.h>
#include <QTime>
#include <QTest>

#include "OpenGLContext.h"
#include "ProjectorOpenGL.h"

#include <opencv2/opencv.hpp>
#include "../cvtools.h"
#include "../codec/Codec.h"
#include "../codec/CodecPhaseShift2x3.h"

#include "Camera.h"

int main(){

    // Get screeninfo for OpenGL projector
    std::vector<ScreenInfo> screenInfo;
    screenInfo = OpenGLContext::GetScreenInfo();
    
    for (unsigned int i=0; i<screenInfo.size(); i++) {
        printf("Screen %d: width %d, height %d\n", i, screenInfo[i].resX, screenInfo[i].resY);
    }
    

    QTime time; time.start();

    ProjectorOpenGL *PP = new ProjectorOpenGL(0);


    unsigned int screenResX, screenResY;
    PP->getScreenRes(&screenResX, &screenResY);

    std::cout << "Project Screen ResX="<<screenResX << ", ResY="<< screenResY << std::endl;

    // open camera
    Camera* camera = Camera::NewCamera(1, 1, triggerModeHardware);

    std::vector<int> in;
    std::vector<double> out;
    cv::Scalar mean, stddev;
    CameraFrame frame;

    camera->startCapture();

    for (int i=0; i < 255; i+=20 )
    {
        if(i >= 255) break;

        cv::Mat pattern = cv::Mat(200, 200, CV_8UC3, cv::Scalar(i, i, i));
        PP->displayTexture((unsigned char*)pattern.ptr(), pattern.cols, pattern.rows);
        QTest::qSleep(100);
        frame = camera->getFrame();
        cv::Mat frameCV(frame.height, frame.width, frame.type, frame.memory);
        frameCV = frameCV.clone();

        cv::meanStdDev(frameCV, mean, stddev);

        in.push_back(i);
        out.push_back(mean[0]);
    }

    camera->stopCapture();

    // output result

    std::ofstream outFile("gamma_test.txt");

    std::cout << "Gamma test result: " << std::endl;
    std::cout << "Input_I\tOutput_I" << std::endl;
    for (size_t i = 0; i < in.size(); i++)
    {
        std::cout << in[i] << "\t" << out[i] << std::endl;
        outFile << in[i] << "\t" << out[i] << std::endl;
    }

    outFile.close();

    delete PP;
    delete camera;

    return 0;
}

