
#include <iostream>
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

int main(){

    // Get screeninfo for OpenGL projector
    std::vector<ScreenInfo> screenInfo;
    screenInfo = OpenGLContext::GetScreenInfo();
    
    for (unsigned int i=0; i<screenInfo.size(); i++) {
        printf("Screen %d: width %d, height %d\n", i, screenInfo[i].resX, screenInfo[i].resY);
    }
    
//    // Fill stripe texture
//    unsigned char stripeImage[200][200][3];
//    for (int k=0; k<3; k++) {
//        for (int j = 0; j < 200; j++) {
//            for (int i = 0; i < 101; i++) {
//                stripeImage[i][j][k] = 50;
//            }
//            for (int i = 101; i < 200; i++) {
//                stripeImage[i][j][k] = 100;
//            }
//        }
//    }

//    // Fill gray texture
//    unsigned char grayImage[200][200][3];
//    for (int k=0; k<3; k++) {
//        for (int i = 0; i < 200; i++) {
//            for (int j = 0; j < 200; j++) {
//                grayImage[i][j][k] = 50;
//            }
//        }
//    }

    QTime time; time.start();

    ProjectorOpenGL *PP = new ProjectorOpenGL(0);


    unsigned int screenResX, screenResY;
    PP->getScreenRes(&screenResX, &screenResY);

    std::cout << "Project Screen ResX="<<screenResX << ", ResY="<< screenResY << std::endl;

    // set the encoder pattern
    Encoder* encoder = NULL;
    encoder = new EncoderPhaseShift2x3(screenResX, screenResY, CodecDir::CodecDirHorizontal);

    unsigned int N = encoder->getNPatterns();

    // Upload patterns to projector/GPU in full projector resolution
    for(unsigned int i=0; i < N; i++){
        cv::Mat pattern = encoder->getEncodingPattern(i);

        // general repmat
        pattern = cv::repeat(pattern, screenResY/pattern.rows + 1, screenResX/pattern.cols + 1);
        pattern = pattern(cv::Range(0, screenResY), cv::Range(0, screenResX));

        PP->setPattern(i, pattern.ptr(), pattern.cols, pattern.rows);

       cv::imwrite(cv::format("pat_%d.bmp", i), pattern);
    }

    std::cout << "Displaying Patterns..." << std::endl;
    for(unsigned int i = 0; i < N; i++){
        time.restart();
        PP->displayPattern(i);
        unsigned int msecelapsed = time.restart();
        std::cout << msecelapsed << std::endl;
    }

//    std::cout << "Displaying texture" << std::endl;
//    time.restart();
//    PP->displayTexture((unsigned char*)stripeImage, 200, 200);
//    QTest::qSleep(2000);
//    unsigned int msecelapsed = time.restart();
//    std::cout << msecelapsed << std::endl;

//    std::cout << "Timing display texture" << std::endl;
//    time.restart();
//    for(unsigned int i=0; i<500; i++){
//        PP->displayTexture((unsigned char*)stripeImage, 200, 200);
//        unsigned int msecelapsed = time.restart();
//        std::cout << msecelapsed << std::endl;
//    }

//    std::cout << "Displaying as pattern" << std::endl;
//    PP->setPattern(0, (unsigned char*)stripeImage, 200, 200);
//    PP->setPattern(1, (unsigned char*)grayImage, 200, 200);
//    time.restart();
//    for(unsigned int i=0; i<500; i++){
//        PP->displayPattern(0);
//        unsigned int msecelapsed = time.restart();
//        std::cout << msecelapsed << std::endl;
//        PP->displayPattern(1);
//        msecelapsed = time.restart();
//        std::cout << msecelapsed << std::endl;
//    }

    // std::cout << "Displaying white and black..." << std::endl;
    // for(unsigned int i = 0; i<300; i++){
    //     time.restart();
    //     PP->displayWhite();
    //     PP->displayBlack();
    //     unsigned int msecelapsed = time.restart();
    //     std::cout << msecelapsed << std::endl;
    // }

//    std::cout << "Displaying walking dot..." << std::endl;
//    cv::Mat tex(screenInfo[1].resY, screenInfo[1].resX, CV_8U);
//    for(unsigned int c = 0; c<screenInfo[1].resX; c++){
//        tex.setTo(0);
//        tex.col(c).setTo(255);
//        std::vector<cv::Mat> texVector;
//        texVector.push_back(tex);
//        texVector.push_back(tex);
//        texVector.push_back(tex);
//        cv::Mat texRGB;
//        cv::merge(texVector, texRGB);
//        PP->displayTexture(texRGB.data, texRGB.cols, texRGB.rows);
//        QTest::qSleep(100);
//    }
//    for(unsigned int r = 0; r<screenInfo[1].resY; r+=10){
//        tex.setTo(0);
//        tex.row(r).setTo(255);
//        std::vector<cv::Mat> texVector;
//        texVector.push_back(tex);
//        texVector.push_back(tex);
//        texVector.push_back(tex);
//        cv::Mat texRGB;
//        cv::merge(texVector, texRGB);
////        cvtools::writeMat(texRGB, "texRGB.mat");
//        PP->displayTexture(texRGB.data, texRGB.cols, texRGB.rows);
//        QTest::qSleep(10000);
//    }

//    std::cout << "Defining pattern sequence..." << std::endl;
//    PP->setPattern(0, (unsigned char*)stripeImage, 200, 200);
//    PP->setPattern(1, (unsigned char*)grayImage, 200, 200);
//    std::cout << "Cycling through pattern sequence..." << std::endl;
//    for(unsigned int i = 0; i<10000; i++){
//        time.restart();
//        PP->displayPattern(i%2);
//        //QTest::qSleep(500);
//        unsigned int msecelapsed = time.restart();
//        std::cout << msecelapsed << std::endl;
//    }

    delete PP;
    delete encoder;
    return 0;
}

