
#include <iostream>
//#include <unistd.h>
#include <QTest>
#include <QTime>
#include <stdio.h>

#include "OpenGLContext.h"
#include "ProjectorLC4500.h"
#include "ProjectorOpenGL.h"

#include "../cvtools.h"
#include <opencv2/opencv.hpp>

int main() {

  //    // Get screeninfo for OpenGL projector
  //    std::vector<ScreenInfo> screenInfo;
  //    screenInfo = OpenGLContext::GetScreenInfo();

  //    for (unsigned int i=0; i<screenInfo.size(); i++) {
  //        printf("Screen %d: width %d, height %d\n", i, screenInfo[i].resX,
  //        screenInfo[i].resY);
  //    }

  //  // Fill stripe texture 1
  //  cv::Mat_<cv::Vec3b> stripeImage1(1140, 1824, {0, 0, 0});
  //  for (int r = 0; r < stripeImage1.rows; r += 4) {
  //    for (int c = 0; c < stripeImage1.cols; ++c) {
  //      stripeImage1[r][c] = {255, 255, 255};
  //    }
  //  }

  //  // Fill stripe texture 2
  //  cv::Mat_<cv::Vec3b> stripeImage2(1140, 1824, {0, 0, 0});
  //  for (int r = 0; r < stripeImage2.rows; ++r) {
  //    for (int c = 0; c < stripeImage2.cols; c += 4) {
  //      stripeImage2[r][c] = {255, 255, 255};
  //      stripeImage2[r][c + 1] = {255, 255, 255};
  //    }
  //  }

  //  cv::imwrite("stripeImage1.bmp", stripeImage1);
  //  cv::imwrite("stripeImage2.bmp", stripeImage2);

  QTime time;
  time.start();

  ProjectorLC4500 *PP = new ProjectorLC4500(1);

  //  time.restart();
  //  PP->setPatterns({stripeImage1.data, stripeImage2.data}, 912, 1140);
  //  std::cout << time.restart() << std::endl;

  //  PP->displayPattern(0);
  //  std::cout << time.restart() << std::endl;
  //  QTest::qSleep(2000);

  //  time.restart();
  //  PP->displayPattern(1);
  //  std::cout << time.restart() << std::endl;
  QTest::qSleep(2000);

  for (int i = 0; i < 12; ++i) {
    PP->displayPattern(i);
  }
  PP->displayWhite();
  QTest::qSleep(2000);
  PP->displayBlack();
  QTest::qSleep(2000);
  PP->displayWhite();
  QTest::qSleep(2000);
  PP->displayBlack();

  for (int i = 0; i < 12; ++i) {
    PP->displayPattern(i);
  }
  PP->displayWhite();
  QTest::qSleep(2000);
  PP->displayBlack();
  QTest::qSleep(2000);
  //  PP->displaySequence(6, 500000);
  //  QTest::qSleep(5000);
  //  PP->displaySequence(6, 50000);
  //  QTest::qSleep(5000);
  //  PP->displaySequence(6, 16666);
  //  QTest::qSleep(5000);
  //  PP->displayPattern(0);
  //  PP->displayPattern(11);
  //  PP->displayPattern(3);

  //  std::cout << "Displaying white and black..." << std::endl;
  //  for (unsigned int i = 0; i < 300; i++) {
  //    time.restart();
  //    PP->displayWhite();
  //    QTest::qSleep(50);
  //    PP->displayBlack();
  //    QTest::qSleep(50);
  //    unsigned int msecelapsed = time.restart();
  //    std::cout << msecelapsed << std::endl;
  //  }

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
  return 0;
}
