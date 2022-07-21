/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#pragma once

#include <QLabel>
#include <QResizeEvent>

#include "Camera.h"
#include <opencv2/opencv.hpp>

class VideoWidget : public QLabel {
  Q_OBJECT
public:
  explicit VideoWidget(QWidget *parent = 0) : QLabel(parent) {}
signals:

public slots:
  void showFrame(CameraFrame frame);
  void showFrameCV(cv::Mat frame);
  void resizeEvent(QResizeEvent *event);

protected:
private:
  QPixmap pixmap;
};
