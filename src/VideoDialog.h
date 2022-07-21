/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#pragma once

#include <QDialog>

#include <opencv2/opencv.hpp>

namespace Ui {
class VideoDialog;
}

class VideoDialog : public QDialog {
  Q_OBJECT

public:
  explicit VideoDialog(const QString &title, QWidget *parent = 0);
  ~VideoDialog();
  QAction *toggleViewAction();
  void showEvent(QShowEvent *);
  void closeEvent(QCloseEvent *);
public slots:
  void showImageCV(cv::Mat image);
  void showImageSeqCV(std::vector<cv::Mat> imageSeq);

private:
  Ui::VideoDialog *ui;
  QAction *action;
};
