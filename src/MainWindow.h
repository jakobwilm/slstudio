/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#pragma once

#include <QElapsedTimer>
#include <QMainWindow>
#include <QPointer>
#include <QSettings>

#include "CodecFactory.h"
#include "LogDialog.h"
#include "ScanWorker.h"
#include "VideoDialog.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  void closeEvent(QCloseEvent *event);
  ~MainWindow();

private slots:

  void onActionStart();
  void onActionStop();
  void onScanWorkerFinished();

  void onActionCalibration();
  void onActionLoadCalibration();
  void onActionPreferences();
  void onActionExportCalibration();

  void updateDisplayRate();
  void receiveNewPointCloud(PointCloudConstPtr pointCloud);

  void imshow(const char *windowName, cv::Mat im, unsigned int x,
              unsigned int y);
  void hist(const char *windowName, cv::Mat im, unsigned int x, unsigned int y);

  void onShowHistogram(cv::Mat im);
  void onShowShading(cv::Mat im);
  void onShowCameraFrames(std::vector<cv::Mat> frameSeq);
  void onShowDecoderUp(cv::Mat im);
  void onShowDecoderVp(cv::Mat im);

  void onActionAbout();

  void on_actionUpload_Scan_Patterns_triggered();

  void on_actionUpload_Calibration_Patterns_triggered();

signals:
  void newPointCloud(PointCloudConstPtr pointCloud);
  void logMessage(const QString &msg);

private:
  Ui::MainWindow *ui;
  std::vector<unsigned int> displayIntervals;

  ScanWorker *scanWorker;
  QThread *scanWorkerThread;

  DecoderWorker *decoderWorker;
  QThread *decoderThread;

  TriangulatorWorker *triangulatorWorker;
  QThread *triangulatorThread;

  std::unique_ptr<QElapsedTimer> timer;
  QSettings *settings;

  VideoDialog *histogramDialog, *shadingDialog, *cameraFramesDialog,
      *decoderUpDialog, *decoderVpDialog;

  LogDialog *logDialog;

public:
};
