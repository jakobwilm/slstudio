/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#pragma once

#include <QDialog>
#include <QFutureWatcher>
#include <QListWidgetItem>
#include <QModelIndex>
#include <memory>

#include "Calibrator.h"
#include "Camera.h"
#include "MainWindow.h"
#include "Projector.h"

namespace Ui {
class CalibrationDialog;
}

class CalibrationDialog : public QDialog {
  Q_OBJECT

public:
  explicit CalibrationDialog(MainWindow *parent = 0);
  ~CalibrationDialog();
  void timerEvent(QTimerEvent *event);
  void closeEvent(QCloseEvent *);
private slots:
  void on_snapButton_clicked();
  void on_calibrateButton_clicked();
  void on_listWidget_itemSelectionChanged();
  void on_saveButton_clicked();
  void onNewSequenceResult(cv::Mat img, unsigned int idx, bool success);
signals:
  void newCalibrationSaved(CalibrationData _calib);
  void logMessage(const QString &msg);

private:
  Ui::CalibrationDialog *ui;
  std::unique_ptr<Camera> camera;
  std::unique_ptr<Projector> projector;
  Calibrator *calibrator;
  CalibrationData calib;
  int liveViewTimer;
  vector<vector<cv::Mat>> frameSeqs;
  vector<unsigned int> activeFrameSeqs;
  bool reviewMode;
  unsigned int timerInterval; // ms
  unsigned int delay;         // ms
  unsigned int screenCols;
  unsigned int screenRows;
  std::vector<cv::Mat> patterns;
};
