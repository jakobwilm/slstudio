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
#include <vector>

#include "CalibrationWorker.h"
#include "Camera.h"
#include "CodecCalibration.h"
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
public slots:
  bool eventFilter(QObject *target, QEvent *event);
private slots:
  void on_snapButton_clicked();
  void on_calibrateButton_clicked();
  void on_listWidget_itemSelectionChanged();
  void on_saveButton_clicked();
  void onNewSequenceResult(const cv::Mat &img, const size_t idx,
                           const bool success);
signals:
  void newCalibrationSaved(CalibrationData _calib);
  void logMessage(const QString &msg);

private:
  Ui::CalibrationDialog *ui;
  std::unique_ptr<Camera> camera;
  std::unique_ptr<Projector> projector;
  std::unique_ptr<EncoderCalibration> encoder;
  CalibrationData calibrationData;
  //  QThread *calibrationWorkerThread;
  CalibrationWorker *calibrationWorker;
  int liveViewTimer;
  std::vector<std::vector<cv::Mat>> frameSeqs;
  std::vector<cv::Mat> seqResults;
  std::vector<size_t> activeFrameSeqs;
  bool reviewMode;
  unsigned int timerInterval; // ms
  unsigned int delay;         // ms
  std::vector<cv::Mat> patterns;
};
