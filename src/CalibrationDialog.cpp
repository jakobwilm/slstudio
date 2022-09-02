#include "CalibrationDialog.h"
#include "ui_CalibrationDialog.h"

#include "ProjectorOpenGL.h"

#include <QDateTime>
#include <QFuture>
#include <QSettings>

#include <opencv2/opencv.hpp>

#include "CameraFactory.h"
#include "ProjectorFactory.h"

#include "cvtools.h"

CalibrationDialog::CalibrationDialog(MainWindow *parent)
    : QDialog(parent), ui(new Ui::CalibrationDialog), reviewMode(false),
      timerInterval(100), delay(100) {
  ui->setupUi(this);
  setSizeGripEnabled(false);

  // Release this dialog on close
  this->setAttribute(Qt::WA_DeleteOnClose);

  QSettings settings("SLStudio");

  // Calibration grid parameters
  ui->spacingSpinBox->setValue(
      settings.value("calibration/spacingSpinBox", 0.004).toFloat());
  ui->rowsSpinBox->setValue(
      settings.value("calibration/rowsSpinBox", 10).toInt());
  ui->colsSpinBox->setValue(
      settings.value("calibration/colsSpinBox", 15).toInt());
  ui->patternComboBox->setCurrentText(
      settings.value("calibration/pattern", "circles").toString());

  // Instantiate camera with software trigger
  int iNum = settings.value("camera/interfaceNumber", 0).toInt();
  int cNum = settings.value("camera/cameraNumber", 0).toInt();
  camera = CameraFactory::NewCamera(iNum, cNum, triggerModeSoftware);

  delay = settings.value("trigger/delay", "100").toInt();

  // Set camera settings
  CameraSettings camSettings;
  camSettings.shutter = settings.value("camera/shutter", 16.666).toFloat();
  camSettings.gain = 0.0;
  camera->setCameraSettings(camSettings);
  camera->startCapture();

  // Initialize projector
  int screenNum = settings.value("projector/screenNumber", -1).toInt();
  projector = ProjectorFactory::NewProjector(screenNum);
  projector->displayWhite();

  if (projector == nullptr) {
    emit logMessage("CalibrationDialog: could not create projector.");
  }

  unsigned int screenResX, screenResY;
  projector->getScreenRes(&screenResX, &screenResY);

  // Encoder
  encoder.reset(new EncoderCalibration(screenResX, screenResY, CodecDirBoth));

  // Calibration worker
  calibrationWorker = new CalibrationWorker(this, screenResX, screenResY);
  calibrationWorkerThread = new QThread(this);
  calibrationWorkerThread->setObjectName("calibrationWorkerThread");
  calibrationWorker->moveToThread(calibrationWorkerThread);

  connect(calibrationWorker, &CalibrationWorker::newSequenceResult, this,
          &CalibrationDialog::onNewSequenceResult);

  connect(calibrationWorker, &CalibrationWorker::logMessage, this,
          &CalibrationDialog::logMessage);

  if (!projector->requiresPatternUpload()) {
    // Upload patterns to projector/GPU
    patterns.resize(encoder->getNPatterns());
    std::vector<const uchar *> patternPtrs(encoder->getNPatterns());
    for (unsigned int i = 0; i < encoder->getNPatterns(); i++) {
      patterns[i] = encoder->getEncodingPattern(i);

      // general repmat
      patterns[i] = cv::repeat(patterns[i], screenResY / patterns[i].rows + 1,
                               screenResX / patterns[i].cols + 1);
      patterns[i] =
          patterns[i](cv::Range(0, screenResY), cv::Range(0, screenResX));

      patternPtrs[i] = patterns[i].data;
    }

    projector->setPatterns(patternPtrs, patterns[0].cols, patterns[0].rows);
  }

  // Start live view
  timerInterval = delay + camSettings.shutter;
  liveViewTimer = startTimer(timerInterval);
}

void CalibrationDialog::timerEvent(QTimerEvent *event) {

  if (event->timerId() != liveViewTimer) {
    std::cerr << "Something fishy..." << std::endl << std::flush;
    return;
  }

  QApplication::processEvents();
  CameraFrame frame = camera->getFrame();

  cv::Mat frameCV(frame.height, frame.width, CV_8U, frame.memory);
  frameCV = frameCV.clone();
  //    cv::resize(frameCV, frameCV, cv::Size(0, 0), 0.5, 0,5);

  ui->videoWidget->showFrameCV(frameCV);

  QApplication::processEvents();
}

CalibrationDialog::~CalibrationDialog() { delete ui; }

void CalibrationDialog::on_snapButton_clicked() {

  // If in review mode
  if (reviewMode) {
    reviewMode = false;
    ui->listWidget->clearSelection();
    liveViewTimer = startTimer(timerInterval);
    ui->snapButton->setText("Snap");
    return;
  }

  ui->snapButton->setEnabled(false);

  // Stop live view
  killTimer(liveViewTimer);

  std::vector<cv::Mat> frameSeq;

  for (unsigned int i = 0; i < encoder->getNPatterns(); i++) {

    // Project pattern
    projector->displayPattern(i);
    QThread::msleep(delay);

    // Effectuate sleep (necessary with some camera implementations)
    QApplication::processEvents();

    // Aquire frame
    CameraFrame frame = camera->getFrame();
    cv::Mat frameCV(frame.height, frame.width, CV_8U, frame.memory);
    frameCV = frameCV.clone();
    //        cv::resize(frameCV, frameCV, cv::Size(0, 0), 0.5, 0,5);

    // Show frame
    ui->videoWidget->showFrameCV(frameCV);

    // Save frame
    frameSeq.push_back(frameCV);
  }

  // Store frame sequence
  frameSeqs.push_back(frameSeq);

  // Add identifier to list
  QListWidgetItem *item = new QListWidgetItem(
      QString("Sequence %1").arg(frameSeqs.size()), ui->listWidget);
  item->setFlags(item->flags() | Qt::ItemIsUserCheckable); // set checkable flag
  item->setCheckState(Qt::Checked); // AND initialize check state

  //    // Allow calibration if enough frame pairs
  //    if(ui->listWidget->count() >= 3)
  ui->calibrateButton->setEnabled(true);

  // Display white
  projector->displayWhite();

#if 0
        // Write frame seq to disk
        for(unsigned int i=0; i<frameSeq.size(); i++){
            QString filename = QString("frameSeq_%1.bmp").arg(i, 2, 10, QChar('0'));
            cv::imwrite(filename.toStdString(), frameSeq[i]);
        }
#endif

  // Restart live view
  liveViewTimer = startTimer(timerInterval);

  ui->snapButton->setEnabled(true);
}

static void plotResiduals(const std::vector<std::vector<cv::Point3f>> Q,
                          const std::vector<std::vector<cv::Point2f>> q,
                          const cv::Matx33f K, const cv::Vec<float, 5> k,
                          const std::vector<cv::Mat> rvecs,
                          const std::vector<cv::Mat> tvecs) {

  //  // per-view reprojection errors:
  //  std::vector<float> pve(Q.size());
  //  pve.resize(Q.size());

  // reprojection errors:
  std::vector<cv::Point2d> res;

  for (size_t i = 0; i < Q.size(); ++i) {

    std::vector<cv::Point2f> qProj;
    cv::projectPoints(cv::Mat(Q[i]), rvecs[i], tvecs[i], K, k, qProj);
    //    float err = 0;
    for (unsigned int j = 0; j < qProj.size(); j++) {
      cv::Point2f d = q[i][j] - qProj[j];
      res.push_back(d);
      //      err += cv::sqrt(d.x * d.x + d.y * d.y);
    }
  }

  double minX = -10.0;
  double maxX = 10.0;
  double minY = -10.0;

  cv::Mat resImage =
      cvtools::plotScatterXY(res, cv::Size(1024, 1024), minX, maxX, minY);

  cv::imwrite("resImage.png", resImage);
}

void CalibrationDialog::on_calibrateButton_clicked() {

  // Disable interface elements
  ui->calibrateButton->setEnabled(false);
  ui->listWidget->setEnabled(false);

  // Stop live view
  killTimer(liveViewTimer);
  reviewMode = true;
  ui->snapButton->setText("Live View");

  // Note which frame sequences are used
  activeFrameSeqs.clear();

  for (int i = 0; i < ui->listWidget->count(); i++) {
    if (ui->listWidget->item(i)->checkState() == Qt::Checked) {
      activeFrameSeqs.push_back(i);
    }
  }

  // Perform calibration
  calibrationWorker->calibrate(calibrationData, frameSeqs, activeFrameSeqs);

  // Re-enable interface elements
  ui->calibrateButton->setEnabled(true);
  ui->listWidget->setEnabled(true);
  ui->saveButton->setEnabled(true);
}

void CalibrationDialog::on_listWidget_itemSelectionChanged() {

  // If selection was cleared
  if (ui->listWidget->selectedItems().isEmpty())
    return;

  // Stop live view
  killTimer(liveViewTimer);
  reviewMode = true;
  ui->snapButton->setText("Live View");

  int currentRow = ui->listWidget->currentRow();
  ui->videoWidget->showFrameCV(frameSeqs[currentRow].back());
}

void CalibrationDialog::on_saveButton_clicked() {

  calibrationData.frameWidth = camera->getFrameWidth();
  calibrationData.frameHeight = camera->getFrameHeight();
  unsigned int screenResX, screenResY;
  projector->getScreenRes(&screenResX, &screenResY);
  calibrationData.screenResX = screenResX;
  calibrationData.screenResY = screenResY;
  calibrationData.calibrationDateTime = QDateTime::currentDateTime()
                                            .toString("DD.MM.YYYY HH:MM:SS")
                                            .toStdString();

  calibrationData.save("calibration.xml");
  this->close();
}

void CalibrationDialog::onNewSequenceResult(cv::Mat img, unsigned int idx,
                                            bool success) {

  // Skip non-active frame sequences
  int idxListView = activeFrameSeqs[idx];

  // Append calibration result to frame sequence
  unsigned int N = encoder->getNPatterns();
  if (frameSeqs[idxListView].size() == N)
    frameSeqs[idxListView].push_back(img);
  else
    frameSeqs[idxListView][N] = img;

  if (!success) // uncheck
    ui->listWidget->item(idxListView)->setCheckState(Qt::Unchecked);

  // Highlight
  ui->listWidget->setCurrentRow(idxListView);
  ui->listWidget->setFocus();
}

void CalibrationDialog::closeEvent(QCloseEvent *) {

  this->deleteLater();

  // Save calibration settings
  QSettings settings;
  settings.setValue("calibration/spacing", ui->spacingSpinBox->value());
  settings.setValue("calibration/rows", ui->rowsSpinBox->value());
  settings.setValue("calibration/cols", ui->colsSpinBox->value());
  settings.setValue("calibration/pattern", ui->patternComboBox->currentText());
}
