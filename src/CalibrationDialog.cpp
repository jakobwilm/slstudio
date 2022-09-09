#include "CalibrationDialog.h"
#include "ui_CalibrationDialog.h"

#include "ProjectorOpenGL.h"

#include <QCollator>
#include <QDateTime>
#include <QFileDialog>
#include <QFuture>
#include <QSettings>

#include <opencv2/opencv.hpp>

#include "CameraFactory.h"
#include "CameraVirtual.h"
#include "ProjectorFactory.h"

#include "cvtools.h"

CalibrationDialog::CalibrationDialog(MainWindow *parent)
    : QDialog(parent), ui(new Ui::CalibrationDialog), reviewMode(false),
      timerInterval(100), delay(100) {
  ui->setupUi(this);
  setSizeGripEnabled(false);

  // Release this dialog on close
  this->setAttribute(Qt::WA_DeleteOnClose);

  qApp->installEventFilter(this);

  QSettings settings("SLStudio");

  // Calibration grid parameters
  ui->spacingSpinBox->setValue(
      settings.value("calibration/spacingSpinBox", 17).toFloat());
  ui->rowsSpinBox->setValue(
      settings.value("calibration/rowsSpinBox", 15).toInt());
  ui->colsSpinBox->setValue(
      settings.value("calibration/colsSpinBox", 20).toInt());
  ui->patternComboBox->setCurrentText(
      settings.value("calibration/pattern", "circles").toString());

  // Instantiate camera with software trigger
  int iNum = settings.value("camera/interfaceNumber", 0).toInt();
  int cNum = settings.value("camera/cameraNumber", 0).toInt();

  camera = CameraFactory::NewCamera(iNum, cNum, triggerModeSoftware);

  if (camera == nullptr) {
    emit logMessage("CalibrationDialog: could not open camera.");
  }

  delay = settings.value("trigger/delay", "100").toInt();

  // Set camera settings
  CameraSettings camSettings;
  camSettings.shutter = settings.value("camera/shutter", 16.666).toFloat();
  camSettings.gain = 0.0;
  if (camera != nullptr) {
    camera->setCameraSettings(camSettings);
    camera->startCapture();
  }

  // Initialize projector
  int screenNum = settings.value("projector/screenNumber", -1).toInt();
  projector = ProjectorFactory::NewProjector(screenNum);

  if (projector == nullptr) {
    emit logMessage("CalibrationDialog: could not create projector.");
  } else {
    projector->displayWhite();
  }

  unsigned int screenResX = 1000, screenResY = 1000;
  if (projector != nullptr) {
    projector->getScreenRes(&screenResX, &screenResY);
    encoder.reset(new EncoderCalibration(screenResX, screenResY, CodecDirBoth));
  }

  // Calibration worker
  calibrationWorker = new CalibrationWorker(this, screenResX, screenResY);
  //  calibrationWorkerThread = new QThread(this);
  //  calibrationWorkerThread->setObjectName("calibrationWorkerThread");
  //  calibrationWorker->moveToThread(calibrationWorkerThread);

  connect(calibrationWorker, &CalibrationWorker::newSequenceResult, this,
          &CalibrationDialog::onNewSequenceResult);

  connect(calibrationWorker, &CalibrationWorker::logMessage, this,
          &CalibrationDialog::logMessage);

  if (projector && !projector->requiresPatternUpload()) {
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

  if (!camera || !projector) {
    return;
  }

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

  if (!camera || !projector) {
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

  // Save calibration settings
  QSettings settings;
  settings.setValue("calibration/spacing", ui->spacingSpinBox->value());
  settings.setValue("calibration/rows", ui->rowsSpinBox->value());
  settings.setValue("calibration/cols", ui->colsSpinBox->value());
  settings.setValue("calibration/pattern", ui->patternComboBox->currentText());

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

  int idx = ui->listWidget->currentRow();
  if (seqResults.size() > idx) {
    ui->videoWidget->showFrameCV(seqResults[idx]);
  } else {
    ui->videoWidget->showFrameCV(frameSeqs[idx][0]);
  }
}

bool CalibrationDialog::eventFilter(QObject *target, QEvent *event) {

  if (event->type() == QEvent::KeyPress) {
    QKeyEvent *e = static_cast<QKeyEvent *>(event);

    // import calibration frame sequences
    if (e->key() == Qt::Key_I) {

      QString path = QFileDialog::getExistingDirectory(
          nullptr, "Import calibration frame sequences",
          QCoreApplication::applicationDirPath());
      if (path.isNull()) {
        return QDialog::eventFilter(target, event);
      }

      QDir dirs(path, "seq_*", QDir::NoSort, QDir::Dirs);
      QStringList dirNames = dirs.entryList();

      QCollator collator;
      collator.setNumericMode(true);

      // sort according to natural ordering
      std::sort(dirNames.begin(), dirNames.end(), collator);

      for (auto &d : dirNames) {

        QDir dir(QDir(path).filePath(d));

        QStringList filenames =
            dir.entryList(QStringList("patternFrames_*.png"));

        // sort according to natural ordering
        std::sort(filenames.begin(), filenames.end(), collator);

        std::vector<cv::Mat> patternFrames;

        for (const auto &f : filenames) {
          //        std::cout << QDir(dir).filePath(f).toStdString() <<
          //        std::endl;
          patternFrames.push_back(cv::imread(
              QDir(dir).filePath(f).toStdString(), cv::IMREAD_GRAYSCALE));
        }

        //      cv::Mat textureFrame =
        //          cv::imread(QDir(path).filePath("textureFrame.png").toStdString());

        // Store frame sequence
        frameSeqs.push_back(patternFrames);

        // Add identifier to list
        QListWidgetItem *item = new QListWidgetItem(
            QString("Sequence %1").arg(frameSeqs.size()), ui->listWidget);
        item->setFlags(item->flags() |
                       Qt::ItemIsUserCheckable); // set checkable
        item->setCheckState(Qt::Checked);        // AND initialize check state
      }
      if (frameSeqs.size() > 0) {
        ui->calibrateButton->setEnabled(true);
      }

      return true;
    }

    // export all calibration frame sequences
    if (e->key() == Qt::Key_E) {

      QString path = QFileDialog::getExistingDirectory(
          nullptr, "Export calibration frame sequences",
          QCoreApplication::applicationDirPath());
      if (path.isNull()) {
        return QDialog::eventFilter(target, event);
      }

      for (int i = 0; i < frameSeqs.size(); i++) {
        QDir seqDir(path);
        seqDir.mkdir(QString("seq_%1").arg(i));
        for (int j = 0; j < frameSeqs[i].size(); j++) {

          QString fileName = QDir(path).filePath(
              QString("seq_%1/patternFrames_%2.png").arg(i).arg(j));
          cv::imwrite(fileName.toStdString(), frameSeqs[i][j]);
        }
      }

      return true;
    }
  }

  return QDialog::eventFilter(target, event);
}

void CalibrationDialog::onNewSequenceResult(const cv::Mat &img,
                                            const size_t idx,
                                            const bool success) {

  if (seqResults.size() <= idx) {
    seqResults.push_back(img);
  } else {
    seqResults[idx] = img;
  }

  if (!success) // uncheck
    ui->listWidget->item(idx)->setCheckState(Qt::Unchecked);

  // Highlight
  ui->listWidget->setCurrentRow(idx);
  ui->listWidget->setFocus();

  QApplication::processEvents();
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

void CalibrationDialog::closeEvent(QCloseEvent *) { this->deleteLater(); }
