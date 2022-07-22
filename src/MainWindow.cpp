#include "MainWindow.h"
#include "ui_MainWindow.h"

#include <QFileDialog>
#include <QThread>
#include <QtConcurrent/QtConcurrentRun>
#include <stdio.h>
#include <string.h>

#include "AboutDialog.h"
#include "CalibrationDialog.h"
#include "PreferenceDialog.h"

#include "VideoWidget.h"

#include "CodecCalibration.h"
#include "CodecFactory.h"
#include "ProjectorLC4500.h"

#include <QtGui>

#include "cvtools.h"

using namespace std;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), scanWorkerThread(NULL),
      settings(new QSettings(this)),
      histogramDialog(new VideoDialog("Histogram", this)),
      shadingDialog(new VideoDialog("Shading", this)),
      cameraFramesDialog(new VideoDialog("Camera Frames", this)),
      decoderUpDialog(new VideoDialog("Decoder Up", this)),
      decoderVpDialog(new VideoDialog("Decoder Vp", this)),
      logDialog(new LogDialog(this)) {

  ui->setupUi(this);

  timer = std::make_unique<QElapsedTimer>();

  // Restore main window geometry
  settings = new QSettings("SLStudio");
  restoreGeometry(settings->value("geometry/mainwindow").toByteArray());
  restoreState(settings->value("state/mainwindow").toByteArray());

  // Ui connections
  connect(ui->pointCloudWidget, SIGNAL(newPointCloudDisplayed()), this,
          SLOT(updateDisplayRate()));
  connect(this, &MainWindow::logMessage, logDialog, &LogDialog::onlogMessage);

  // Add view menu actions
  ui->menuView->addAction(histogramDialog->toggleViewAction());
  ui->menuView->addAction(shadingDialog->toggleViewAction());
  ui->menuView->addAction(cameraFramesDialog->toggleViewAction());
  ui->menuView->addAction(decoderUpDialog->toggleViewAction());
  ui->menuView->addAction(decoderVpDialog->toggleViewAction());
  ui->menuView->addAction(logDialog->toggleViewAction());

  // Restore Geometry
  logDialog->restoreGeometry(settings->value("geometry/log").toByteArray());
  histogramDialog->restoreGeometry(
      settings->value("geometry/histogram").toByteArray());
  shadingDialog->restoreGeometry(
      settings->value("geometry/shading").toByteArray());
  cameraFramesDialog->restoreGeometry(
      settings->value("geometry/cameraFrames").toByteArray());
  decoderUpDialog->restoreGeometry(
      settings->value("geometry/decoderUp").toByteArray());
  decoderVpDialog->restoreGeometry(
      settings->value("geometry/decoderVp").toByteArray());

  // Restore Visibility
  logDialog->setVisible(settings->value("visible/log", false).toBool());
  histogramDialog->setVisible(
      settings->value("visible/histogram", false).toBool());
  shadingDialog->setVisible(settings->value("visible/shading", false).toBool());
  cameraFramesDialog->setVisible(
      settings->value("visible/cameraFrames", false).toBool());
  decoderUpDialog->setVisible(
      settings->value("visible/decoderUp", false).toBool());
  decoderVpDialog->setVisible(
      settings->value("visible/decoderVp", false).toBool());
}

void MainWindow::onShowHistogram(cv::Mat im) {
  if (histogramDialog->isVisible())
    histogramDialog->showImageCV(im);
}

void MainWindow::onShowShading(cv::Mat im) {
  if (shadingDialog->isVisible())
    shadingDialog->showImageCV(im);
}

void MainWindow::onShowCameraFrames(std::vector<cv::Mat> frameSeq) {
  if (cameraFramesDialog->isVisible())
    cameraFramesDialog->showImageSeqCV(frameSeq);
}

void MainWindow::onShowDecoderUp(cv::Mat im) {
  if (decoderUpDialog->isVisible())
    decoderUpDialog->showImageCV(im);
}

void MainWindow::onShowDecoderVp(cv::Mat im) {
  if (decoderVpDialog->isVisible()) {
    decoderVpDialog->showImageCV(im);
  }
}

void MainWindow::onActionStart() {

  QSettings settings("SLStudio");
  int iNum = settings.value("camera/interfaceNumber", -1).toInt();
  if (iNum == -1) {
    QString path = QFileDialog::getExistingDirectory(
        nullptr, "Load image files",
        settings.value("virtualCameraPath").toString());
    QDir dir(path);
    if (dir.exists()) {
      settings.setValue("virtualCameraPath", path);
    }
  }

  // Prepare scanWorker on separate thread
  scanWorker = new ScanWorker(this);
  scanWorkerThread = new QThread(this);
  scanWorkerThread->setObjectName("scanWorkerThread");
  scanWorker->moveToThread(scanWorkerThread);
  connect(scanWorker, &ScanWorker::finished, this,
          &MainWindow::onScanWorkerFinished);

  // Prepare decoderWorker on separate thread
  decoderWorker = new DecoderWorker();
  decoderThread = new QThread(this);
  decoderThread->setObjectName("decoderThread");
  decoderWorker->moveToThread(decoderThread);
  connect(decoderThread, &QThread::started, decoderWorker,
          &DecoderWorker::setup);
  connect(decoderThread, &QThread::finished, decoderWorker,
          &QObject::deleteLater);

  // Prepare triangulatorWorker on separate thread
  triangulatorWorker = new TriangulatorWorker();
  triangulatorThread = new QThread(this);
  triangulatorThread->setObjectName("triangulatorThread");
  triangulatorWorker->moveToThread(triangulatorThread);
  connect(triangulatorThread, SIGNAL(started()), triangulatorWorker,
          SLOT(setup()));
  connect(triangulatorThread, SIGNAL(finished()), triangulatorWorker,
          SLOT(deleteLater()));

  // Register metatypes
  qRegisterMetaType<cv::Mat>("cv::Mat");
  qRegisterMetaType<std::vector<cv::Mat>>("std::vector<cv::Mat>");
  qRegisterMetaType<PointCloudConstPtr>("PointCloudConstPtr");
  qRegisterMetaType<QTextBlock>("QTextBlock");
  qRegisterMetaType<QTextCursor>("QTextCursor");

  // Inter thread connections
  connect(scanWorker, &ScanWorker::showHistogram, this,
          &MainWindow::onShowHistogram);
  connect(scanWorker, &ScanWorker::newFrameSeq, decoderWorker,
          &DecoderWorker::decodeSequence);
  connect(scanWorker, &ScanWorker::newFrameSeq, this,
          &MainWindow::onShowCameraFrames);
  connect(scanWorker, &ScanWorker::logMessage, logDialog,
          &LogDialog::onlogMessage);
  connect(decoderWorker, &DecoderWorker::showShading, this,
          &MainWindow::onShowShading);
  connect(decoderWorker, &DecoderWorker::showDecoderUp, this,
          &MainWindow::onShowDecoderUp);
  connect(decoderWorker, &DecoderWorker::showDecoderVp, this,
          &MainWindow::onShowDecoderVp);
  connect(decoderWorker, &DecoderWorker::newUpVp, triangulatorWorker,
          &TriangulatorWorker::triangulatePointCloud);
  connect(decoderWorker, &DecoderWorker::logMessage, logDialog,
          &LogDialog::onlogMessage);
  connect(triangulatorWorker, &TriangulatorWorker::newPointCloud, this,
          &MainWindow::receiveNewPointCloud);
  connect(triangulatorWorker, &TriangulatorWorker::imshow, this,
          &MainWindow::imshow);
  connect(triangulatorWorker, &TriangulatorWorker::logMessage, logDialog,
          &LogDialog::onlogMessage);

  // Start threads
  decoderThread->start(QThread::LowPriority);
  triangulatorThread->start(QThread::LowPriority);
  scanWorkerThread->start(QThread::TimeCriticalPriority);

  // Setup and start processing
  QMetaObject::invokeMethod(decoderWorker, "setup");
  QMetaObject::invokeMethod(triangulatorWorker, "setup");
  QMetaObject::invokeMethod(scanWorker, "setup");
  QMetaObject::invokeMethod(scanWorker, "doWork");
  timer->start();

  // Change ui elements
  ui->actionStart->setEnabled(false);
  ui->actionStop->setEnabled(true);
  ui->actionTracking->setEnabled(true);
  ui->actionSavePointCloud->setEnabled(true);
  ui->actionSaveScreenshot->setEnabled(true);
  ui->actionCalibration->setEnabled(false);
}

void MainWindow::onActionStop() {
  // Stop processing on scan worker thread
  QMetaObject::invokeMethod(scanWorker, "stopWorking");
}

void MainWindow::onScanWorkerFinished() {

  // Terminate scan worker thread
  scanWorkerThread->quit();
  scanWorkerThread->wait();
  scanWorkerThread->deleteLater();

  decoderThread->quit();
  decoderThread->wait();

  triangulatorThread->quit();
  triangulatorThread->wait();

  // Change ui elements
  ui->actionStart->setEnabled(true);
  ui->actionStop->setEnabled(false);
  ui->actionTracking->setEnabled(false);
  ui->actionCalibration->setEnabled(true);
}

void MainWindow::onActionCalibration() {
  CalibrationDialog *calibrationDialog = new CalibrationDialog(this);
  connect(calibrationDialog, &CalibrationDialog::logMessage, logDialog,
          &LogDialog::onlogMessage);
  calibrationDialog->exec();
}

void MainWindow::onActionPreferences() {
  PreferenceDialog *preferenceDialog = new PreferenceDialog(this);
  preferenceDialog->exec();
}

void MainWindow::updateDisplayRate() {

  int mSecElapsed = timer->restart();
  displayIntervals.push_back(mSecElapsed);

  if (displayIntervals.size() > 10)
    displayIntervals.erase(displayIntervals.begin(),
                           displayIntervals.end() - 10);

  float meanMSecElapsed = 0;
  for (unsigned int i = 0; i < displayIntervals.size(); i++)
    meanMSecElapsed += displayIntervals[i];

  meanMSecElapsed /= displayIntervals.size();

  QString fpsString =
      QString("PCPS: %1").arg(1000.0 / meanMSecElapsed, 0, 'f', 2);
  ui->statusBar->showMessage(fpsString);
}

void MainWindow::receiveNewPointCloud(PointCloudConstPtr pointCloud) {
  // Display point cloud in widget
  if (ui->actionUpdatePointClouds->isChecked())
    ui->pointCloudWidget->updatePointCloud(pointCloud);
}

void MainWindow::closeEvent(QCloseEvent *event) {

  // Save main window geometry
  settings->setValue("geometry/mainwindow", saveGeometry());
  settings->setValue("state/mainwindow", saveState());

  // Store Geometry
  settings->setValue("geometry/log", logDialog->saveGeometry());
  settings->setValue("geometry/histogram", histogramDialog->saveGeometry());
  settings->setValue("geometry/shading", shadingDialog->saveGeometry());
  settings->setValue("geometry/decoderUp", decoderUpDialog->saveGeometry());
  settings->setValue("geometry/decoderVp", decoderVpDialog->saveGeometry());

  // Store Visibility
  settings->setValue("visible/log", logDialog->isVisible());
  settings->setValue("visible/histogram", histogramDialog->isVisible());
  settings->setValue("visible/shading", shadingDialog->isVisible());
  settings->setValue("visible/decoderUp", decoderUpDialog->isVisible());
  settings->setValue("visible/decoderVp", decoderVpDialog->isVisible());

  event->accept();
}

MainWindow::~MainWindow() {
  delete ui;
  delete settings;
}

void MainWindow::onActionLoadCalibration() {
  QString fileName = QFileDialog::getOpenFileName(
      this, "Choose calibration file", QString(), "*.xml");
  if (!(fileName.length() == 0)) {
    CalibrationData calibration;
    calibration.load(fileName);
    calibration.save("calibration.xml");
  }
}

void MainWindow::onActionExportCalibration() {
  CalibrationData calibration;
  calibration.load("calibration.xml");
  //  Non native file dialog
  //    QFileDialog saveFileDialog(this, "Export Calibration", QString(),
  //    "*.xml;;*.slcalib;;*.m"); saveFileDialog.setDefaultSuffix("xml");
  //    saveFileDialog.exec();
  //    QString fileName = saveFileDialog.selectedFiles().first();
  //  Native file dialog
  QString selectedFilter;
  QString fileName =
      QFileDialog::getSaveFileName(this, "Export Calibration", QString(),
                                   "*.xml;;*.slcalib;;*.m", &selectedFilter);

  if (!(fileName.length() == 0)) {
    QFileInfo info(fileName);
    QString type = info.suffix();
    if (type == "")
      fileName.append(selectedFilter.remove(0, 1));
    calibration.save(fileName);
  }
}

void MainWindow::onActionAbout() {
  AboutDialog *aboutDialog = new AboutDialog(this);
  aboutDialog->exec();
}

// Debuggings slots for plotting on the main thread
void MainWindow::hist(const char *windowName, cv::Mat im, unsigned int x,
                      unsigned int y) {
  cvtools::hist(windowName, im, x, y);
}
void MainWindow::imshow(const char *windowName, cv::Mat im, unsigned int x,
                        unsigned int y) {
  cvtools::imshow(windowName, im, x, y);
}

void MainWindow::on_actionUpload_Scan_Patterns_triggered() {
  logDialog->setVisible(true);

  int screenNum = settings->value("projector/screenNumber", -1).toInt();
  if (screenNum != -3) {
    std::cerr << "Can only upload patterns to LC4500 projector." << std::endl;
    return;
  }

  std::cout << "Uploading patterns to projector..." << std::endl;

  auto projector = std::make_unique<ProjectorLC4500>(0).release();

  // Initialize encoder
  unsigned int screenResX, screenResY;
  projector->getScreenRes(&screenResX, &screenResY);

  std::string codecName =
      settings->value("pattern/mode", "PhaseShift2x3").toString().toStdString();
  if (Codecs.count(codecName) == 0) {
    std::cerr << "ScanWorker: invalid codec " << codecName << std::endl;
    return;
  }

  CodecDir dir = static_cast<CodecDir>(
      settings->value("pattern/direction", CodecDirHorizontal).toInt());
  if (dir == CodecDirNone) {
    std::cerr << "ScanWorker: invalid coding direction " << std::endl;
  }

  auto encoder = EncoderFactory::NewEncoder(Codecs.at(codecName), screenResX,
                                            screenResY, dir)
                     .release();

  // Run pattern upload on background thread
  QFutureWatcher<void> *watcher = new QFutureWatcher<void>(this);
  connect(watcher, &QFutureWatcher<void>::finished, this,
          [projector, encoder]() {
            std::cout << "Upload done";
            delete projector;
            delete encoder;
          });
  QFuture<bool> future =
      QtConcurrent::run(&ScanWorker::setPatterns, encoder, projector);
  watcher->setFuture(future);
}

void MainWindow::on_actionUpload_Calibration_Patterns_triggered() {

  logDialog->setVisible(true);

  int screenNum = settings->value("projector/screenNumber", -1).toInt();
  if (screenNum != -3) {
    std::cerr << "Can only upload patterns to LC4500 projector." << std::endl;
    return;
  }

  std::cout << "Uploading patterns to projector..." << std::endl;

  auto projector = new ProjectorLC4500(0);

  // Initialize encoder
  unsigned int screenResX, screenResY;
  projector->getScreenRes(&screenResX, &screenResY);
  auto encoder =
      std::make_unique<EncoderCalibration>(screenResX, screenResY, CodecDirBoth)
          .release();

  // Run pattern upload on background thread
  QFutureWatcher<void> *watcher = new QFutureWatcher<void>(this);
  connect(watcher, &QFutureWatcher<void>::finished, this,
          [projector, encoder]() {
            std::cout << "Upload done";
            delete projector;
            delete encoder;
          });
  QFuture<bool> future =
      QtConcurrent::run(&ScanWorker::setPatterns, encoder, projector);
  watcher->setFuture(future);
}
