#include "SLStudio.h"
#include "ui_SLStudio.h"

#include <stdio.h>
#include <string.h>
#include <QThread>
#include <QFileDialog>

#include "SLCalibrationDialog.h"
#include "SLPreferenceDialog.h"
#include "SLAboutDialog.h"

#include "SLVideoWidget.h"

#include <QtGui>

#include "cvtools.h"

using namespace std;

SLStudio::SLStudio(QWidget *parent) : QMainWindow(parent), ui(new Ui::SLStudio),
        scanWorkerThread(NULL), settings(NULL),
        histogramDialog(NULL), shadingDialog(NULL), decoderUpDialog(NULL), decoderVpDialog(NULL), trackerDialog(NULL){

    ui->setupUi(this);

    time = new QTime;

    // Restore main window geometry
    settings = new QSettings("SLStudio");
    restoreGeometry(settings->value("geometry/mainwindow").toByteArray());
    restoreState(settings->value("state/mainwindow").toByteArray());

    // Ui connections
    connect(ui->pointCloudWidget, SIGNAL(newPointCloudDisplayed()), this, SLOT(updateDisplayRate()));

    // Create video dialogs
    histogramDialog = new SLVideoDialog("Histogram", this);
    shadingDialog  = new SLVideoDialog("Shading", this);
    cameraFramesDialog  = new SLVideoDialog("Camera Frames", this);
    decoderUpDialog = new SLVideoDialog("Decoder Up", this);
    decoderVpDialog = new SLVideoDialog("Decoder Vp", this);

    // Add view menu actions
    ui->menuView->addAction(histogramDialog->toggleViewAction());
    ui->menuView->addAction(shadingDialog->toggleViewAction());
    ui->menuView->addAction(cameraFramesDialog->toggleViewAction());
    ui->menuView->addAction(decoderUpDialog->toggleViewAction());
    ui->menuView->addAction(decoderVpDialog->toggleViewAction());

    // Restore Geometry
    histogramDialog->restoreGeometry(settings->value("geometry/histogram").toByteArray());
    shadingDialog->restoreGeometry(settings->value("geometry/shading").toByteArray());
    cameraFramesDialog->restoreGeometry(settings->value("geometry/cameraFrames").toByteArray());
    decoderUpDialog->restoreGeometry(settings->value("geometry/decoderUp").toByteArray());
    decoderVpDialog->restoreGeometry(settings->value("geometry/decoderVp").toByteArray());

    // Restore Visibility
    histogramDialog->setVisible(settings->value("visible/histogram", false).toBool());
    shadingDialog->setVisible(settings->value("visible/shading", false).toBool());
    cameraFramesDialog->setVisible(settings->value("visible/cameraFrames", false).toBool());
    decoderUpDialog->setVisible(settings->value("visible/decoderUp", false).toBool());
    decoderVpDialog->setVisible(settings->value("visible/decoderVp", false).toBool());

    // Tracker Dialog
    trackerDialog = new SLTrackerDialog(this);
    ui->menuView->addAction(trackerDialog->toggleViewAction());
    trackerDialog->setVisible(settings->value("visible/trackerDialog", false).toBool());

}

void SLStudio::onShowHistogram(cv::Mat im){
    if(histogramDialog->isVisible())
        histogramDialog->showImageCV(im);
}

void SLStudio::onShowShading(cv::Mat im){
    if(shadingDialog->isVisible())
        shadingDialog->showImageCV(im);
}

void SLStudio::onShowCameraFrames(std::vector<cv::Mat> frameSeq){
    if(cameraFramesDialog->isVisible())
        cameraFramesDialog->showImageSeqCV(frameSeq);
}

void SLStudio::onShowDecoderUp(cv::Mat im){
    if(decoderUpDialog->isVisible())
        decoderUpDialog->showImageCV(im);
}

void SLStudio::onShowDecoderVp(cv::Mat im){
    if(decoderVpDialog->isVisible()){
        decoderVpDialog->showImageCV(im);
        //std::cout << "Showing now!" << std::endl;
    }
}

void SLStudio::onActionStart(){

    // Prepare scanWorker on separate thread
    scanWorker = new SLScanWorker(this);
    scanWorkerThread = new QThread(this);
    scanWorkerThread->setObjectName("scanWorkerThread");
    scanWorker->moveToThread(scanWorkerThread);
    connect(scanWorker, SIGNAL(finished()), this, SLOT(onScanWorkerFinished()));

    // Prepare decoderWorker on separate thread
    decoderWorker = new SLDecoderWorker();
    decoderThread = new QThread(this);
    decoderThread->setObjectName("decoderThread");
    decoderWorker->moveToThread(decoderThread);
    connect(decoderThread, SIGNAL(started()), decoderWorker, SLOT(setup()));
    connect(decoderThread, SIGNAL(finished()), decoderWorker, SLOT(deleteLater()));
    connect(decoderThread, SIGNAL(finished()), decoderThread, SLOT(deleteLater()));

    // Prepare triangulatorWorker on separate thread
    triangulatorWorker = new SLTriangulatorWorker();
    triangulatorThread = new QThread(this);
    triangulatorThread->setObjectName("triangulatorThread");
    triangulatorWorker->moveToThread(triangulatorThread);
    connect(triangulatorThread, SIGNAL(started()), triangulatorWorker, SLOT(setup()));
    connect(triangulatorThread, SIGNAL(finished()), triangulatorWorker, SLOT(deleteLater()));
    connect(triangulatorThread, SIGNAL(finished()), triangulatorThread, SLOT(deleteLater()));

    // Register metatypes
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType< std::vector<cv::Mat> >("std::vector<cv::Mat>");
    qRegisterMetaType< PointCloudConstPtr >("PointCloudConstPtr");

    // Inter thread connections
    connect(scanWorker, SIGNAL(showHistogram(cv::Mat)), this, SLOT(onShowHistogram(cv::Mat)));
    connect(scanWorker, SIGNAL(newFrameSeq(std::vector<cv::Mat>)), decoderWorker, SLOT(decodeSequence(std::vector<cv::Mat>)));
    connect(scanWorker, SIGNAL(newFrameSeq(std::vector<cv::Mat>)), this, SLOT(onShowCameraFrames(std::vector<cv::Mat>)));
    connect(decoderWorker, SIGNAL(showShading(cv::Mat)), this, SLOT(onShowShading(cv::Mat)));
    connect(decoderWorker, SIGNAL(showDecoderUp(cv::Mat)), this, SLOT(onShowDecoderUp(cv::Mat)));
    connect(decoderWorker, SIGNAL(showDecoderVp(cv::Mat)), this, SLOT(onShowDecoderVp(cv::Mat)));
    connect(decoderWorker, SIGNAL(newUpVp(cv::Mat,cv::Mat,cv::Mat,cv::Mat)), triangulatorWorker, SLOT(triangulatePointCloud(cv::Mat,cv::Mat,cv::Mat,cv::Mat)));
    connect(triangulatorWorker, SIGNAL(newPointCloud(PointCloudConstPtr)), this, SLOT(receiveNewPointCloud(PointCloudConstPtr)));
    connect(triangulatorWorker, SIGNAL(imshow(const char*,cv::Mat,uint,uint)), this, SLOT(imshow(const char*,cv::Mat,uint,uint)));

    // Start threads
    decoderThread->start(QThread::LowPriority);
    triangulatorThread->start(QThread::LowPriority);
    scanWorkerThread->start(QThread::TimeCriticalPriority);

    // Setup and start processing
    QMetaObject::invokeMethod(decoderWorker, "setup");
    QMetaObject::invokeMethod(triangulatorWorker, "setup");
    QMetaObject::invokeMethod(scanWorker, "setup");
    QMetaObject::invokeMethod(scanWorker, "doWork");
    time->start();

    // Change ui elements
    ui->actionStart->setEnabled(false);
    ui->actionStop->setEnabled(true);
    ui->actionTracking->setEnabled(true);
    ui->actionSavePointCloud->setEnabled(true);
    ui->actionSaveScreenshot->setEnabled(true);
    ui->actionCalibration->setEnabled(false);

}

void SLStudio::onActionStop(){
    // Stop processing on scan worker thread
    QMetaObject::invokeMethod(scanWorker, "stopWorking");

    //cv::destroyAllWindows();

    decoderThread->quit();
    decoderThread->wait();

    std::cout<<"decoderThread deleted\n"<<std::flush;

    triangulatorThread->quit();
    triangulatorThread->wait();

    std::cout<<"triangulatorThread deleted\n"<<std::flush;
}

void SLStudio::onScanWorkerFinished(){
    QMetaObject::invokeMethod(scanWorker, "deleteLater");

    // Terminate scan worker thread
    scanWorkerThread->quit();
    scanWorkerThread->wait();
    //scanWorkerThread->deleteLater();
    //delete scanWorkerThread;

    // Change ui elements
    ui->actionStart->setEnabled(true);
    ui->actionStop->setEnabled(false);
    ui->actionTracking->setEnabled(false);
    ui->actionCalibration->setEnabled(true);
}

void SLStudio::onActionCalibration(){
    SLCalibrationDialog *calibrationDialog = new SLCalibrationDialog(this);
    calibrationDialog->exec();
}


void SLStudio::onActionPreferences(){
    SLPreferenceDialog *preferenceDialog = new SLPreferenceDialog(this);
    preferenceDialog->exec();
}

void SLStudio::updateDisplayRate(){

    int mSecElapsed = time->restart();
    displayIntervals.push_back(mSecElapsed);

    if(displayIntervals.size() > 10)
        displayIntervals.erase(displayIntervals.begin(), displayIntervals.end()-10);

    float meanMSecElapsed = 0;
    for(unsigned int i=0; i<displayIntervals.size(); i++)
        meanMSecElapsed += displayIntervals[i];

    meanMSecElapsed /= displayIntervals.size();

    QString fpsString = QString("PCPS: %1").arg(1000.0/meanMSecElapsed, 0, 'f', 2);
    ui->statusBar->showMessage(fpsString);

}

void SLStudio::receiveNewPointCloud(PointCloudConstPtr pointCloud){
    // Display point cloud in widget
    if(ui->actionUpdatePointClouds->isChecked())
        ui->pointCloudWidget->updatePointCloud(pointCloud);

    if(trackerDialog->isVisible())
        trackerDialog->receiveNewPointCloud(pointCloud);
}

void SLStudio::closeEvent(QCloseEvent *event){

    // Save main window geometry
    settings->setValue("geometry/mainwindow", saveGeometry());
    settings->setValue("state/mainwindow", saveState());

    // Store Geometry
    settings->setValue("geometry/histogram", histogramDialog->saveGeometry());
    settings->setValue("geometry/shading", shadingDialog->saveGeometry());
    settings->setValue("geometry/decoderUp", decoderUpDialog->saveGeometry());
    settings->setValue("geometry/decoderVp", decoderVpDialog->saveGeometry());

    // Store Visibility
    settings->setValue("visible/histogram", histogramDialog->isVisible());
    settings->setValue("visible/shading", shadingDialog->isVisible());
    settings->setValue("visible/decoderUp", decoderUpDialog->isVisible());
    settings->setValue("visible/decoderVp", decoderVpDialog->isVisible());

    // Store data for trackerDialog (temp)
    settings->setValue("geometry/trackerDialog", trackerDialog->saveGeometry());
    settings->setValue("visible/trackerDialog", trackerDialog->isVisible());

    event->accept();
}

SLStudio::~SLStudio(){
    delete ui;
    delete settings;
}

void SLStudio::onActionLoadCalibration(){
    QString fileName = QFileDialog::getOpenFileName(this, "Choose calibration file", QString(), "*.xml");
    if(!(fileName.length() == 0)){
        CalibrationData calibration;
        calibration.load(fileName.toStdString());
        calibration.save("calibration.xml");
    }
}

void SLStudio::onActionExportCalibration(){
    CalibrationData calibration;
    calibration.load("calibration.xml");
//  Non native file dialog
//    QFileDialog saveFileDialog(this, "Export Calibration", QString(), "*.xml;;*.slcalib;;*.m");
//    saveFileDialog.setDefaultSuffix("xml");
//    saveFileDialog.exec();
//    QString fileName = saveFileDialog.selectedFiles().first();
//  Native file dialog
    QString selectedFilter;
    QString fileName = QFileDialog::getSaveFileName(this, "Export Calibration", QString(), "*.xml;;*.slcalib;;*.m", &selectedFilter);

    if(!(fileName.length() == 0)){
        QFileInfo info(fileName);
        QString type = info.suffix();
        if(type == "")
            fileName.append(selectedFilter.remove(0,1));
        calibration.save(fileName.toStdString());
    }
}

void SLStudio::onActionAbout(){
    SLAboutDialog *aboutDialog = new SLAboutDialog(this);
    aboutDialog->exec();
}

// Debuggings slots for plotting on the main thread
void SLStudio::hist(const char* windowName, cv::Mat im, unsigned int x, unsigned int y){
    cvtools::hist(windowName, im, x, y);
}
void SLStudio::imshow(const char* windowName, cv::Mat im, unsigned int x, unsigned int y){
    cvtools::imshow(windowName, im, x, y);
}


