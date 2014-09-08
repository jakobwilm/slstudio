#include "CameraTest.h"
#include "ui_CameraTest.h"
#include "CameraWorker.h"
#include <stdio.h>
#include <QThread>

CameraTest::CameraTest(QWidget *parent) : QDialog(parent), ui(new Ui::CameraTest){

    ui->setupUi(this);

    std::vector< std::vector<CameraInfo> > cameras;
    cameras = Camera::GetInterfaceCameraList();

    cameraThread = new QThread;
    cameraWorker = new CameraWorker;

    cameraWorker->moveToThread(cameraThread);
    cameraThread->start();

    qRegisterMetaType<cv::Mat>("cv::Mat");
    connect(cameraWorker, SIGNAL(newFrame(cv::Mat)), ui->widget, SLOT(showFrameCV(cv::Mat)));

    // Setup camera worker for interface X camera Y
    QMetaObject::invokeMethod(cameraWorker, "setup", Q_ARG(unsigned, 1), Q_ARG(unsigned, 0));
    // Start capturing
    QMetaObject::invokeMethod(cameraWorker, "doWork");

}

void CameraTest::closeEvent(QCloseEvent *){

    disconnect(cameraWorker, SIGNAL(newFrame(cv::Mat)), ui->widget, SLOT(showFrameCV(cv::Mat)));
    QApplication::processEvents();

    QMetaObject::invokeMethod(cameraWorker, "stopWorking");
    QMetaObject::invokeMethod(cameraWorker, "deleteLater");

    cameraThread->quit();
    cameraThread->wait();
    cameraThread->deleteLater();

}

CameraTest::~CameraTest(){
    delete ui;
}
