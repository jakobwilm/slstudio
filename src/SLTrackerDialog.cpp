#include "SLTrackerDialog.h"
#include "ui_SLTrackerDialog.h"

SLTrackerDialog::SLTrackerDialog(QWidget *parent) : QDialog(parent), ui(new Ui::SLTrackerDialog), trackerWorkerThread(NULL), trackerWorker(NULL), tracking(false){
    ui->setupUi(this);

    // Create QDockWidget like action associated with dialog
    action = new QAction("Tracker", this);
    action->setCheckable(true);
    connect(action, SIGNAL(toggled(bool)), this, SLOT(setShown(bool)));

    // Create traces
    ui->translationTraceWidget->setBounds(100, -180, 180);
    ui->translationTraceWidget->addTrace("tx");
    ui->translationTraceWidget->addTrace("ty");
    ui->translationTraceWidget->addTrace("tz");

    ui->rotationTraceWidget->setBounds(100, -0.5, 1.5);
    ui->rotationTraceWidget->addTrace("qw");
    ui->rotationTraceWidget->addTrace("qx");
    ui->rotationTraceWidget->addTrace("qy");
    ui->rotationTraceWidget->addTrace("qz");
}

// QDockWidget like, return a checkable action in sync with visibility
QAction* SLTrackerDialog::toggleViewAction(){
    return action;
}

void SLTrackerDialog::receiveNewPointCloud(PointCloudConstPtr pointCloud){
    emit newPointCloud(pointCloud);
}

SLTrackerDialog::~SLTrackerDialog(){

    delete ui;

}

void SLTrackerDialog::on_startStopPushButton_clicked(){

    if(!tracking){
        // Prepare trackerWorker on separate thread
        trackerWorker = new SLTrackerWorker();
        trackerWorkerThread = new QThread(this);
        trackerWorkerThread->setObjectName("trackerWorkerThread");

        trackerWorker->moveToThread(trackerWorkerThread);
        trackerWorkerThread->start(QThread::LowPriority);

        QMetaObject::invokeMethod(trackerWorker, "setup");

        qRegisterMetaType< Eigen::Affine3f >("Eigen::Affine3f");
        connect(this, SIGNAL(newPointCloud(PointCloudConstPtr)), trackerWorker, SLOT(trackPointCloud(PointCloudConstPtr)));
//        connect(trackerWorker, SIGNAL(newPoseEstimate(Eigen::Affine3f)), ui->poseWidget, SLOT(showPoseEstimate(Eigen::Affine3f)));
        connect(trackerWorker, SIGNAL(newPoseEstimate(Eigen::Affine3f)), this, SLOT(showPoseEstimate(Eigen::Affine3f)));
        connect(trackerWorkerThread, SIGNAL(finished()), trackerWorker, SLOT(deleteLater()));

        tracking = true;
        ui->startStopPushButton->setText("Stop Tracking");

    } else {
        // Terminate tracker worker thread
        if(trackerWorkerThread && trackerWorkerThread->isRunning()){
            trackerWorkerThread->quit();
            trackerWorkerThread->wait();
        }

        disconnect(trackerWorker);
        tracking = false;
        ui->startStopPushButton->setText("Start Tracking");
    }
}

void SLTrackerDialog::showEvent(QShowEvent *){
    if(!action->isChecked())
        action->setChecked(true);
}

void SLTrackerDialog::closeEvent(QCloseEvent *){
    action->setChecked(false);
}

void SLTrackerDialog::showPoseEstimate(const Eigen::Affine3f & T){

    if(ui->poseTab->isVisible()){
        ui->poseWidget->showPoseEstimate(T);
    } else if(ui->traceTab->isVisible()){
        Eigen::Vector3f t(T.translation());
        Eigen::Quaternionf q(T.rotation());

        ui->translationTraceWidget->addMeasurement("tx", t(0));
        ui->translationTraceWidget->addMeasurement("ty", t(1));
        ui->translationTraceWidget->addMeasurement("tz", t(2));
        ui->translationTraceWidget->draw();

        ui->rotationTraceWidget->addMeasurement("qw", q.w());
        ui->rotationTraceWidget->addMeasurement("qx", q.x());
        ui->rotationTraceWidget->addMeasurement("qy", q.y());
        ui->rotationTraceWidget->addMeasurement("qz", q.z());
        ui->rotationTraceWidget->draw();
    }
}
