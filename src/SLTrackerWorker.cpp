#include "SLTrackerWorker.h"
#include "CalibrationData.h"

#include <QSettings>
#include <QEvent>
#include <QCoreApplication>

#include "TrackerICP.h"
#include "TrackerNDT.h"
#include "TrackerPCL.h"
#include <Eigen/Eigen>

void SLTrackerWorker::setup(){

    // Initialize the tracker object
    tracker = new TrackerICP();
    CalibrationData calibration = CalibrationData();
    calibration.load("calibration.xml");
    Eigen::Matrix3f Kc;
    Kc << calibration.Kc(0,0), calibration.Kc(0,1), calibration.Kc(0,2),
          calibration.Kc(1,0), calibration.Kc(1,1), calibration.Kc(1,2),
          calibration.Kc(2,0), calibration.Kc(2,1), calibration.Kc(2,2);
    tracker->setCameraMatrix(Kc);

    QSettings settings("SLStudio");
    writeToDisk = settings.value("writeToDisk/tracking",false).toBool();
    if(writeToDisk){

        // Tracking file
        QString fileName = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
        fileName.append(".track");
        ofStream = new std::ofstream;
        ofStream->open(fileName.toLocal8Bit().data(), std::ofstream::out);
        (*ofStream) << "#V1.1 SLStudio Tracking File" << std::endl << "t_ms,Tx,Ty,Tz,Q0,Qx,Qy,Qz" << std::endl;

        trackingTime.start();
    }
}

void SLTrackerWorker::setReference(PointCloudConstPtr referencePointCloud){
    tracker->setReference(referencePointCloud);
    referenceSet = true;
    return;
}

void SLTrackerWorker::trackPointCloud(PointCloudConstPtr pointCloud){

    // Recursively call self until latest event is hit
    busy = true;
    QCoreApplication::sendPostedEvents(this, QEvent::MetaCall);
    bool result = busy;
    busy = false;
    if(!result){
        std::cerr << "SLTrackerWorker: dropped point cloud!" << std::endl;
        return;
    }

    if(!referenceSet){
        tracker->setReference(pointCloud);
        referenceSet = true;
        return;
    }

    performanceTime.start();

    Eigen::Affine3f T;
    bool converged;

    tracker->determineTransformation(pointCloud, T, converged);

    // Emit result
    if(converged)
        emit newPoseEstimate(T);

//    std::cout << "Pose: " << T.matrix() << std::endl;

    std::cout << "Tracker: " << performanceTime.elapsed() << "ms" << std::endl;

    if(writeToDisk){
        Eigen::Vector3f t(T.translation());
        Eigen::Quaternionf q(T.rotation());

        (*ofStream) << trackingTime.elapsed() << ",";
        (*ofStream) << t.x() << "," << t.y() << "," << t.z() << ",";
        (*ofStream) << q.w() << "," << q.x() << "," << q.y() << "," << q.z() ;
        (*ofStream) << std::endl;
    }


}

SLTrackerWorker::~SLTrackerWorker(){
    delete tracker;
    if(writeToDisk){
        ofStream->flush();
        ofStream->close();
        delete ofStream;
    }
}
