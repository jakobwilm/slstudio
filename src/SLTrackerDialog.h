/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) 2013 -- 2014 Jakob Wilm, DTU, Kgs.Lyngby, Denmark
 *
*/

#ifndef SLTRACKERDIALOG_H
#define SLTRACKERDIALOG_H

#include <QDialog>
#include <QThread>

#include "SLTrackerWorker.h"

namespace Ui {
class SLTrackerDialog;
}

class SLTrackerDialog : public QDialog {
    Q_OBJECT

    public:
        explicit SLTrackerDialog(QWidget *parent = 0);
        QAction* toggleViewAction();
        void showEvent(QShowEvent *);
        void closeEvent(QCloseEvent *);
        ~SLTrackerDialog();
    public slots:
        void receiveNewPointCloud(PointCloudConstPtr pointCloud);
        void showPoseEstimate(const Eigen::Affine3f & T);
    private slots:
        void on_startStopPushButton_clicked();
    signals:
        void newPointCloud(PointCloudConstPtr pointCloud);
    private:
        Ui::SLTrackerDialog *ui;
        QThread *trackerWorkerThread;
        SLTrackerWorker *trackerWorker;
        bool tracking;
        QAction *action;

};

#endif // SLTRACKERDIALOG_H
