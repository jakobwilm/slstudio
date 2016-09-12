/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) 2013 -- 2014 Jakob Wilm, DTU, Kgs.Lyngby, Denmark
 *
*/

#ifndef SLCALIBRATAIONDIALOG_H
#define SLCALIBRATAIONDIALOG_H

#include <QDialog>
#include <QModelIndex>
#include <QListWidgetItem>

#include "Camera.h"
#include "Projector.h"
#include "Calibrator.h"
#include "SLStudio.h"

namespace Ui {
    class SLCalibrationDialog;
}

class SLCalibrationDialog : public QDialog, public CalibratorObserver {
    Q_OBJECT
    
    public:
        explicit SLCalibrationDialog(SLStudio *parent = 0);
        ~SLCalibrationDialog();
        void timerEvent(QTimerEvent *event);
        void closeEvent(QCloseEvent *);
    private slots:
        void on_snapButton_clicked();
        void on_calibrateButton_clicked();
        void on_listWidget_itemSelectionChanged();
        void on_saveButton_clicked();
        void newSequenceResult(cv::Mat img, unsigned int idx, bool success);
        virtual void reject() {close();}

    signals:
        void newCalibrationSaved(CalibrationData _calib);
        
    private:
        Ui::SLCalibrationDialog *ui;
        Camera *camera;
        Projector *projector;
        Calibrator *calibrator;
        CalibrationData calib;
        int liveViewTimer;
        vector< vector<cv::Mat> > frameSeqs;
        vector<unsigned int> activeFrameSeqs;
        bool reviewMode;
        unsigned int timerInterval; //ms
        unsigned int delay; //ms
        bool diamondPattern;
        unsigned int screenCols;
        unsigned int screenRows;
};

#endif // SLCALIBRATAIONDIALOG_H
