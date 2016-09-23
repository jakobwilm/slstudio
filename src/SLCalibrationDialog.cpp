#include "SLCalibrationDialog.h"
#include "ui_SLCalibrationDialog.h"

#include <QtTest/QTest>
#include <QSettings>

#include <opencv2/opencv.hpp>

#include "Camera.h"
#include "SLProjectorVirtual.h"
#include "Projector.h"

#include "CalibratorLocHom.h"
#include "CalibratorRBF.h"

#include "cvtools.h"


SLCalibrationDialog::SLCalibrationDialog(SLStudio *parent) : QDialog(parent), ui(new Ui::SLCalibrationDialog), reviewMode(false), timerInterval(100), delay(100) {
    ui->setupUi(this);
    setSizeGripEnabled(false);

    // Release this dialog on close
    this->setAttribute(Qt::WA_DeleteOnClose);

    QSettings settings("SLStudio");

    //Checkerboard parameters
    unsigned int checkerSize = settings.value("calibration/checkerSize",8).toInt();
    ui->checkerSizeBox->setValue(checkerSize);
    unsigned int checkerRows = settings.value("calibration/checkerRows",8).toInt();
    ui->checkerRowsBox->setValue(checkerRows);
    unsigned int checkerCols = settings.value("calibration/checkerCols",8).toInt();
    ui->checkerColsBox->setValue(checkerCols);

    // Instatiate camera with software trigger
    int iNum = settings.value("camera/interfaceNumber", 0).toInt();
    int cNum = settings.value("camera/cameraNumber", 0).toInt();
    camera = Camera::NewCamera(iNum,cNum,triggerModeSoftware);

    delay = settings.value("trigger/delay", "100").toInt();

    // Set camera settings
    CameraSettings camSettings;
    camSettings.shutter = settings.value("camera/shutter", 16.666).toFloat();
    camSettings.gain = 0.0;
    camera->setCameraSettings(camSettings);
    camera->startCapture();

    // Initialize projector
    int screenNum = settings.value("projector/screenNumber", -1).toInt();
    if(screenNum == -1)
        projector = new SLProjectorVirtual(screenNum);
    else if(screenNum >= 0)
        projector = Projector::NewProjector(projectorTypeOpenGL, screenNum);
    else if(screenNum == -2)
        projector = Projector::NewProjector(projectorTypeLC3000);
    else if(screenNum == -3)
        projector = Projector::NewProjector(projectorTypeLC4500);
    else if(screenNum == -4)
        projector = Projector::NewProjector(projectorTypeQtGL);
    else
        std::cerr << "SLCalibrationDialog: invalid projector id " << screenNum << std::endl;

    unsigned int screenResX, screenResY;
    projector->getScreenRes(&screenResX, &screenResY);

    diamondPattern = settings.value("projector/diamondPattern", false).toBool();

    // Unique number of rows and columns
    if(diamondPattern){
        screenCols = 2*screenResX;
        screenRows = screenResY;
    } else {
        screenCols = screenResX;
        screenRows = screenResY;
    }

    // Create calibrator
    calibrator = new CalibratorLocHom(screenCols, screenRows);

    calibrator->setObserver(this);

    // Upload patterns to projector/GPU
    for(unsigned int i=0; i<calibrator->getNPatterns(); i++){
        cv::Mat pattern = calibrator->getCalibrationPattern(i);

        // general repmat
        pattern = cv::repeat(pattern, screenRows/pattern.rows + 1, screenCols/pattern.cols + 1);
        pattern = pattern(cv::Range(0, screenRows), cv::Range(0, screenCols));

        if(diamondPattern)
            pattern = cvtools::diamondDownsample(pattern);

        if (!pattern.isContinuous()) {
            pattern = pattern.clone();
        }

        projector->setPattern(i, pattern.ptr(), pattern.cols, pattern.rows);
    }

    // Start live view
    timerInterval = delay + camSettings.shutter;
    liveViewTimer = startTimer(timerInterval);
}

void SLCalibrationDialog::timerEvent(QTimerEvent *event){

    if(event->timerId() != liveViewTimer){
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

SLCalibrationDialog::~SLCalibrationDialog(){
    delete ui;
}

void SLCalibrationDialog::on_snapButton_clicked(){

    // If in review mode
    if(reviewMode){
        reviewMode = false;
        ui->listWidget->clearSelection();
        liveViewTimer = startTimer(timerInterval);
        ui->snapButton->setText("Snap");
        return;
    }

    ui->snapButton->setEnabled(false);

    // Stop live view
    killTimer(liveViewTimer);

    vector<cv::Mat> frameSeq;

    for(unsigned int i=0; i<calibrator->getNPatterns(); i++){

        // Project pattern
        projector->displayPattern(i);
        QTest::qSleep(delay);

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
    QListWidgetItem* item = new QListWidgetItem(QString("Sequence %1").arg(frameSeqs.size()), ui->listWidget);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable); // set checkable flag
    item->setCheckState(Qt::Checked); // AND initialize check state

    //    // Allow calibration if enough frame pairs
    //    if(ui->listWidget->count() >= 3)
    ui->calibrateButton->setEnabled(true);

    // Display white
    projector->displayWhite();

    #if 1
        // Write frame seq to disk
        for(int i=0; i<frameSeq.size(); i++){
            QString filename = QString("frameSeq_%1.png").arg(i, 3, 10, QChar('0'));
            cv::imwrite(filename.toStdString(), frameSeq[i]);
        }
    #endif

    // Restart live view
    liveViewTimer = startTimer(timerInterval);

    ui->snapButton->setEnabled(true);

}

void SLCalibrationDialog::on_calibrateButton_clicked(){

    // Disable interface elements
    ui->calibrateButton->setEnabled(false);
    ui->listWidget->setEnabled(false);

    // Stop live view
    killTimer(liveViewTimer);
    reviewMode = true;
    ui->snapButton->setText("Live View");

    calibrator->reset();

    // Note which frame sequences are used
    activeFrameSeqs.clear();

    for(int i=0; i<ui->listWidget->count(); i++){
        if(ui->listWidget->item(i)->checkState() == Qt::Checked){
            vector<cv::Mat> frameSeq(frameSeqs[i].begin(), frameSeqs[i].begin() + calibrator->getNPatterns());
            calibrator->addFrameSequence(frameSeq);
            activeFrameSeqs.push_back(i);
        }
    }

    // Perform calibration
    unsigned int checkerSize = ui->checkerSizeBox->value();
    unsigned int checkerRows = ui->checkerRowsBox->value();
    unsigned int checkerCols = ui->checkerColsBox->value();
    calib = calibrator->calibrate(checkerSize, checkerRows, checkerCols);

    // Re-enable interface elements
    ui->calibrateButton->setEnabled(true);
    ui->listWidget->setEnabled(true);
    ui->saveButton->setEnabled(true);
}

void SLCalibrationDialog::on_listWidget_itemSelectionChanged(){

    // If selection was cleared
    if(ui->listWidget->selectedItems().isEmpty())
        return;

    // Stop live view
    killTimer(liveViewTimer);
    reviewMode = true;
    ui->snapButton->setText("Live View");

    int currentRow = ui->listWidget->currentRow();
    ui->videoWidget->showFrameCV(frameSeqs[currentRow].back());
}


void SLCalibrationDialog::on_saveButton_clicked(){

    calib.frameWidth = camera->getFrameWidth();
    calib.frameHeight = camera->getFrameHeight();
    unsigned int screenResX, screenResY;
    projector->getScreenRes(&screenResX, &screenResY);
    calib.screenResX = screenResX;
    calib.screenResY = screenResY;
    calib.calibrationDateTime = QDateTime::currentDateTime().toString("DD.MM.YYYY HH:MM:SS").toStdString();

    calib.save("calibration.xml");
    this->close();
}


void SLCalibrationDialog::newSequenceResult(cv::Mat img, unsigned int idx, bool success){

    // Skip non-active frame sequences
    int idxListView = activeFrameSeqs[idx];

    // Append calibration result to frame sequence
    unsigned int N = calibrator->getNPatterns();
    if(frameSeqs[idxListView].size() == N)
        frameSeqs[idxListView].push_back(img);
    else
        frameSeqs[idxListView][N] = img;

    if(!success) // uncheck
        ui->listWidget->item(idxListView)->setCheckState(Qt::Unchecked);

    // Highlight
    ui->listWidget->setCurrentRow(idxListView);
    ui->listWidget->setFocus();

    QApplication::processEvents();
}

void SLCalibrationDialog::closeEvent(QCloseEvent *){

    delete camera;
    delete projector;
    delete calibrator;
    this->deleteLater();

    // Save calibration settings
    QSettings settings("SLStudio");
    unsigned int checkerSize = ui->checkerSizeBox->value();
    settings.setValue("calibration/checkerSize", checkerSize);
    unsigned int checkerRows = ui->checkerRowsBox->value();
    settings.setValue("calibration/checkerRows", checkerRows);
    unsigned int checkerCols = ui->checkerColsBox->value();
    settings.setValue("calibration/checkerCols", checkerCols);

}
