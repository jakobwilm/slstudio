#include "SLVideoDialog.h"
#include "ui_SLVideoDialog.h"

#include <QSettings>

SLVideoDialog::SLVideoDialog(const QString &title, QWidget *parent) : QDialog(parent),ui(new Ui::SLVideoDialog){
    ui->setupUi(this);

    //this->setWindowFlags(Qt::Window);

    // Set window title
    this->setWindowTitle(title);

    // Create QDockWidget like action associated with dialog
    action = new QAction(title, this);
    action->setCheckable(true);
    connect(action, SIGNAL(toggled(bool)), this, SLOT(setShown(bool)));
}

// QDockWidget like, return a checkable action in sync with visibility
QAction* SLVideoDialog::toggleViewAction(){
    return action;
}

static QImage cvMat2qImage(cv::Mat mat){

    // 8-bits unsigned, NO. OF CHANNELS=1
    if(mat.type()==CV_8UC1) {
        // Set the color table (used to translate colour indexes to qRgb values)
        QVector<QRgb> colorTable;
        for (int i=0; i<256; i++)
            colorTable.push_back(qRgb(i,i,i));
        // Copy input Mat
        QImage img((const uchar*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        return img;
    } else if(mat.type()==CV_8UC3) {
        // Copy input Mat
        QImage img((const uchar*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return img.rgbSwapped();
    } else if(mat.type()==CV_16UC1) {
        mat.convertTo(mat, CV_8UC1, 1.0/256.0);
        return cvMat2qImage(mat);
    } else if(mat.type()==CV_32FC1) {
        cv::Mat rgb(mat.size(), CV_32FC3);
        rgb.addref();
        cv::cvtColor(mat, rgb, cv::COLOR_GRAY2RGB);
        // Copy input Mat
        QImage img((const uchar*)rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB32);
        rgb.release();
        return img;
    } else {
        std::cerr << "SLVideoDialog: cv::Mat could not be converted to QImage!";
        return QImage();
    }
}

void SLVideoDialog::showImageCV(cv::Mat image){
    if(!action->isChecked())
        return;

    QImage qimage = cvMat2qImage(image);

    int w = ui->label->width();
    int h = ui->label->height();
    QPixmap pixmap = QPixmap::fromImage(qimage);

    ui->label->setPixmap(pixmap.scaled(w,h,Qt::KeepAspectRatio));

}

void SLVideoDialog::showEvent(QShowEvent *){
    if(!action->isChecked())
        action->setChecked(true);
}

void SLVideoDialog::closeEvent(QCloseEvent *){
    action->setChecked(false);
}

SLVideoDialog::~SLVideoDialog(){
    delete ui;

}
