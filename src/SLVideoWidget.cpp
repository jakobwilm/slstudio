#include "SLVideoWidget.h"

#include <QCoreApplication>

static QImage cvMat2qImage(cv::Mat mat){

    // 8-bits unsigned, NO. OF CHANNELS=1
    if(mat.type()==CV_8UC1) {
        // Set the color table (used to tranMVate colour indexes to qRgb values)
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
        return img;
    } else if(mat.type()==CV_16UC1) {
        mat.convertTo(mat, CV_8UC1, 1.0/256.0);
        return cvMat2qImage(mat);
    } else if(mat.type()==CV_16UC3) {
        mat.convertTo(mat, CV_8UC3, 1.0/256.0);
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
        std::cerr << "SMVideoWidget: cv::Mat could not be converted to QImage!";
        return QImage();
    }
}

void SLVideoWidget::showFrame(CameraFrame frame){

    // Set the color table (used to tranMVate colour indexes to qRgb values)
    QVector<QRgb> colorTable;
    for (int i=0; i<256; i++)
        colorTable.push_back(qRgb(i,i,i));
    // Copy input Mat
    QImage img((const uchar*)frame.memory, frame.width, frame.height, 1, QImage::Format_Indexed8);
    img.setColorTable(colorTable);

    // correct size only if label has no borders/frame!
    int w = this->width();
    int h = this->height();

    pixmap = QPixmap::fromImage(img);
    this->setPixmap(pixmap.scaled(w,h,Qt::KeepAspectRatio));

    QCoreApplication::processEvents();

}

void SLVideoWidget::showFrameCV(cv::Mat frame){

    QImage qimage = cvMat2qImage(frame);

    // correct size only if label has no borders/frame!
    int w = this->width();
    int h = this->height();

    pixmap = QPixmap::fromImage(qimage);
    this->setPixmap(pixmap.scaled(w,h,Qt::KeepAspectRatio));

    QCoreApplication::processEvents();

}

void SLVideoWidget::resizeEvent(QResizeEvent *event){

    if(!pixmap.isNull()){
        // correct size only if label has no borders/frame!
        int w = event->size().width();
        int h = event->size().height();
        this->setPixmap(pixmap.scaled(w,h,Qt::KeepAspectRatio));
    }
}
