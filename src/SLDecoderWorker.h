/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) 2013 -- 2014 Jakob Wilm, DTU, Kgs.Lyngby, Denmark
 *
*/

#ifndef SLDECODERWORKER_H
#define SLDECODERWORKER_H

#include <QObject>
#include <QTime>

#include "Codec.h"

class SLDecoderWorker : public QObject {
    Q_OBJECT

    public:
        SLDecoderWorker(): screenCols(0), screenRows(0){}
        ~SLDecoderWorker();
    public slots:
        void setup();
        void decodeSequence(std::vector<cv::Mat> frameSeq);
    signals:
        void imshow(const char* windowName, cv::Mat mat, unsigned int x, unsigned int y);
        void showShading(cv::Mat mat);
        void showDecoderUp(cv::Mat mat);
        void showDecoderVp(cv::Mat mat);
        void newUpVp(cv::Mat up, cv::Mat vp, cv::Mat mask, cv::Mat shading);
        void error(QString err);
        //void finished();
    private:
        Decoder *decoder;
        unsigned int screenCols, screenRows;
        QTime time;
        bool busy;
};

#endif
