#ifndef SLPROJECTORWORKER_H
#define SLPROJECTORWORKER_H

#include <QObject>
#include <opencv2/opencv.hpp>
#include "Projector.h"
#include "Codec.h"

class SLProjectorWorker : public QObject{
    Q_OBJECT

    public:
        SLProjectorWorker(): _isWorking(false), projector(NULL), screenResX(0), screenResY(0) {}
        bool isWorking(){return _isWorking;}
        ~SLProjectorWorker();
    public slots:
        void setup();
        void doWork();
        void stopWorking(){_isWorking = false;}
    signals:
        void finished();
    private:
        bool _isWorking;
        Projector *projector;
        Codec *codec;
        unsigned int screenResX, screenResY;
};

#endif // SLProjectorWorker_H
