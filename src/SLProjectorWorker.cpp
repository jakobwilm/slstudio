#include "SLProjectorWorker.h"

#include "Camera.h"
#include <QApplication>
#include <QSettings>

#include "CodecUpPhaseShift.h"

void SLProjectorWorker::setup(){

    // Initialize projector
    QSettings settings("DTU", "SLStudio");
    unsigned int screenNum = settings.value("projector/screenNumber", 0).toInt();
    projector = new Projector(screenNum);
    projector->getScreenRes(&screenResX, &screenResY);

    std::cout << "SLProjectorWorker: opened screen " << screenResX << " x " << screenResY << std::endl << std::flush;

    // Instatiate codec
    bool bDiamondPattern = true;
    codec = new CodecUpPhaseShift(screenResX, screenResY, bDiamondPattern);
}

void SLProjectorWorker::doWork(){

    _isWorking = true;

    while(_isWorking){

        for(unsigned int i=0; i<codec->getNPatterns(); i++){
            // Get coded pattern
            cv::Mat pattern = codec->getEncodingPattern(i);

            // Project coded pattern
            projector->displayTexture(pattern.ptr(), pattern.cols, pattern.rows);
        }

        QApplication::processEvents();
    }

    // Emit finished signal
    emit finished();
}

SLProjectorWorker::~SLProjectorWorker(){
    delete projector;
    delete codec;
}
