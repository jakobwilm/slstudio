/*
 * CalibratorLocHome -- Calibrate using "local homographies" as proposed by Moreno, Taubin.
*/

#ifndef CALIBRATORLOCHOM_H
#define CALIBRATORLOCHOM_H

#include "Calibrator.h"

#include "Codec.h"

using namespace std;

class SLALGORITHM_EXPORT CalibratorLocHom : public Calibrator {
    public:
        CalibratorLocHom(unsigned int _screenCols, unsigned int _screenRows);
        CalibrationData calibrate(const int checkerSize, const int checkerRows, const int checkerCols);
        ~CalibratorLocHom(){delete encoder; delete decoder;}
    private:
        Encoder *encoder;
        Decoder *decoder;
};

#endif // CALIBRATORLOCHOM_H
