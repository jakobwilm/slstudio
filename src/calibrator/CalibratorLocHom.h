/*
 * CalibratorLocHome -- Calibrate using "local homographies" as proposed by Moreno, Taubin.
*/

#ifndef CALIBRATORLOCHOM_H
#define CALIBRATORLOCHOM_H

#include "Calibrator.h"

#include "Codec.h"

using namespace std;

class CalibratorLocHom : public Calibrator {
    Q_OBJECT
    public:
        CalibratorLocHom(unsigned int _screenCols, unsigned int _screenRows);
        CalibrationData calibrate();
        ~CalibratorLocHom(){delete encoder; delete decoder;}
    private:
        Encoder *encoder;
        Decoder *decoder;
};

#endif // CALIBRATORLOCHOM_H
