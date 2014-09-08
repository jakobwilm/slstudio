/*
 * CalibratorRBF -- Calibrate using radial basis functions, as proposed by us.
*/

#ifndef CalibratorRBF_H
#define CalibratorRBF_H

#include "Calibrator.h"

#include "Codec.h"

using namespace std;

class CalibratorRBF : public Calibrator {
    Q_OBJECT
    public:
        CalibratorRBF(unsigned int _screenCols, unsigned int _screenRows);
        CalibrationData calibrate();
        ~CalibratorRBF(){delete encoder; delete decoder;}
    private:
        Encoder *encoder;
        Decoder *decoder;
};

#endif // CalibratorRBF_H
