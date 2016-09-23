/*
 * CalibratorRBF -- Calibrate using radial basis functions, as proposed by us.
*/

#ifndef CalibratorRBF_H
#define CalibratorRBF_H

#include "Calibrator.h"

#include "Codec.h"

using namespace std;

class SLALGORITHM_EXPORT CalibratorRBF : public Calibrator {
    public:
        CalibratorRBF(unsigned int _screenCols, unsigned int _screenRows);
        CalibrationData calibrate(const int checkerSize, const int checkerRows, const int checkerCols);
        ~CalibratorRBF(){delete encoder; delete decoder;}
    private:
        Encoder *encoder;
        Decoder *decoder;
};

#endif // CalibratorRBF_H
