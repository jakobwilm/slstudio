/*
 * CalibratorRBF -- Calibrate using radial basis functions, as proposed by us.
 */

#pragma once

#include "Calibrator.h"

#include "Codec.h"

using namespace std;

class CalibratorRBF : public Calibrator {
  Q_OBJECT
public:
  CalibratorRBF(QObject *parent, unsigned int _screenCols,
                unsigned int _screenRows);
  CalibrationData calibrate();
  ~CalibratorRBF() {}

private:
  std::unique_ptr<Encoder> encoder;
  std::unique_ptr<Decoder> decoder;
};
