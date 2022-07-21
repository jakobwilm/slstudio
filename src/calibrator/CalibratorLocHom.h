/*
 * CalibratorLocHome -- Calibrate using "local homographies" as proposed by
 * Moreno, Taubin.
 */

#pragma once

#include "Calibrator.h"

#include "Codec.h"

using namespace std;

class CalibratorLocHom : public Calibrator {
  Q_OBJECT
public:
  CalibratorLocHom(QObject *parent, unsigned int _screenCols,
                   unsigned int _screenRows);
  CalibrationData calibrate();
  ~CalibratorLocHom() {}

private:
  std::unique_ptr<Encoder> encoder;
  std::unique_ptr<Decoder> decoder;
};
