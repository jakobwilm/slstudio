/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#ifndef SLPROJECTORVIRTUAL_H
#define SLPROJECTORVIRTUAL_H

#include "Projector.h"

#include <QElapsedTimer>

// Virtual Projector implementation. Does essentially nothing.
class SLProjectorVirtual : public Projector {
public:
  // Interface function
  SLProjectorVirtual(unsigned int);
  void setPattern(unsigned int /*patternNumber*/, const unsigned char * /*tex*/,
                  unsigned int /*texWidth*/, unsigned int /*texHeight*/) {}
  void displayPattern(unsigned int /*patternNumber*/);
  void displayTexture(const unsigned char * /*tex*/, unsigned int /*width*/,
                      unsigned int /*height*/);
  void displayBlack();
  void displayWhite();
  void getScreenRes(unsigned int * /*nx*/, unsigned int * /*ny*/);
  ~SLProjectorVirtual();

private:
  void waitForProjection();
  QElapsedTimer timer;
};

#endif
