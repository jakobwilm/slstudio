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
  void setPatterns(const std::vector<const unsigned char *> patterns,
                   unsigned int patternWidth,
                   unsigned int patternHeight) override {}
  void displayPattern(unsigned int /*patternNumber*/) override;
  void displayTexture(const unsigned char * /*tex*/, unsigned int /*width*/,
                      unsigned int /*height*/);
  void displayBlack() override;
  void displayWhite() override;
  void getScreenRes(unsigned int * /*nx*/, unsigned int * /*ny*/) override;
  ~SLProjectorVirtual();

private:
  void waitForProjection();
  QElapsedTimer timer;
};

#endif
