/*
 *
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#pragma once

#include "Projector.h"

#include <QElapsedTimer>

// Virtual Projector implementation. Does essentially nothing.
class ProjectorVirtual : public Projector {
public:
  // Interface function
  ProjectorVirtual(unsigned int);
  void setPatterns(const std::vector<const unsigned char *> patterns,
                   unsigned int patternWidth,
                   unsigned int patternHeight) override {}
  void displayPattern(unsigned int /*patternNumber*/) override;
  void displayTexture(const unsigned char * /*tex*/, unsigned int /*width*/,
                      unsigned int /*height*/);
  void displayBlack() override;
  void displayWhite() override;
  void getScreenRes(unsigned int * /*nx*/, unsigned int * /*ny*/) override;
  bool requiresPatternUpload() override { return false; }
  ~ProjectorVirtual();

private:
  void waitForProjection();
  QElapsedTimer timer;
};
