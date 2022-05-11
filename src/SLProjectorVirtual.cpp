#include "SLProjectorVirtual.h"

#include <QTest>

SLProjectorVirtual::SLProjectorVirtual(unsigned int) { timer.start(); }

void SLProjectorVirtual::waitForProjection() {
  // Wait till 17 msec have elapsed on time
  unsigned int elapsed = timer.elapsed();
  if (elapsed < 33)
    QTest::qSleep(33 - elapsed);

  // Reset time
  timer.restart();
}

void SLProjectorVirtual::displayPattern(unsigned int) {
  this->waitForProjection();
}

void SLProjectorVirtual::displayTexture(const unsigned char *, unsigned int,
                                        unsigned int) {
  this->waitForProjection();
}

void SLProjectorVirtual::displayBlack() { this->waitForProjection(); }

void SLProjectorVirtual::displayWhite() { this->waitForProjection(); }

void SLProjectorVirtual::getScreenRes(unsigned int *nx, unsigned int *ny) {
  *nx = 912;
  *ny = 1140;
}

SLProjectorVirtual::~SLProjectorVirtual() {}
