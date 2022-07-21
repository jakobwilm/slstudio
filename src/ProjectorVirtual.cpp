#include "ProjectorVirtual.h"

#include <QThread>

ProjectorVirtual::ProjectorVirtual(unsigned int) { timer.start(); }

void ProjectorVirtual::waitForProjection() {
  // Wait till 17 msec have elapsed on time
  unsigned int elapsed = timer.elapsed();
  if (elapsed < 33)
    QThread::msleep(33 - elapsed);

  // Reset time
  timer.restart();
}

void ProjectorVirtual::displayPattern(unsigned int) {
  this->waitForProjection();
}

void ProjectorVirtual::displayTexture(const unsigned char *, unsigned int,
                                        unsigned int) {
  this->waitForProjection();
}

void ProjectorVirtual::displayBlack() { this->waitForProjection(); }

void ProjectorVirtual::displayWhite() { this->waitForProjection(); }

void ProjectorVirtual::getScreenRes(unsigned int *nx, unsigned int *ny) {
  *nx = 912;
  *ny = 1140;
}

ProjectorVirtual::~ProjectorVirtual() {}
