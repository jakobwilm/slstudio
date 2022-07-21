#pragma once

#include "Projector.h"
#include <iostream>
#include <map>
#include <sys/types.h>

// Projector implementation for LightCrafter 4500 USB Api
class ProjectorLC4500 : public Projector {
public:
  // Interface function
  ProjectorLC4500(unsigned int);
  // Define preset pattern sequence and upload to GPU
  void setPatterns(const std::vector<const unsigned char *> patterns,
                   unsigned int patternWidth,
                   unsigned int patternHeight) override;
  void displayPattern(unsigned int i) override;
  void displayBlack() override;
  void displayWhite() override;
  void getScreenRes(unsigned int *nx, unsigned int *ny) override;
  bool requiresPatternUpload() override { return true; }
  ~ProjectorLC4500();

private:
  unsigned int nPatterns;
  bool isRunning;
  bool setToVideoMode();
  bool setToPatternMode();
};
