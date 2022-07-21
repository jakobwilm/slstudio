#pragma once

#include <iostream>
#include <vector>

// Abstract Projector base class
class Projector {
public:
  // Interface function
  Projector() {}
  // Define preset pattern sequence
  virtual void setPatterns(const std::vector<const unsigned char *> patterns,
                           unsigned int patternWidth,
                           unsigned int patternHeight) = 0;
  virtual void displayPattern(unsigned int patternNumber) = 0;
  // Monochrome color display
  virtual void displayBlack() = 0;
  virtual void displayWhite() = 0;
  virtual void getScreenRes(unsigned int *nx, unsigned int *ny) = 0;
  virtual bool requiresPatternUpload() = 0;
  virtual ~Projector() {}
};
