#ifndef PROJECTORLC3000_H
#define PROJECTORLC3000_H

#include <iostream>
#include <sys/types.h>
#include <vector>

#include "Projector.h"

#include "LC3000API/lcr_cmd.h"

// Projector implementation for LightCrafter 3000 USB Api
class ProjectorLC3000 : public Projector {
public:
  // Interface function
  ProjectorLC3000(unsigned int);
  // Define preset pattern sequence and upload to GPU
  void setPatterns(const std::vector<const unsigned char *> patterns,
                   unsigned int patternWidth,
                   unsigned int patternHeight) override;
  void displayPattern(unsigned int patternNumber) override;
  void displayBlack() override;
  void displayWhite() override;
  void getScreenRes(unsigned int *nx, unsigned int *ny) override;
  ~ProjectorLC3000();
  bool ptn_seq_mode;

private:
  LCR_PatternSeqSetting_t patternSeqSettings;
  ErrorCode_t res;
};

#endif
