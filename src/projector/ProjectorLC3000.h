#pragma once

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
  // Define preset pattern sequence and upload to projector
  void setPatterns(const std::vector<const unsigned char *> patterns,
                   unsigned int patternWidth,
                   unsigned int patternHeight) override;
  void displayPattern(unsigned int patternNumber) override;
  void displayBlack() override;
  void displayWhite() override;
  void getScreenRes(unsigned int *nx, unsigned int *ny) override;
  bool requiresPatternUpload() override { return false; }
  ~ProjectorLC3000();

private:
  LCR_PatternSeqSetting_t patternSeqSettings;
  bool ptn_seq_mode;
  ErrorCode_t res;
};
