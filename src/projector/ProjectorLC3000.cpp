#include "ProjectorLC3000.h"

// Additions to the LC3000 API
#include "LC3000API/lcr_packetizer.h"
ErrorCode_t LCR_CMD_DisplayPattern(LCR_PatternCount_t PtnNumber) {
  /* Generate packet */
  LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0405);
  LCR_CMD_PKT_PutInt(PtnNumber, 2);
  if (LCR_CMD_PKT_SendCommand())
    return FAIL;

  return SUCCESS;
}

#include "LC3000API/bitmap_image.hpp"

static void ReportError(ErrorCode_t res, const char *place) {
  if (res != SUCCESS) {
    printf("ProjectorLC3000: Error at %s (%d)\n", place, res);
    fflush(stdout);
  }
}

ProjectorLC3000::ProjectorLC3000(unsigned int) {

  // Make connection
  res = LCR_CMD_Open();
  ReportError(res, "LCR_CMD_Open()");

  // Set power on (if in standby)
  res = LCR_CMD_SetPowerMode(PWR_NORMAL);
  ReportError(res, "LCR_CMD_SetPowerMode()");

  // Set LED currents to max brightness
  LCR_LEDCurrent_t ledCurrents = {255, 255, 255};
  res = LCR_CMD_SetLEDCurrent(&ledCurrents);
  ReportError(res, "LCR_CMD_SetLEDCurrent()");

  // Set display settings to no rotate, no long axis flix, short axis flip
  LCR_DisplaySetting_t displaySettings = {0, 0, 1};
  res = LCR_CMD_SetDisplaySetting(&displaySettings);
  ReportError(res, "LCR_CMD_SetDisplaySetting()");

  // Set trigger output settings
  LCR_CamTriggerSetting_t triggerSettings;
  triggerSettings.Enable = 1;
  triggerSettings.Source = 0;
  triggerSettings.Polarity = TRIGGER_EDGE_POS;
  triggerSettings.Delay = 0;
  triggerSettings.PulseWidth = 200; // us
  res = LCR_CMD_SetCamTriggerSetting(&triggerSettings);
  ReportError(res, "LCR_CMD_SetCamTriggerSetting()");

  // Set default pattern sequence settings
  patternSeqSettings.BitDepth = 8;
  patternSeqSettings.NumPatterns = 1;
  patternSeqSettings.PatternType = PTN_TYPE_NORMAL;
  patternSeqSettings.InputTriggerType = TRIGGER_TYPE_SW;
  patternSeqSettings.InputTriggerDelay = 0;
  patternSeqSettings.AutoTriggerPeriod = 0;
  patternSeqSettings.ExposureTime = 16666; // us
  patternSeqSettings.LEDSelect = LED_RED;
  patternSeqSettings.Repeat = 1;
  res = LCR_CMD_SetPatternSeqSetting(&patternSeqSettings);
  ReportError(res, "LCR_CMD_SetPatternSeqSetting()");

  // project white
  this->displayWhite();
}

void ProjectorLC3000::setPatterns(
    const std::vector<const unsigned char *> patterns,
    unsigned int patternWidth, unsigned int patternHeight) {

  // Set pattern count
  unsigned int nPatterns = patterns.size();
  patternSeqSettings.NumPatterns = nPatterns;
  res = LCR_CMD_SetPatternSeqSetting(&patternSeqSettings);
  ReportError(res, "LCR_CMD_SetPatternSeqSetting()");

  for (size_t p = 0; p < nPatterns; ++p) {

    // Tile texture
    char texTiled[608][684][3];
    for (unsigned int j = 0; j < 608; j++) {
      int jIdx = j % patternWidth;
      for (unsigned int i = 0; i < 684; i++) {
        int iIdx = i % patternHeight;
        texTiled[j][i][0] = patterns[p][iIdx * patternWidth * 3 + jIdx * 3 + 0];
        texTiled[j][i][1] = patterns[p][iIdx * patternWidth * 3 + jIdx * 3 + 1];
        texTiled[j][i][2] = patterns[p][iIdx * patternWidth * 3 + jIdx * 3 + 2];
      }
    }
    // Bitmap version of texture
    bitmap_image img(608, 684);
    for (unsigned int j = 0; j < 608; j++)
      for (unsigned int i = 0; i < 684; i++)
        img.set_pixel(j, i, texTiled[j][i][0], texTiled[j][i][1],
                      texTiled[j][i][2]);

    // Save
    img.save_image("tex.bmp");

    // Set as pattern
    res = LCR_CMD_DefinePatternBMP(p, "tex.bmp");
    ReportError(res, "LCR_CMD_DefinePattern()");
  }
}

void ProjectorLC3000::displayPattern(unsigned int patternNumber) {

  // Set display mode
  if (this->ptn_seq_mode == false) {
    res = LCR_CMD_SetDisplayMode(DISP_MODE_PTN_SEQ);
    ReportError(res, "LCR_CMD_SetDisplayMode()");
    this->ptn_seq_mode = true;
  }

  res = LCR_CMD_DisplayPattern(patternNumber);
  ReportError(res, "LCR_CMD_DisplayPattern()");
}

void ProjectorLC3000::displayBlack() {
  // Set display to image/static color
  res = LCR_CMD_SetDisplayMode(DISP_MODE_IMAGE);
  ReportError(res, "LCR_CMD_SetDisplayMode()");

  res = LCR_CMD_DisplayStaticColor(0);
  ReportError(res, "LCR_CMD_DisplayStaticColor()");
}
void ProjectorLC3000::displayWhite() {
  // Set display to image/static color
  res = LCR_CMD_SetDisplayMode(DISP_MODE_IMAGE);
  ReportError(res, "LCR_CMD_SetDisplayMode()");

  res = LCR_CMD_DisplayStaticColor((255 << 16) | (255 << 8) | (255));
  ReportError(res, "LCR_CMD_DisplayStaticColor()");
  this->ptn_seq_mode = false;
}

void ProjectorLC3000::getScreenRes(unsigned int *nx, unsigned int *ny) {
  *nx = 608;
  *ny = 684;
}

ProjectorLC3000::~ProjectorLC3000() {
  res = LCR_CMD_Close();
  ReportError(res, "LCR_CMD_Close()");
}
