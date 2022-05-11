/*
 * dlpc350_api.h
 *
 * This module provides C callable APIs for each of the command supported
 * by LightCrafter4500 platform and detailed in the programmer's guide.
 *
 * Copyright (C) {2015} Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef DLPC350_API_H
#define DLPC350_API_H

#include "dlpc350_common.h"

typedef struct _hidmessageStruct {
  struct _hidhead {
    struct _packetcontrolStruct {
      unsigned char dest : 3; /* 0 - ProjCtrl; 1 - RFC; 7 - Debugmsg */
      unsigned char reserved : 2;
      unsigned char nack : 1;  /* Command Handler Error */
      unsigned char reply : 1; /* Host wants a reply from device */
      unsigned char rw : 1;    /* Write = 0; Read = 1 */
    } flags;
    unsigned char seq;
    unsigned short length;
  } head;
  union {
    unsigned short cmd;
    unsigned char data[HID_MESSAGE_MAX_SIZE];
  } text;
} hidMessageStruct;

typedef struct _readCmdData {
  unsigned char CMD2;
  unsigned char CMD3;
  unsigned short len;
} CmdFormat;

typedef struct _rectangle {
  unsigned short firstPixel;
  unsigned short firstLine;
  unsigned short pixelsPerLine;
  unsigned short linesPerFrame;
} rectangle;

typedef struct _vidSigStatus {
  unsigned char Status;
  unsigned int HRes;
  unsigned int VRes;
  unsigned char RSVD;
  unsigned char HSyncPol;
  unsigned char VSyncPol;
  unsigned long int PixClock;
  unsigned int HFreq;
  unsigned int VFreq;
  unsigned int TotPixPerLine;
  unsigned int TotLinPerFrame;
  unsigned int ActvPixPerLine;
  unsigned int ActvLinePerFrame;
  unsigned int FirstActvPix;
  unsigned int FirstActvLine;
} VideoSigStatus;

typedef enum {
  VID_SIG_STAT,
  SOURCE_SEL,
  PIXEL_FORMAT,
  CLK_SEL,
  CHANNEL_SWAP,
  FPD_MODE,
  CURTAIN_COLOR,
  POWER_CONTROL,
  FLIP_LONG,
  FLIP_SHORT,
  TPG_SEL,
  PWM_INVERT,
  LED_ENABLE,
  GET_VERSION,
  GET_FIRMWAE_TAG_INFO,
  SW_RESET,
  DMD_PARK,
  BUFFER_FREEZE,
  STATUS_HW,
  STATUS_SYS,
  STATUS_MAIN,
  CSC_DATA,
  GAMMA_CTL,
  BC_CTL,
  PWM_ENABLE,
  PWM_SETUP,
  PWM_CAPTURE_CONFIG,
  GPIO_CONFIG,
  LED_CURRENT,
  DISP_CONFIG,
  TEMP_CONFIG,
  TEMP_READ,
  MEM_CONTROL,
  I2C_CONTROL,
  LUT_VALID,
  DISP_MODE,
  TRIG_OUT1_CTL,
  TRIG_OUT2_CTL,
  RED_STROBE_DLY,
  GRN_STROBE_DLY,
  BLU_STROBE_DLY,
  PAT_DISP_MODE,
  PAT_TRIG_MODE,
  PAT_START_STOP,
  BUFFER_SWAP,
  BUFFER_WR_DISABLE,
  CURRENT_RD_BUFFER,
  PAT_EXPO_PRD,
  INVERT_DATA,
  PAT_CONFIG,
  MBOX_ADDRESS,
  MBOX_CONTROL,
  MBOX_DATA,
  TRIG_IN1_DELAY,
  TRIG_IN2_CONTROL,
  IMAGE_LOAD,
  IMAGE_LOAD_TIMING,
  I2C0_CTRL,
  MBOX_EXP_DATA,
  MBOX_EXP_ADDRESS,
  EXP_PAT_CONFIG,
  NUM_IMAGE_IN_FLASH,
  I2C0_STAT,
  GPCLK_CONFIG,
  PULSE_GPIO_23,
  ENABLE_DLPC350_DEBUG,
  TPG_COLOR,
  PWM_CAPTURE_READ,
  PROG_MODE,
  BL_STATUS,
  BL_SPL_MODE,
  BL_GET_MANID,
  BL_GET_DEVID,
  BL_GET_CHKSUM,
  BL_SET_SECTADDR,
  BL_SECT_ERASE,
  BL_SET_DNLDSIZE,
  BL_DNLD_DATA,
  BL_FLASH_TYPE,
  BL_CALC_CHKSUM,
  BL_PROG_MODE
} DLPC350_CMD;

int DLPC350_GetVideoSignalStatus(VideoSigStatus *vidSigStat);
int DLPC350_SetInputSource(unsigned int source, unsigned int portWidth);
int DLPC350_GetInputSource(unsigned int *pSource, unsigned int *portWidth);
int DLPC350_SetPixelFormat(unsigned int format);
int DLPC350_GetPixelFormat(unsigned int *pFormat);
int DLPC350_SetPortClock(unsigned int clock);
int DLPC350_GetPortClock(unsigned int *pClock);
int DLPC350_SetDataChannelSwap(unsigned int port, unsigned int swap);
int DLPC350_GetDataChannelSwap(unsigned int *pPort, unsigned int *pSwap);
int DLPC350_SetFPD_Mode_Field(unsigned int PixelMappingMode, bool SwapPolarity,
                              unsigned int FieldSignalSelect);
int DLPC350_GetFPD_Mode_Field(unsigned int *pPixelMappingMode,
                              bool *pSwapPolarity,
                              unsigned int *pFieldSignalSelect);
int DLPC350_SetPowerMode(bool);
int DLPC350_GetPowerMode(bool *Standby);
int DLPC350_SetLongAxisImageFlip(bool);
bool DLPC350_GetLongAxisImageFlip();
int DLPC350_SetShortAxisImageFlip(bool);
bool DLPC350_GetShortAxisImageFlip();
int DLPC350_SetTPGSelect(unsigned int pattern);
int DLPC350_GetTPGSelect(unsigned int *pPattern);
int DLPC350_SetLEDPWMInvert(bool invert);
int DLPC350_GetLEDPWMInvert(bool *inverted);
int DLPC350_SetLedEnables(bool SeqCtrl, bool Red, bool Green, bool Blue);
int DLPC350_GetLedEnables(bool *pSeqCtrl, bool *pRed, bool *pGreen,
                          bool *pBlue);
int DLPC350_GetVersion(unsigned int *pApp_ver, unsigned int *pAPI_ver,
                       unsigned int *pSWConfig_ver,
                       unsigned int *pSeqConfig_ver);
int DLPC350_GetFirmwareVersion(unsigned int *pFW_ver);
int DLPC350_SoftwareReset(void);
int DLPC350_GetStatus(unsigned char *pHWStatus, unsigned char *pSysStatus,
                      unsigned char *pMainStatus);
int DLPC350_SetPWMEnable(unsigned int channel, bool Enable);
int DLPC350_GetPWMEnable(unsigned int channel, bool *pEnable);
int DLPC350_SetPWMConfig(unsigned int channel, unsigned int pulsePeriod,
                         unsigned int dutyCycle);
int DLPC350_GetPWMConfig(unsigned int channel, unsigned int *pPulsePeriod,
                         unsigned int *pDutyCycle);
int DLPC350_SetPWMCaptureConfig(unsigned int channel, bool enable,
                                unsigned int sampleRate);
int DLPC350_GetPWMCaptureConfig(unsigned int channel, bool *pEnabled,
                                unsigned int *pSampleRate);
int DLPC350_SetGPIOConfig(unsigned int pinNum, bool enAltFunc, bool altFunc1,
                          bool dirOutput, bool outTypeOpenDrain, bool pinState);
int DLPC350_GetGPIOConfig(unsigned int pinNum, bool *pEnAltFunc,
                          bool *pAltFunc1, bool *pDirOutput,
                          bool *pOutTypeOpenDrain, bool *pState);
int DLPC350_GetLedCurrents(unsigned char *pRed, unsigned char *pGreen,
                           unsigned char *pBlue);
int DLPC350_SetLedCurrents(unsigned char RedCurrent, unsigned char GreenCurrent,
                           unsigned char BlueCurrent);
int DLPC350_SetDisplay(rectangle croppedArea, rectangle displayArea);
int DLPC350_GetDisplay(rectangle *pCroppedArea, rectangle *pDisplayArea);
int DLPC350_MemRead(unsigned int addr, unsigned int *readWord);
int DLPC350_MemWrite(unsigned int addr, unsigned int data);
int DLPC350_ValidatePatLutData(unsigned int *pStatus);
int DLPC350_StartPatLutValidate();
int DLPC350_CheckPatLutValidate(bool *ready, unsigned int *pStatus);
int DLPC350_SetPatternDisplayMode(bool external);
int DLPC350_GetPatternDisplayMode(bool *external);
int DLPC350_SetTrigOutConfig(unsigned int trigOutNum, bool invert,
                             unsigned int rising, unsigned int falling);
int DLPC350_GetTrigOutConfig(unsigned int trigOutNum, bool *pInvert,
                             unsigned int *pRising, unsigned int *pFalling);
int DLPC350_SetRedLEDStrobeDelay(unsigned char rising, unsigned char falling);
int DLPC350_SetGreenLEDStrobeDelay(unsigned char rising, unsigned char falling);
int DLPC350_SetBlueLEDStrobeDelay(unsigned char rising, unsigned char falling);
int DLPC350_GetRedLEDStrobeDelay(unsigned char *, unsigned char *);
int DLPC350_GetGreenLEDStrobeDelay(unsigned char *, unsigned char *);
int DLPC350_GetBlueLEDStrobeDelay(unsigned char *, unsigned char *);
int DLPC350_EnterProgrammingMode(void);
int DLPC350_ExitProgrammingMode(void);
int DLPC350_GetProgrammingMode(bool *ProgMode);
int DLPC350_GetFlashManID(unsigned short *manID);
int DLPC350_GetFlashDevID(unsigned long long *devID);
int DLPC350_GetBLStatus(unsigned char *BL_Status);
int DLPC350_SetFlashAddr(unsigned int Addr);
int DLPC350_FlashSectorErase(void);
int DLPC350_SetUploadSize(unsigned long int dataLen);
int DLPC350_UploadData(unsigned char *pByteArray, unsigned int dataLen);
void DLPC350_WaitForFlashReady(void);
int DLPC350_SetFlashType(unsigned char Type);
int DLPC350_CalculateFlashChecksum(void);
int DLPC350_GetFlashChecksum(unsigned int *checksum);
int DLPC350_SetMode(bool SLmode);
int DLPC350_GetMode(bool *pMode);
int DLPC350_LoadImageIndex(unsigned int index);
int DLPC350_GetImageIndex(unsigned int *pIndex);
int DLPC350_GetNumImagesInFlash(unsigned int *pNumImgInFlash);
int DLPC350_SetTPGColor(unsigned short redFG, unsigned short greenFG,
                        unsigned short blueFG, unsigned short redBG,
                        unsigned short greenBG, unsigned short blueBG);
int DLPC350_GetTPGColor(unsigned short *pRedFG, unsigned short *pGreenFG,
                        unsigned short *pBlueFG, unsigned short *pRedBG,
                        unsigned short *pGreenBG, unsigned short *pBlueBG);
int DLPC350_ClearPatLut(void);
int DLPC350_ClearExpLut(void);
int DLPC350_AddToPatLut(int TrigType, int PatNum, int BitDepth, int LEDSelect,
                        bool InvertPat, bool InsertBlack, bool BufSwap,
                        bool trigOutPrev);
int DLPC350_AddToExpLut(int TrigType, int PatNum, int BitDepth, int LEDSelect,
                        bool InvertPat, bool InsertBlack, bool BufSwap,
                        bool trigOutPrev, unsigned int exp_time_us,
                        unsigned int ptn_frame_period_us);
int DLPC350_GetPatLutItem(int index, int *pTrigType, int *pPatNum,
                          int *pBitDepth, int *pLEDSelect, bool *pInvertPat,
                          bool *pInsertBlack, bool *pBufSwap,
                          bool *pTrigOutPrev);
int DLPC350_GetVarExpPatLutItem(int index, int *pTrigType, int *pPatNum,
                                int *pBitDepth, int *pLEDSelect,
                                bool *pInvertPat, bool *pInsertBlack,
                                bool *pBufSwap, bool *pTrigOutPrev,
                                int *pPatExp, int *pPatPeriod);
int DLPC350_SendPatLut(void);
int DLPC350_SendVarExpPatLut(void);
int DLPC350_SendImageLut(unsigned char *lutEntries, unsigned int numEntries);
int DLPC350_SendVarExpImageLut(unsigned char *lutEntries,
                               unsigned int numEntries);
int DLPC350_GetPatLut(int numEntries);
int DLPC350_GetVarExpPatLut(int numEntries);
int DLPC350_GetImageLut(unsigned char *pLut, int numEntries);
int DLPC350_GetvarExpImageLut(unsigned char *pLut, int numEntries);
int DLPC350_SetPatternTriggerMode(int);
int DLPC350_GetPatternTriggerMode(int *);
int DLPC350_PatternDisplay(unsigned int Action);
int DLPC350_GetPatternDisplay(unsigned int *pAction);
int DLPC350_SetVarExpPatternConfig(unsigned int numLutEntries,
                                   unsigned int numPatsForTrigOut2,
                                   unsigned int numImages, bool repeat);
int DLPC350_GetVarExpPatternConfig(unsigned int *pNumLutEntries,
                                   unsigned int *pNumPatsForTrigOut2,
                                   unsigned int *pNumImages, bool *pRepeat);
int DLPC350_SetPatternConfig(unsigned int numLutEntries, bool repeat,
                             unsigned int numPatsForTrigOut2,
                             unsigned int numImages);
int DLPC350_GetPatternConfig(unsigned int *pNumLutEntries, bool *pRepeat,
                             unsigned int *pNumPatsForTrigOut2,
                             unsigned int *pNumImages);
int DLPC350_SetExposure_FramePeriod(unsigned int exposurePeriod,
                                    unsigned int framePeriod);
int DLPC350_GetExposure_FramePeriod(unsigned int *pExposure,
                                    unsigned int *pFramePeriod);
int DLPC350_SetTrigIn1Delay(unsigned int Delay);
int DLPC350_GetTrigIn1Delay(unsigned int *pDelay);
int DLPC350_SetTrigIn2Pol(bool isFallingEdge);
int DLPC350_GetTrigIn2Pol(bool *pIsFallingEdge);
int DLPC350_SetInvertData(bool invert);
int DLPC350_PWMCaptureRead(unsigned int channel, unsigned int *pLowPeriod,
                           unsigned int *pHighPeriod);
int DLPC350_SetGeneralPurposeClockOutFreq(unsigned int clkId, bool enable,
                                          unsigned int clkDivider);
int DLPC350_GetGeneralPurposeClockOutFreq(unsigned int clkId, bool *pEnabled,
                                          unsigned int *pClkDivider);
int DLPC350_MeasureImageLoadTiming(unsigned int startIndex,
                                   unsigned int numFlash);
int DLPC350_ReadImageLoadTiming(unsigned int *pTimingData);
int DLPC350_SetFreeze(bool Freeze);
int DLPC350_GetFirmwareTagInfo(unsigned char *pFwTagInfo);
int DLPC350_I2C0WriteData(bool is7Bit, unsigned int sclClk,
                          unsigned int devAddr, unsigned int numWriteBytes,
                          unsigned char *pWdata);
int DLPC350_I2C0ReadData(bool is7Bit, unsigned int sclClk, unsigned int devAddr,
                         unsigned int numWriteBytes, unsigned int numReadBytes,
                         unsigned char *pWData, unsigned char *pRdata);
int DLPC350_I2C0TranStat(unsigned char *pStat);
bool DLPC350_isSourceLocked(VideoSigStatus VidSig);
#endif // API_H
