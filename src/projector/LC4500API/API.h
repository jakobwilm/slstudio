/*
 * API.h
 *
 * This module provides C callable APIs for each of the command supported by LightCrafter4500 platform and detailed in the programmer's guide.
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
*/

#ifndef API_H
#define API_H

/* Bit masks. */
#define BIT0        0x01
#define BIT1        0x02
#define BIT2        0x04
#define BIT3        0x08
#define BIT4        0x10
#define BIT5        0x20
#define BIT6        0x40
#define BIT7        0x80
#define BIT8      0x0100
#define BIT9      0x0200
#define BIT10     0x0400
#define BIT11     0x0800
#define BIT12     0x1000
#define BIT13     0x2000
#define BIT14     0x4000
#define BIT15     0x8000
#define BIT16 0x00010000
#define BIT17 0x00020000
#define BIT18 0x00040000
#define BIT19 0x00080000
#define BIT20 0x00100000
#define BIT21 0x00200000
#define BIT22 0x00400000
#define BIT23 0x00800000
#define BIT24 0x01000000
#define BIT25 0x02000000
#define BIT26 0x04000000
#define BIT27 0x08000000
#define BIT28 0x10000000
#define BIT29 0x20000000
#define BIT30 0x40000000
#define BIT31 0x80000000

#define STAT_BIT_FLASH_BUSY     BIT3
#define HID_MESSAGE_MAX_SIZE    512

typedef struct _hidmessageStruct
{
    struct _hidhead
    {
        struct _packetcontrolStruct
        {
            unsigned char dest		:3; /* 0 - ProjCtrl; 1 - RFC; 7 - Debugmsg */
            unsigned char reserved	:2;
            unsigned char nack		:1; /* Command Handler Error */
            unsigned char reply	:1; /* Host wants a reply from device */
            unsigned char rw		:1; /* Write = 0; Read = 1 */
        }flags;
        unsigned char seq;
        unsigned short length;
    }head;
    union
    {
        unsigned short cmd;
        unsigned char data[HID_MESSAGE_MAX_SIZE];
    }text;
}hidMessageStruct;

typedef struct _readCmdData
{
    unsigned char CMD2;
    unsigned char CMD3;
    unsigned short len;
}CmdFormat;

typedef struct _rectangle
{
    unsigned short firstPixel;
    unsigned short firstLine;
    unsigned short pixelsPerLine;
    unsigned short linesPerFrame;
}rectangle;

typedef enum
{
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
    SPLASH_LOAD,
    SPLASH_LOAD_TIMING,
    GPCLK_CONFIG,
    PULSE_GPIO_23,
    ENABLE_LCR_DEBUG,
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
    BL_PROG_MODE,
}LCR_CMD;

int LCR_SetInputSource(unsigned int source, unsigned int portWidth);
int LCR_GetInputSource(unsigned int *pSource, unsigned int *portWidth);
int LCR_SetPixelFormat(unsigned int format);
int LCR_GetPixelFormat(unsigned int *pFormat);
int LCR_SetPortClock(unsigned int clock);
int LCR_GetPortClock(unsigned int *pClock);
int LCR_SetDataChannelSwap(unsigned int port, unsigned int swap);
int LCR_GetDataChannelSwap(unsigned int *pPort, unsigned int *pSwap);
int LCR_SetFPD_Mode_Field(unsigned int PixelMappingMode, bool SwapPolarity, unsigned int FieldSignalSelect);
int LCR_GetFPD_Mode_Field(unsigned int *pPixelMappingMode, bool *pSwapPolarity, unsigned int *pFieldSignalSelect);
int LCR_SetPowerMode(bool);
int LCR_SetLongAxisImageFlip(bool);
bool LCR_GetLongAxisImageFlip();
int LCR_SetShortAxisImageFlip(bool);
bool LCR_GetShortAxisImageFlip();
int LCR_SetTPGSelect(unsigned int pattern);
int LCR_GetTPGSelect(unsigned int *pPattern);
int LCR_SetPWMInvert(bool invert);
int LCR_GetPWMInvert(bool *inverted);
int LCR_SetLedEnables(bool SeqCtrl, bool Red, bool Green, bool Blue);
int LCR_GetLedEnables(bool *pSeqCtrl, bool *pRed, bool *pGreen, bool *pBlue);
int LCR_GetVersion(unsigned int *pApp_ver, unsigned int *pAPI_ver, unsigned int *pSWConfig_ver, unsigned int *pSeqConfig_ver);
int LCR_SoftwareReset(void);
int LCR_GetStatus(unsigned char *pHWStatus, unsigned char *pSysStatus, unsigned char *pMainStatus);
int LCR_SetPWMEnable(unsigned int channel, bool Enable);
int LCR_GetPWMEnable(unsigned int channel, bool *pEnable);
int LCR_SetPWMConfig(unsigned int channel, unsigned int pulsePeriod, unsigned int dutyCycle);
int LCR_GetPWMConfig(unsigned int channel, unsigned int *pPulsePeriod, unsigned int *pDutyCycle);
int LCR_SetPWMCaptureConfig(unsigned int channel, bool enable, unsigned int sampleRate);
int LCR_GetPWMCaptureConfig(unsigned int channel, bool *pEnabled, unsigned int *pSampleRate);
int LCR_SetGPIOConfig(unsigned int pinNum, bool enAltFunc, bool altFunc1, bool dirOutput, bool outTypeOpenDrain, bool pinState);
int LCR_GetGPIOConfig(unsigned int pinNum, bool *pEnAltFunc, bool *pAltFunc1, bool *pDirOutput, bool *pOutTypeOpenDrain, bool *pState);
int LCR_GetLedCurrents(unsigned char *pRed, unsigned char *pGreen, unsigned char *pBlue);
int LCR_SetLedCurrents(unsigned char RedCurrent, unsigned char GreenCurrent, unsigned char BlueCurrent);
int LCR_SetDisplay(rectangle croppedArea, rectangle displayArea);
int LCR_GetDisplay(rectangle *pCroppedArea, rectangle *pDisplayArea);
int LCR_MemRead(unsigned int addr, unsigned int *readWord);
int LCR_MemWrite(unsigned int addr, unsigned int data);
int LCR_ValidatePatLutData(unsigned int *pStatus);
int LCR_SetPatternDisplayMode(bool external);
int LCR_GetPatternDisplayMode(bool *external);
int LCR_SetTrigOutConfig(unsigned int trigOutNum, bool invert, unsigned int rising, unsigned int falling);
int LCR_GetTrigOutConfig(unsigned int trigOutNum, bool *pInvert,unsigned int *pRising, unsigned int *pFalling);
int LCR_SetRedLEDStrobeDelay(unsigned char rising, unsigned char falling);
int LCR_SetGreenLEDStrobeDelay(unsigned char rising, unsigned char falling);
int LCR_SetBlueLEDStrobeDelay(unsigned char rising, unsigned char falling);
int LCR_GetRedLEDStrobeDelay(unsigned char *, unsigned char *);
int LCR_GetGreenLEDStrobeDelay(unsigned char *, unsigned char *);
int LCR_GetBlueLEDStrobeDelay(unsigned char *, unsigned char *);
int LCR_SetProgrammingMode(bool EnterProgMode);
int LCR_ExitProgrammingMode(void);
int LCR_GetProgrammingMode(bool *ProgMode);
int LCR_GetFlashManID(unsigned short *manID);
int LCR_GetFlashDevID(unsigned long long *devID);
int LCR_GetBLStatus(unsigned char *BL_Status);
int LCR_BLSpecialMode(unsigned int Mode);
int LCR_SetFlashAddr(unsigned int Addr);
int LCR_FlashSectorErase(void);
int LCR_SetDownloadSize(unsigned int dataLen);
int LCR_DownloadData(unsigned char *pByteArray, unsigned int dataLen);
void LCR_WaitForFlashReady(void);
int LCR_SetFlashType(unsigned char Type);
int LCR_CalculateFlashChecksum(void);
int LCR_GetFlashChecksum(unsigned int*checksum);
int LCR_SetMode(bool SLmode);
int LCR_GetMode(bool *pMode);
int LCR_LoadSplash(unsigned int index);
int LCR_GetSplashIndex(unsigned int *pIndex);
int LCR_SetTPGColor(unsigned short redFG, unsigned short greenFG, unsigned short blueFG, unsigned short redBG, unsigned short greenBG, unsigned short blueBG);
int LCR_GetTPGColor(unsigned short *pRedFG, unsigned short *pGreenFG, unsigned short *pBlueFG, unsigned short *pRedBG, unsigned short *pGreenBG, unsigned short *pBlueBG);
int LCR_ClearPatLut(void);
int LCR_AddToPatLut(int TrigType, int PatNum,int BitDepth,int LEDSelect,bool InvertPat, bool InsertBlack,bool BufSwap, bool trigOutPrev);
int LCR_GetPatLutItem(int index, int *pTrigType, int *pPatNum,int *pBitDepth,int *pLEDSelect,bool *pInvertPat, bool *pInsertBlack,bool *pBufSwap, bool *pTrigOutPrev);
int LCR_SendPatLut(void);
int LCR_SendSplashLut(unsigned char *lutEntries, unsigned int numEntries);
int LCR_GetPatLut(int numEntries);
int LCR_GetSplashLut(unsigned char *pLut, int numEntries);
int LCR_SetPatternTriggerMode(bool);
int LCR_GetPatternTriggerMode(bool *);
int LCR_PatternDisplay(int Action);
int LCR_SetPatternConfig(unsigned int numLutEntries, bool repeat, unsigned int numPatsForTrigOut2, unsigned int numSplash);
int LCR_GetPatternConfig(unsigned int *pNumLutEntries, bool *pRepeat, unsigned int *pNumPatsForTrigOut2, unsigned int *pNumSplash);
int LCR_SetExpsosure_FramePeriod(unsigned int exposurePeriod, unsigned int framePeriod);
int LCR_GetExposure_FramePeriod(unsigned int *pExposure, unsigned int *pFramePeriod);
int LCR_SetTrigIn1Delay(unsigned int Delay);
int LCR_GetTrigIn1Delay(unsigned int *pDelay);
int LCR_SetInvertData(bool invert);
int LCR_PWMCaptureRead(unsigned int channel, unsigned int *pLowPeriod, unsigned int *pHighPeriod);
int LCR_SetGeneralPurposeClockOutFreq(unsigned int clkId, bool enable, unsigned int clkDivider);
int LCR_GetGeneralPurposeClockOutFreq(unsigned int clkId, bool *pEnabled, unsigned int *pClkDivider);
int LCR_MeasureSplashLoadTiming(unsigned int startIndex, unsigned int numSplash);
int LCR_ReadSplashLoadTiming(unsigned int *pTimingData);


#endif // API_H
