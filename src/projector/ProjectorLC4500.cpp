#include "ProjectorLC4500.h"

#include "LC4500API/dlpc350_BMPParser.h"
#include "LC4500API/dlpc350_api.h"
#include "LC4500API/dlpc350_firmware.h"
#include "LC4500API/dlpc350_usb.h"

#include <QThread>
#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <fstream>

void showError(std::string err) {
  std::cerr << "ProjectorLC4500: " << err.c_str() << std::endl;
}

ProjectorLC4500::ProjectorLC4500(unsigned int)
    : nPatterns(0), isRunning(false) {

  std::cout << "ProjectorLC4500: preparing LightCrafter 4500 for duty... "
            << std::endl;

  // Initialize usb connection
  if (DLPC350_USB_Init()) {
    showError("Could not init USB!");
  }
  if (DLPC350_USB_Open()) {
    showError("Could not open USB!");
  }
  if (!DLPC350_USB_IsConnected()) {
    showError("Could not connect.");
  }
  //    unsigned char HWStatus, SysStatus, MainStatus;
  //    while(DLPC350_GetStatus(&HWStatus, &SysStatus, &MainStatus) != 0){
  //        std::cout << ".";
  //        continue;
  //    }

  // Make sure LC is not in standby
  bool isStandby;
  DLPC350_GetPowerMode(&isStandby);
  if (isStandby) {
    DLPC350_SetPowerMode(0);
    QThread::msleep(2000);
  }
  while (isStandby) {
    QThread::msleep(50);
    DLPC350_GetPowerMode(&isStandby);
  }

  // Set LED selection
  const bool SeqCtrl = false; // manual (false) or automatic (true)
  const bool LEDRed = true;
  const bool LEDGreen = true;
  const bool LEDBlue = true;
  DLPC350_SetLedEnables(SeqCtrl, LEDRed, LEDGreen, LEDBlue);

  // Set LED currents
  DLPC350_SetLEDPWMInvert(false);

  const unsigned char RedCurrent = 50;
  const unsigned char GreenCurrent = 50;
  const unsigned char BlueCurrent = 50;
  DLPC350_SetLedCurrents(255 - RedCurrent, 255 - GreenCurrent,
                         255 - BlueCurrent);

  // Set to pattern sequence mode
  const bool patternSequenceMode = false;
  if (!DLPC350_SetMode(patternSequenceMode)) {
    showError("Error Setting Pattern Sequence Mode");
  }
}

void ProjectorLC4500::setPatterns(
    const std::vector<const unsigned char *> patterns,
    unsigned int patternWidth, unsigned int patternHeight) {

  nPatterns = patterns.size();

  int ret = -1;

  std::string fwFilePath = "./DLPR350PROM_v4.4.0.bin"; // in current directory

  char const *envPath =
      getenv("LC4500_FW_PATH"); // firmware filePath given in env. var.
  if (envPath != nullptr) {
    fwFilePath = envPath;
  }

  std::ifstream inFile(fwFilePath, std::ios::binary);
  if (!inFile.is_open()) {
    std::cerr << "Unable to open firmware image file. Export file path to env. "
                 "var. LC4500_FW_PATH.\n";
    return;
  }

  std::vector<char> byteArray((std::istreambuf_iterator<char>(inFile)),
                              std::istreambuf_iterator<char>());

  inFile.close();

  ret = DLPC350_Frmw_CopyAndVerifyImage((unsigned char *)byteArray.data(),
                                        byteArray.size());
  if (ret < 0) {
    std::cerr << "ProjectorLC4500: invalid firmware image.\n";
    return;
  }

  //  auto v = DLPC350_Frmw_GetVersionNumber();

  // ensure version 4. 4 of firmware such that the following flash addresses are
  // as expected
  unsigned int API_ver, App_ver, SWConfig_ver, SeqConfig_ver;
  ret = DLPC350_GetVersion(&App_ver, &API_ver, &SWConfig_ver, &SeqConfig_ver);
  if (ret < 0) {
    std::cerr << "ProjectorLC4500: could not get version number.\n";
    return;
  }

  std::string versionStr = std::to_string(App_ver >> 24) + '.' +
                           std::to_string((App_ver << 8) >> 24) + '.' +
                           std::to_string((App_ver << 16) >> 16);

  assert(versionStr == "4.4.0");
  // on LC4500 eval module with fw 4.4, the first sector on flash memory
  // available for splash images. the corresponding address is also given in the
  // firmware's flash table which resides at offset 128kbyte
  const int splashImageSectorStart = 11;

  std::cout << "Entering programming mode...\n";
  ret = DLPC350_EnterProgrammingMode();
  if (ret < 0) {
    std::cerr << "ProjectorLC4500: could not enter programming mode.\n";
    DLPC350_ExitProgrammingMode();
    return;
  }

  // wait until usb connection is again established
  while (DLPC350_USB_Open() < 0) {
    QThread::msleep(200);
  }

  // ensure that this is the LC4500 evaluation module with
  // its standard memory chip
  unsigned short manId = 0;
  ret = DLPC350_GetFlashManID(&manId);
  if (ret < 0) {
    std::cerr << "ProjectorLC4500: could not get flash manufacturer id.\n";
    DLPC350_ExitProgrammingMode();
    return;
  }
  assert(manId == 0x0020);

  unsigned long long devId = 0;
  ret = DLPC350_GetFlashDevID(&devId);
  if (ret < 0) {
    std::cerr << "ProjectorLC4500: could not get flash device id.\n";
    DLPC350_ExitProgrammingMode();
    return;
  }
  assert(devId == 0x227e);

  DLPC350_SetFlashType(0x00);

  const int flashTableSize = 128 * 1024;
  unsigned char *flashTableFMW = (unsigned char *)malloc(flashTableSize);
  DLPC350_Frmw_UpdateFlashTableSplashAddress(
      flashTableFMW, splashImageSectorStart * 0x00020000);

  const int flashTableAddress = 128 * 1024;
  DLPC350_SetFlashAddr(flashTableAddress);
  DLPC350_FlashSectorErase();
  DLPC350_WaitForFlashReady();

  std::cout << "Writing flash table..." << std::endl;

  int bytesToWrite = flashTableSize;

  DLPC350_SetFlashAddr(flashTableAddress);
  DLPC350_SetUploadSize(flashTableSize);

  while (bytesToWrite > 0) {
    int bytesSent = DLPC350_UploadData(
        flashTableFMW + flashTableSize - bytesToWrite, bytesToWrite);

    if (bytesSent < 0) {
      std::cerr << "ProjectorLC4500: could not write flash table.\n";
      DLPC350_ExitProgrammingMode();
      return;
    }
    bytesToWrite -= bytesSent;
  }

  DLPC350_WaitForFlashReady();

  // construct the raw buffer of image and header data
  DLPC350_Frmw_SPLASH_InitBuffer(nPatterns);

  for (size_t p = 0; p < nPatterns; ++p) {

    // tile pattern and diamond downsample
    char patternTiled[PTN_HEIGHT][PTN_WIDTH][3];
    for (unsigned int j = 0; j < PTN_HEIGHT; j++) {
      const int jIdx = j % patternHeight;
      for (unsigned int i = 0; i < PTN_WIDTH; i++) {
        int iIdx = (i * 2 + j % 2) % patternWidth;
        patternTiled[j][i][0] =
            patterns[p][jIdx * patternWidth * 3 + iIdx * 3 + 0];
        patternTiled[j][i][1] =
            patterns[p][jIdx * patternWidth * 3 + iIdx * 3 + 1];
        patternTiled[j][i][2] =
            patterns[p][jIdx * patternWidth * 3 + iIdx * 3 + 2];
      }
    }

    cv::Mat mat(PTN_HEIGHT, PTN_WIDTH, CV_8UC3, patternTiled);
    std::vector<uchar> buf;
    cv::imencode(".bmp", mat, buf);
    //    cv::imwrite("patternTiled" + std::to_string(p) + ".bmp", mat);

    uint8 compression =
        SPLASH_NOCOMP_SPECIFIED; // will try line repetition and
                                 // run-length-encoding compression on the image
    uint32 compSize = 0;

    ret = DLPC350_Frmw_SPLASH_AddSplash(buf.data(), &compression, &compSize);
    if (ret < 0) {
      std::cerr << "ProjectorLC4500: could not create splash image buffer.\n";
      DLPC350_ExitProgrammingMode();
      return;
    }
  }

  uint32 splashBufferSize = 0;
  unsigned char *splashBuffer = nullptr;
  DLPC350_Frmw_Get_NewSplashBuffer(&splashBuffer, &splashBufferSize);

  // erase flash sectors
  const int splashImageSectorEnd =
      splashImageSectorStart + splashBufferSize / 0x00020000 + 1;

  for (int i = splashImageSectorStart; i < splashImageSectorEnd; ++i) {
    unsigned int sectorAddress =
        0x00000000 + i * 0x00020000; // see FlashDeviceParameters.txt
    std::cout << "ProjectorLC4500: erasing flash sector " << sectorAddress
              << std::endl;
    DLPC350_SetFlashAddr(sectorAddress);

    DLPC350_FlashSectorErase(); // warning: only erase if flash image area is
                                // correct on current fw
  }

  // upload new patterns
  DLPC350_SetFlashAddr(0x00000000 + splashImageSectorStart * 0x00020000);
  DLPC350_SetUploadSize(splashBufferSize);
  int bytesToUpload = splashBufferSize;

  while (bytesToUpload > 0) {
    int bytesUploaded = DLPC350_UploadData(
        splashBuffer + splashBufferSize - bytesToUpload, bytesToUpload);

    bytesToUpload -= bytesUploaded;

    std::cout << "uploading data: " << splashBufferSize - bytesToUpload << '/'
              << splashBufferSize << std::endl;

    if (bytesUploaded < 0) {
      std::cerr << "ProjectorLC4500: could not upload patterns.\n";
      DLPC350_ExitProgrammingMode();
      return;
    }
  }

  DLPC350_WaitForFlashReady();
  DLPC350_ExitProgrammingMode();

  // wait until usb connection is again established
  while (DLPC350_USB_Open() < 0) {
    QThread::msleep(100);
  }

  return;
}

void ProjectorLC4500::displayPattern(unsigned int i) {

  setToPatternMode();

  DLPC350_ClearPatLut();

  const int triggerTypeInternal = 0;
  const int patternEntry = 0;
  const int bitDepth = 8;
  const int ledWhite = 7;
  DLPC350_AddToPatLut(triggerTypeInternal, patternEntry, bitDepth, ledWhite,
                      false, true, true, false);

  const bool fromHDMI = false;
  DLPC350_SetPatternDisplayMode(fromHDMI);

  DLPC350_SetPatternConfig(1, true, 1, 1);

  // standard 60 Hz pattern exposure and frame period
  DLPC350_SetExposure_FramePeriod(1E6 / 60.0, 1E6 / 60.0);

  const int triggerModeInternal = 1;
  DLPC350_SetPatternTriggerMode(triggerModeInternal);

  DLPC350_SendPatLut();

  unsigned char splashLut = i;
  DLPC350_SendImageLut(&splashLut, 1);

  DLPC350_StartPatLutValidate();
  bool ready = false;
  unsigned int status;
  while (!ready) {
    QThread::msleep(50);
    DLPC350_CheckPatLutValidate(&ready, &status);
  }

  // play sequence
  DLPC350_PatternDisplay(2);
}

void ProjectorLC4500::displayBlack() {

  setToVideoMode();

  // test pattern solid field
  DLPC350_SetInputSource(1, 0);
  DLPC350_SetTPGSelect(0x0);

  DLPC350_SetLedEnables(false, false, false, false);
}

void ProjectorLC4500::displayWhite() {

  setToVideoMode();

  // test pattern solid field
  DLPC350_SetInputSource(1, 0);
  DLPC350_SetTPGSelect(0x0);

  DLPC350_SetLedEnables(false, true, true, true);
}

void ProjectorLC4500::getScreenRes(unsigned int *nx, unsigned int *ny) {
  *nx = PTN_WIDTH * 2;
  *ny = PTN_HEIGHT;
}

bool ProjectorLC4500::setToPatternMode() {

  // Check if it is in Video Mode
  bool mode;
  DLPC350_GetMode(&mode);
  if (mode == false) {

    // Switch to Pattern Mode
    DLPC350_SetMode(true);
    QThread::msleep(100);
    int i = 0;
    while (1) {
      DLPC350_GetMode(&mode);
      if (mode)
        break;
      QThread::msleep(100);
      if (i++ > 10)
        break;
    }
  }

  return true;
}

bool ProjectorLC4500::setToVideoMode() {

  // Check if it is in Pattern Mode
  bool mode;
  DLPC350_GetMode(&mode);
  if (mode == true) {
    // First stop pattern sequence
    unsigned int patMode;
    DLPC350_GetPatternDisplay(&patMode);
    // if it is in PAUSE or RUN mode
    if (patMode != 0) {
      int j = 0;
      unsigned int patMode;

      DLPC350_PatternDisplay(0);
      QThread::msleep(100);
      while (1) {
        DLPC350_GetPatternDisplay(&patMode);
        if (patMode == 0)
          break;
        else
          DLPC350_PatternDisplay(0);
        QThread::msleep(100);
        if (j++ > 10)
          break;
      }
    }

    // Switch to Video Mode
    DLPC350_SetMode(false);
    QThread::msleep(100);
    int i = 0;
    while (1) {
      DLPC350_GetMode(&mode);
      if (!mode)
        break;
      QThread::msleep(100);
      if (i++ > 10)
        break;
    }
  }

  return true;
}

ProjectorLC4500::~ProjectorLC4500() {

  // Stop pattern sequence
  //  DLPC350_PatternDisplay(0);

  DLPC350_USB_Close();
  //  DLPC350_USB_Exit();
}
