#include "ProjectorLC4500.h"

#include "LC4500API/dlpc350_BMPParser.h"
#include "LC4500API/dlpc350_api.h"
#include "LC4500API/dlpc350_firmware.h"
#include "LC4500API/dlpc350_usb.h"
#include <QTest>
#include <QThread>
#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

void showError(std::string err) {
  std::cerr << "lc4500startup: " << err.c_str() << std::endl;
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

  // DLPC350_SetPatternConfig(6, true, 6, 2);

  //  // Internal trigger
  //  const bool patternTriggerMode = true;
  //  DLPC350_SetPatternTriggerMode(patternTriggerMode);

  //  DLPC350_StartPatLutValidate();
  //  bool ready = false;
  //  unsigned int status;
  //  while (!ready) {
  //    QThread::msleep(50);
  //    DLPC350_CheckPatLutValidate(&ready, &status);
  //  }
}

void ProjectorLC4500::setPattern(unsigned int patternNumber,
                                 const unsigned char *tex,
                                 unsigned int texWidth,
                                 unsigned int texHeight) {

  assert(texWidth == PTN_WIDTH);
  assert(texHeight == PTN_HEIGHT);

  std::copy(tex, tex + 912 * 1140, patterns[patternNumber].begin());
}

void ProjectorLC4500::displayPattern(unsigned int i) {

  assert(patterns.count(i) > 0);

  displayTexture(patterns[i].data(), 912, 1140);
}

void ProjectorLC4500::displayTexture(const unsigned char *tex,
                                     unsigned int texWidth,
                                     unsigned int texHeight) {

  assert(texWidth == PTN_WIDTH);
  assert(texHeight == PTN_HEIGHT);

  setToPatternMode();

  int ret = -1;

  // ensure version 4.4 of firmware such that the following flash addresses are
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

  DLPC350_Frmw_SPLASH_InitBuffer(1);

  std::cout << "Entering programming mode...\n";
  ret = DLPC350_EnterProgrammingMode();
  if (ret < 0) {
    std::cerr << "ProjectorLC4500: could not enter programming mode.\n";
    DLPC350_ExitProgrammingMode();
    return;
  }

  // wait until usb connection is again established
  while (DLPC350_USB_Open() < 0) {
    QTest::qWait(200);
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

  DLPC350_SetFlashType(0);

  // flash table as given in 4.4.0 stock firmware
  FLASH_TABLE flashTable;
  flashTable.Signature = 0x1234567;
  flashTable.Boot_Address = 0xf9000000;
  flashTable.Version = 0x13;
  flashTable.Free_Area_Start = 0x17c580;
  flashTable.AppCode[0] = {0xf9030000, 408152};
  flashTable.AppCode[1] = {0xffffffff, 0xffffffff};
  flashTable.AppCode[2] = {0xffffffff, 0xffffffff};
  flashTable.AppCode[3] = {0xffffffff, 0xffffffff};
  flashTable.ASIC_Config_Data[0] = {0xf909b180, 175760};
  flashTable.ASIC_Config_Data[1] = {0xf902a000, 3824};
  flashTable.ASIC_Config_Data[2] = {0xf90e0a40, 517512};
  flashTable.ASIC_Config_Data[3] = {0xffffffff, 0xffffffff};
  flashTable.Sequence[0] = {0xf90c6020, 104904};
  flashTable.Sequence[1] = {0xf90df9f0, 52};
  flashTable.Sequence[2] = {0xffffffff, 0xffffffff};
  flashTable.Sequence[3] = {0xffffffff, 0xffffffff};
  flashTable.APPL_Config_Data[0] = {0xf9093c00, 30064};
  flashTable.APPL_Config_Data[1] = {0xffffffff, 0xffffffff};
  flashTable.APPL_Config_Data[2] = {0xffffffff, 0xffffffff};
  flashTable.APPL_Config_Data[3] = {0xffffffff, 0xffffffff};
  flashTable.OSD_Data[0] = {0xffffffff, 0xffffffff};
  flashTable.OSD_Data[1] = {0xffffffff, 0xffffffff};
  flashTable.OSD_Data[2] = {0xffffffff, 0xffffffff};
  flashTable.OSD_Data[3] = {0xffffffff, 0xffffffff};
  flashTable.Splash_Data[0] = {0xf915efd0, 37520};
  flashTable.Splash_Data[1] = {0xf9168260, 39504};
  flashTable.Splash_Data[2] = {0xf9171cb0, 43216};
  flashTable.Splash_Data[3] = {0xffffffff, 0xffffffff};
  flashTable.APPL_OtherBinary[0] = {0xffffffff, 0xffffffff};
  flashTable.APPL_OtherBinary[1] = {0xffffffff, 0xffffffff};
  flashTable.APPL_OtherBinary[2] = {0xffffffff, 0xffffffff};
  flashTable.APPL_OtherBinary[3] = {0xffffffff, 0xffffffff};
  for (auto &sp2 : flashTable.Splash_Data2) {
    sp2 = {0xffffffff, 0xffffffff};
  }
  for (auto &sp2 : flashTable.Batch_File) {
    sp2 = {0xffffffff, 0xffffffff};
  }
  flashTable.Batch_File[0] = {0xf90dfa40, 1024};
  flashTable.Batch_File[1] = {0xf90dfe40, 1024};
  flashTable.Batch_File[2] = {0xf90e0240, 1024};
  flashTable.Batch_File[3] = {0xf90e0640, 1024};

  // adjust splash image address to sector boundary
  flashTable.Splash_Data[0].Address =
      FLASH_BASE_ADDRESS + splashImageSectorStart * 0x00020000;

  DLPC350_SetFlashAddr(128 * 1024);
  DLPC350_FlashSectorErase();
  DLPC350_WaitForFlashReady();

  std::cout << "Writing flash table..." << std::endl;
  int flashTableSize = 128 * 1024;
  int bytesToWrite = flashTableSize;

  DLPC350_SetFlashAddr(128 * 1024);
  DLPC350_SetUploadSize(flashTableSize);

  unsigned char *pFlashTable = reinterpret_cast<unsigned char *>(&flashTable);
  while (bytesToWrite > 0) {
    int bytesSent = DLPC350_UploadData(
        pFlashTable + flashTableSize - bytesToWrite, bytesToWrite);

    if (bytesSent < 0) {
      std::cerr << "ProjectorLC4500: could not write flash table.\n";
      DLPC350_ExitProgrammingMode();
      return;
    }
    bytesToWrite -= bytesSent;
  }

  DLPC350_WaitForFlashReady();

  // construct the raw buffer of image and header data
  DLPC350_Frmw_SPLASH_InitBuffer(1);

  //  unsigned char *pByteArray =
  //      new unsigned char[PTN_WIDTH * PTN_HEIGHT * BYTES_PER_PIXEL];
  //  auto writeDataFun = [](void *Param, uint8 *Data, uint32 Size) ->
  //  ErrorCode_t {
  //    for (uint32 i = 0; i < Size; ++i) {
  //      std::cout << "writing " << Data[i] << '\n';
  //      (*static_cast<uchar **>(Param))[i] = Data[i];
  //    }
  //    *static_cast<uchar **>(Param) += Size;
  //    return SUCCESS;
  //  };
  //  auto readDataFun = [](void *Param, uint32 X, uint32 Y, uint8 *PixValue,
  //                        uint32 Count) -> ErrorCode_t {
  //    for (uint32 i = 0; i < Count; ++i) {
  //      PixValue[i] = static_cast<const uchar *>(Param)[X + Y * PTN_WIDTH +
  //      i];
  //    }
  //    return SUCCESS;
  //  };

  //  // copy texture into bytearray in 3byte/pixel bmp representation
  //  BMP_Image_t fileInfo;
  //  BMP_InitImage(&fileInfo, PTN_WIDTH, PTN_HEIGHT, 8 * BYTES_PER_PIXEL);
  //  unsigned char *pWrite = pByteArray;
  //  unsigned char *pTex = const_cast<unsigned char *>(tex);
  //  BMP_StoreImage(&fileInfo, writeDataFun, &pWrite, readDataFun, pTex);

  std::vector<uchar> buf;
  cv::Mat mat(1140, 912, CV_8U, const_cast<unsigned char *>(tex));
  cv::cvtColor(mat, mat, cv::COLOR_GRAY2BGR);
  cv::imencode(".bmp", mat, buf);

  uint8 compression = -1; // will try line repetition or run-length-encoding
                          // compression on the image
  uint32 compSize;

  ret = DLPC350_Frmw_SPLASH_AddSplash(buf.data(), &compression, &compSize);
  if (ret < 0) {
    std::cerr << "ProjectorLC4500: could not create splash image buffer.\n";
    DLPC350_ExitProgrammingMode();
    return;
  }

  uint32 splashBufferSize = 0;
  unsigned char *splashBuffer = nullptr;
  DLPC350_Frmw_Get_NewSplashBuffer(&splashBuffer, &splashBufferSize);

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

  DLPC350_SetFlashAddr(0x00000000 + splashImageSectorStart * 0x00020000);
  DLPC350_SetUploadSize(splashBufferSize);
  int bytesToUpload = splashBufferSize;

  while (bytesToUpload > 0) {
    int bytesUploaded = DLPC350_UploadData(
        splashBuffer + splashBufferSize - bytesToUpload, bytesToUpload);

    bytesToUpload -= bytesUploaded;

    std::cout << splashBufferSize - bytesToUpload << '/' << splashBufferSize
              << std::endl;

    if (bytesUploaded < 0) {
      std::cerr << "ProjectorLC4500: could not upload patterns.\n";
      DLPC350_ExitProgrammingMode();
      return;
    }
  }

  DLPC350_WaitForFlashReady();
  DLPC350_ExitProgrammingMode();

  return;
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
  *nx = 912;
  *ny = 1140;
}

bool ProjectorLC4500::setToPatternMode() {

  // Check if it is in Video Mode
  bool mode;
  DLPC350_GetMode(&mode);
  if (mode == false) {

    // Switch to Pattern Mode
    DLPC350_SetMode(true);
    QTest::qSleep(100);
    int i = 0;
    while (1) {
      DLPC350_GetMode(&mode);
      if (mode)
        break;
      QTest::qSleep(100);
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
      QTest::qSleep(100);
      while (1) {
        DLPC350_GetPatternDisplay(&patMode);
        if (patMode == 0)
          break;
        else
          DLPC350_PatternDisplay(0);
        QTest::qSleep(100);
        if (j++ > 10)
          break;
      }
    }

    // Switch to Video Mode
    DLPC350_SetMode(false);
    QTest::qSleep(100);
    int i = 0;
    while (1) {
      DLPC350_GetMode(&mode);
      if (!mode)
        break;
      QTest::qSleep(100);
      if (i++ > 10)
        break;
    }
  }

  return true;
}

ProjectorLC4500::~ProjectorLC4500() {

  // Stop pattern sequence
  DLPC350_PatternDisplay(0);

  DLPC350_USB_Close();
  DLPC350_USB_Exit();
}
