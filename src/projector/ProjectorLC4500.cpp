#include "ProjectorLC4500.h"

#include <QThread>
#include <iostream>

#include "LC4500API/dlpc350_api.h"
#include "LC4500API/dlpc350_usb.h"

void showError(std::string err){
    std::cerr << "lc4500startup: " << err.c_str() << std::endl;
}

ProjectorLC4500::ProjectorLC4500(unsigned int): nPatterns(0), isRunning(false){

    std::cout << "ProjectorLC4500: preparing LightCrafter 4500 for duty... " << std::endl;

    // Initialize usb connection
    if(DLPC350_USB_Init()){
        showError("Could not init USB!");
    }
    if(DLPC350_USB_Open()){
        showError("Could not connect!");
    }
    if(!DLPC350_USB_IsConnected()){
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
    if(isStandby){
        DLPC350_SetPowerMode(0);
        QThread::msleep(5000);
    }
    while(isStandby){
        QThread::msleep(50);
        DLPC350_GetPowerMode(&isStandby);
    }

//    // Set LED selection
//    const bool SeqCtrl  = false; // manual (false) or automatic (true)
//    const bool LEDRed  = false;
//    const bool LEDGreen  = false;
//    const bool LEDBlue  = true;
//    DLPC350_SetLedEnables(SeqCtrl, LEDRed, LEDGreen,  LEDBlue);

//    // Set LED currents
//    const unsigned char RedCurrent = 0;
//    const unsigned char GreenCurrent = 0;
//    const unsigned char BlueCurrent = 100;
//    DLPC350_SetLedCurrents(255-RedCurrent, 255-GreenCurrent, 255-BlueCurrent);

//    unsigned char Red;
//    unsigned char Green;
//    unsigned char Blue;
//    DLPC350_GetLedCurrents(&Red, &Green, &Blue);

    // Set to pattern sequence mode
    const bool patternSequenceMode = true;
    if(!DLPC350_SetMode(patternSequenceMode)){
        showError("Error Setting Pattern Sequence Mode");
    }
        
//    // Clear pattern LUT
//    DLPC350_ClearPatLut();
    
//    const int bitDepth = 8;
//    const int ledSelect = 1;
//    bool invertPattern = false;

//    // 2 x 8.333 exposures of the 2x3 psp pattern
//    //int DLPC350_AddToPatLut(int TrigType, int PatNum,int BitDepth,int LEDSelect,bool InvertPat, bool InsertBlack,bool BufSwap, bool trigOutPrev)
//    DLPC350_AddToPatLut(0, 1, bitDepth, ledSelect, invertPattern, false, true, false);
//    DLPC350_AddToPatLut(3, 1, bitDepth, ledSelect, invertPattern, true, false, true);
//    DLPC350_AddToPatLut(0, 0, bitDepth, ledSelect, invertPattern, false, false, false);
//    DLPC350_AddToPatLut(3, 0, bitDepth, ledSelect, invertPattern, true, false, true);
//    DLPC350_AddToPatLut(0, 2, bitDepth, ledSelect, invertPattern, false, false, false);
//    DLPC350_AddToPatLut(3, 2, bitDepth, ledSelect, invertPattern, true, false, true);

//    DLPC350_AddToPatLut(0, 1, bitDepth, ledSelect, invertPattern, false, true, false);
//    DLPC350_AddToPatLut(3, 1, bitDepth, ledSelect, invertPattern, true, false, true);
//    DLPC350_AddToPatLut(0, 0, bitDepth, ledSelect, invertPattern, false, false, false);
//    DLPC350_AddToPatLut(3, 0, bitDepth, ledSelect, invertPattern, true, false, true);
//    DLPC350_AddToPatLut(0, 2, bitDepth, ledSelect, invertPattern, false, false, false);
//    DLPC350_AddToPatLut(3, 2, bitDepth, ledSelect, invertPattern, true, false, true);

//    // Set to internal flash source
//    const bool patternDisplayMode = false;
//    DLPC350_SetPatternDisplayMode(patternDisplayMode);

//    DLPC350_SetPatternConfig(12, true, 12, 2);

//    DLPC350_SetExpsosure_FramePeriod(16666, 16666);

//    // Internal trigger
//    const bool patternTriggerMode = true;
//    DLPC350_SetPatternTriggerMode(patternTriggerMode);

//    if(DLPC350_SendPatLut() < 0)
//        showError("Error Sending Pattern LUT");

//    unsigned char splashLutEntries[] = {3, 4};
//    unsigned int numEntries = 2;
//    DLPC350_SendSplashLut(splashLutEntries, numEntries);

    // Validate pattern LUT
//    unsigned int status;
//    if(DLPC350_ValidatePatLutData(&status) < 0)
//        showError("Error validating LUT data");

    DLPC350_StartPatLutValidate();
    bool ready = false;
    unsigned int status;
    while(!ready){
        QThread::msleep(50);
        DLPC350_CheckPatLutValidate(&ready, &status);
    }

//    // Set trigger signal configuration
//    DLPC350_SetTrigOutConfig(1, true, 0, 0);

}

void ProjectorLC4500::setPattern(unsigned int patternNumber, const unsigned char *tex, unsigned int texWidth, unsigned int texHeight){

}

void ProjectorLC4500::displayPattern(unsigned int){

    if(!isRunning){
        // Start pattern sequence
        DLPC350_PatternDisplay(2);
        isRunning = true;
    }
}

void ProjectorLC4500::displayTexture(const unsigned char *tex, unsigned int texWidth, unsigned int texHeight){

}

void ProjectorLC4500::displayBlack(){

}

void ProjectorLC4500::displayWhite(){

}

void ProjectorLC4500::getScreenRes(unsigned int *nx, unsigned int *ny){
    *nx = 912;
    *ny = 1140;
}

ProjectorLC4500::~ProjectorLC4500(){

    // Stop pattern sequence
    DLPC350_PatternDisplay(0);

    DLPC350_USB_Close();
    DLPC350_USB_Exit();

}

