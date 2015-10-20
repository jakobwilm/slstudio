#include "ProjectorLC4500.h"

#include <iostream>

#include "LC4500API/API.h"
#include "LC4500API/usb.h"

void showError(std::string err){
    std::cerr << "lc4500startup: " << err.c_str() << std::endl;
}


ProjectorLC4500::ProjectorLC4500(unsigned int): nPatterns(0), isRunning(false){

    std::cout << "ProjectorLC4500: preparing LightCrafter 4500 for duty... " << std::endl;

    // Initialize usb connection
    if(USB_Init()){
        showError("Could not init USB!");
    }
    if(USB_Open()){
        showError("Could not connect!");
    }
    if(!USB_IsConnected()){
        showError("Could not connect.");
    }
//    unsigned char HWStatus, SysStatus, MainStatus;
//    while(LCR_GetStatus(&HWStatus, &SysStatus, &MainStatus) != 0){
//        std::cout << ".";
//        continue;
//    }

    // Make sure LC is not in standby
    const bool standby = false;
    if(!LCR_SetPowerMode(standby)){
        showError("Error Setting Power Mode");
    }

    // Set LED selection
    const bool SeqCtrl  = true; // manual (false) or automatic (true)
    const bool LEDRed  = true;
    const bool LEDGreen  = false;
    const bool LEDBlue  = false;
    LCR_SetLedEnables(SeqCtrl, LEDRed, LEDGreen,  LEDBlue);

    // Set LED currents
    const unsigned char RedCurrent = 90;
    const unsigned char GreenCurrent = 0;
    const unsigned char BlueCurrent = 0;
    LCR_SetLedCurrents(255-RedCurrent, 255-GreenCurrent, 255-BlueCurrent);

    unsigned char Red;
    unsigned char Green;
    unsigned char Blue;
    LCR_GetLedCurrents(&Red, &Green, &Blue);

    // Set to pattern sequence mode
    const bool patternSequenceMode = true;
    if(!LCR_SetMode(patternSequenceMode)){
        showError("Error Setting Mode");
    }
        
    // Clear pattern LUT
    LCR_ClearPatLut();
    
    const int bitDepth = 8;
    const int ledSelect = 1;
    bool invertPattern = false;

    //int LCR_AddToPatLut(int TrigType, int PatNum,int BitDepth,int LEDSelect,bool InvertPat, bool InsertBlack,bool BufSwap, bool trigOutPrev)
    // 2 x 8.333 exposures of the 2x3 psp pattern
    LCR_AddToPatLut(0, 1, bitDepth, ledSelect, invertPattern, false, true, false);
    LCR_AddToPatLut(3, 1, bitDepth, ledSelect, invertPattern, true, false, true);
    LCR_AddToPatLut(0, 0, bitDepth, ledSelect, invertPattern, false, false, false);
    LCR_AddToPatLut(3, 0, bitDepth, ledSelect, invertPattern, true, false, true);
    LCR_AddToPatLut(0, 2, bitDepth, ledSelect, invertPattern, false, false, false);
    LCR_AddToPatLut(3, 2, bitDepth, ledSelect, invertPattern, true, false, true);

    LCR_AddToPatLut(0, 1, bitDepth, ledSelect, invertPattern, false, true, false);
    LCR_AddToPatLut(3, 1, bitDepth, ledSelect, invertPattern, true, false, true);
    LCR_AddToPatLut(0, 0, bitDepth, ledSelect, invertPattern, false, false, false);
    LCR_AddToPatLut(3, 0, bitDepth, ledSelect, invertPattern, true, false, true);
    LCR_AddToPatLut(0, 2, bitDepth, ledSelect, invertPattern, false, false, false);
    LCR_AddToPatLut(3, 2, bitDepth, ledSelect, invertPattern, true, false, true);

    // Set to internal flash source
    const bool patternDisplayMode = false;
    LCR_SetPatternDisplayMode(patternDisplayMode);

    LCR_SetPatternConfig(12, true, 12, 2);

    LCR_SetExpsosure_FramePeriod(16666, 16666);

    // Internal trigger
    const bool patternTriggerMode = true;
    LCR_SetPatternTriggerMode(patternTriggerMode);

    if(LCR_SendPatLut() < 0)
        showError("Error Sending Pattern LUT");

    unsigned char splashLutEntries[] = {3, 4};
    unsigned int numEntries = 2;
    LCR_SendSplashLut(splashLutEntries, numEntries);

    unsigned int status;
    if(LCR_ValidatePatLutData(&status) < 0)
        showError("Error validating LUT data");

    // Set trigger signal configuration
    LCR_SetTrigOutConfig(1, true, 0, 0);

}

void ProjectorLC4500::setPattern(unsigned int patternNumber, const unsigned char *tex, unsigned int texWidth, unsigned int texHeight){

}

void ProjectorLC4500::displayPattern(unsigned int){

    if(!isRunning){
        // Start pattern sequence
        LCR_PatternDisplay(2);
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
    LCR_PatternDisplay(0);

    USB_Close();
    USB_Exit();

}

