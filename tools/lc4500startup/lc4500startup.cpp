#include <iostream>

#include "usb.h"
#include "API.h"

using namespace std;

void showError(std::string err){
    std::cerr << "lc4500startup: " << err << std::endl;
}

int main()
{
    cout << "lc4500startup: preparing LightCrafter 4500 for duty... " << flush;

    // Initialize usb connection
    if(USB_Init()){
        showError("Could not init USB!");
        return -1;
    }
    if(USB_Open()){
        showError("Could not connect!");
        return -1;
    }

    // Make sure LC is not in standby
    const bool standby = false;
    if(!LCR_SetPowerMode(standby)){
        showError("Error Setting Power Mode");
        return -1;
    }

    // Set LED selection
    const bool SeqCtrl  = false; // manual (false) or automatic (true)
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
        return -1;
    }

    // Set to external (HDMI) pattern input
    const bool external = true;
    if(!LCR_SetPatternDisplayMode(external)){
        showError("Error Setting Pattern Display Mode");
        return -1;
    }

    // Clear the Pattern Sequence LUT
    LCR_ClearPatLut();

    // Add two patterns to LUT
    const int TrigType = 1;
    const int PatNum = 1;
    const int BitDepth = 8;
    const int LEDSelectRed = 1;
    const bool InvertPat = false;
    const bool InsertBlack = false;
    const bool BufSwap = true;
    const bool trigOutPrev = false;
    if(LCR_AddToPatLut(TrigType, PatNum, BitDepth, LEDSelectRed, InvertPat, InsertBlack, BufSwap, trigOutPrev) == -1){
        showError("Error Adding Pattern LUT");
        return -1;
    }
    if(LCR_AddToPatLut(TrigType, PatNum, BitDepth, LEDSelectRed, InvertPat, InsertBlack, BufSwap, trigOutPrev) == -1){
        showError("Error Adding Pattern LUT");
        return -1;
    }

    unsigned char splashLut[64];
    int numSplashLutEntries = 0;
    const unsigned int frameIndexZero = 0;
    splashLut[numSplashLutEntries++] = frameIndexZero;

    // Configure pattern sequence mode
    const unsigned int numLutEntries = 2;
    const bool repeat = true;
    const unsigned int numPatsForTrigOut2 = 2;
    if(LCR_SetPatternConfig(numLutEntries, repeat, numPatsForTrigOut2, numSplashLutEntries) < 0){
        showError("Error Sending Pattern Config");
        return -1;
    }

    // Set exposure, frame period, etc.
    const unsigned int exposurePeriod = 8333; //us
    const unsigned int framePeriod = 8333; //us
    if(LCR_SetExpsosure_FramePeriod(exposurePeriod, framePeriod) < 0){
        showError("Error Sending Exposure period");
        return -1;
    }

    // Set the pattern trigger mode
    const bool vsyncPatternTriggerMode = false;
    if(LCR_SetPatternTriggerMode(vsyncPatternTriggerMode) < 0){
        showError("Error Sending trigger Mode");
        return -1;
    }

    // Send LUT
    if(LCR_SendPatLut() < 0){
        showError("Error Sending Pattern LUT");
        return -1;
    }

    // Send splash LUT
    if(LCR_SendSplashLut(splashLut, numSplashLutEntries) < 0){
        showError("Error Sending Splash LUT");
        return -1;
    }

    // Validate LUT data
    unsigned int status;
    if(LCR_ValidatePatLutData(&status) < 0){
        showError("Error validating LUT data");
        return -1;
    }

    // Start pattern sequence
    if(LCR_PatternDisplay(2) < 0){
        //Start pattern display
        showError("Error starting pattern display");
        return -1;
    }

    USB_Close();

    if(USB_Exit()){
        showError("Could not exit!");
        return -1;
    }

    cout << "done!" << endl;

    return 0;
}

