#include "ProjectorLC4500.h"

#include "LC4500API/API.h"
#include "LC4500API/usb.h"

void showError(std::string err){
    std::cerr << "lc4500startup: " << err << std::endl;
}


ProjectorLC4500::ProjectorLC4500(unsigned int): nPatterns(0){

    std::cout << "ProjectorLC4500: preparing LightCrafter 4500 for duty... " << std::endl;

    // Initialize usb connection
    if(USB_Init()){
        showError("Could not init USB!");
    }
    if(USB_Open()){
        showError("Could not connect!");
    }

    // Make sure LC is not in standby
    const bool standby = false;
    if(!LCR_SetPowerMode(standby)){
        showError("Error Setting Power Mode");
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
    }

    // Set to external (HDMI) pattern input
    const bool external = true;
    if(!LCR_SetPatternDisplayMode(external)){
        showError("Error Setting Pattern Display Mode");
    }


}

void ProjectorLC4500::setPattern(unsigned int patternNumber, const unsigned char *tex, unsigned int texWidth, unsigned int texHeight){

}

void ProjectorLC4500::displayPattern(unsigned int){

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

}

