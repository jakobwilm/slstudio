#include <iostream>

#include "lcr_cmd.h"

using namespace std;

#define HandleResult(res,place) if (res!=SUCCESS) {printf("lc3000startup: Error at %s (%d)\n",place,res); fflush(stdout);}

int main(){

    cout << "lc3000startup: preparing LightCrafter 3000 for duty... " << flush;

    // Make connection
    int res = LCR_CMD_Open();
    HandleResult(res,"LCR_CMD_Open()")

    // Set power on (if in standby)
    res = LCR_CMD_SetPowerMode(PWR_NORMAL);
    HandleResult(res,"LCR_CMD_SetPowerMode()")

    // Set LED currents to max brightness
    LCR_LEDCurrent_t ledCurrents = {255,255,255};
    res = LCR_CMD_SetLEDCurrent(&ledCurrents);
    HandleResult(res,"LCR_CMD_SetLEDCurrent()")

    // Set display settings to no rotate, no long axis flix, short axis flip
    LCR_DisplaySetting_t displaySettings = {0,0,1};
    res = LCR_CMD_SetDisplaySetting(&displaySettings);
    HandleResult(res,"LCR_CMD_SetDisplaySetting()")

    // Set display mode to pattern sequence first (due to bug in LC firmware)
    LCR_CMD_SetDisplayMode(DISP_MODE_PTN_SEQ);

    // Set display mode to HDMI
    LCR_CMD_SetDisplayMode(DISP_MODE_VIDEO);

    // Set video settings
    LCR_VideoSetting_t videoSettings;
    videoSettings.ResolutionX = 608;
    videoSettings.ResolutionY = 684;
    videoSettings.FirstPix = 0;
    videoSettings.FirstLine = 0;
    videoSettings.ActiveWidth = 608;
    videoSettings.ActiveHeight = 684;
    res = LCR_CMD_SetVideoSetting(&videoSettings);
    HandleResult(res,"LCR_CMD_SetVideoSetting()")

    // Set video mode settings
    LCR_VideoModeSetting_t videoModeSettings;
    videoModeSettings.FrameRate = 60;
    videoModeSettings.BitDepth = 8;
    videoModeSettings.RGB = 2;
    res = LCR_CMD_SetVideoMode(&videoModeSettings);
    HandleResult(res,"LCR_CMD_SetVideoMode()")

    // Set trigger output settings
    LCR_CamTriggerSetting_t triggerSettings;
    triggerSettings.Enable = 1;
    triggerSettings.Source = 0;
    triggerSettings.Polarity = TRIGGER_EDGE_POS;
    triggerSettings.Delay = 0;
    triggerSettings.PulseWidth = 200; //us
    res = LCR_CMD_SetCamTriggerSetting(&triggerSettings);
    HandleResult(res,"LCR_CMD_SetCamTriggerSetting()")

    cout << "done!" << endl;

    return 0;
}

