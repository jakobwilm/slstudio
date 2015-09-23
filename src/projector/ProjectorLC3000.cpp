#include "ProjectorLC3000.h"

// Additions to the LC3000 API
#include "LC3000API/lcr_packetizer.h"
ErrorCode_t LCR_CMD_DisplayPattern(LCR_PatternCount_t PtnNumber){
    /* Generate packet */
    LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0405);
    LCR_CMD_PKT_PutInt(PtnNumber, 2);
    if(LCR_CMD_PKT_SendCommand())
        return FAIL;

    return SUCCESS;
}

#include "LC3000API/bitmap_image.hpp"

#define HandleResult(res,place) if (res!=SUCCESS) {printf("ProjectorLC3000: Error at %s (%d)\n",place,res); fflush(stdout);}

ProjectorLC3000::ProjectorLC3000(unsigned int){

    // Make connection
    res = LCR_CMD_Open();
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

    // Set trigger output settings
    LCR_CamTriggerSetting_t triggerSettings;
    triggerSettings.Enable = 1;
    triggerSettings.Source = 0;
    triggerSettings.Polarity = TRIGGER_EDGE_POS;
    triggerSettings.Delay = 0;
    triggerSettings.PulseWidth = 200; //us
    res = LCR_CMD_SetCamTriggerSetting(&triggerSettings);
    HandleResult(res,"LCR_CMD_SetCamTriggerSetting()")

    // Set default pattern sequence settings
    patternSeqSettings.BitDepth = 8;
    patternSeqSettings.NumPatterns = 1;
    patternSeqSettings.PatternType = PTN_TYPE_NORMAL;
    patternSeqSettings.InputTriggerType = TRIGGER_TYPE_SW;
    patternSeqSettings.InputTriggerDelay = 0;
    patternSeqSettings.AutoTriggerPeriod = 0;
    patternSeqSettings.ExposureTime = 16666; //us
    patternSeqSettings.LEDSelect = LED_RED;
    patternSeqSettings.Repeat = 1;
    res = LCR_CMD_SetPatternSeqSetting(&patternSeqSettings);
    HandleResult(res,"LCR_CMD_SetPatternSeqSetting()")

    //project white
    this->displayWhite();
}

void ProjectorLC3000::setPattern(unsigned int patternNumber, const unsigned char *tex, unsigned int texWidth, unsigned int texHeight){

    // Set pattern count
    unsigned int nPatterns = std::max((unsigned int)patternSeqSettings.NumPatterns, patternNumber+1);
    patternSeqSettings.NumPatterns = nPatterns;
    res = LCR_CMD_SetPatternSeqSetting(&patternSeqSettings);
    HandleResult(res,"LCR_CMD_SetPatternSeqSetting()")

    // Tile texture
    char texTiled[608][684][3];
    for(unsigned int j=0; j<608; j++){
        int jIdx = j%texWidth;
        for(unsigned int i=0; i<684; i++){
            int iIdx = i%texHeight;
            texTiled[j][i][0] = tex[iIdx*texWidth*3 + jIdx*3 + 0];
            texTiled[j][i][1] = tex[iIdx*texWidth*3 + jIdx*3 + 1];
            texTiled[j][i][2] = tex[iIdx*texWidth*3 + jIdx*3 + 2];
        }
    }
    // Bitmap version of texture
    bitmap_image img(608, 684);
    for(unsigned int j=0; j<608; j++)
        for(unsigned int i=0; i<684; i++)
            img.set_pixel(j, i, texTiled[j][i][0], texTiled[j][i][1], texTiled[j][i][2]);

    // Save
    img.save_image("tex.bmp");

    // Set as pattern
    res = LCR_CMD_DefinePatternBMP(patternNumber, "tex.bmp");
    HandleResult(res,"LCR_CMD_DefinePattern()")
}

void ProjectorLC3000::displayPattern(unsigned int patternNumber){

    // Set display mode
    if ( this->ptn_seq_mode == false ) {
        res = LCR_CMD_SetDisplayMode(DISP_MODE_PTN_SEQ);
        HandleResult(res,"LCR_CMD_SetDisplayMode()")
        this->ptn_seq_mode = true;
    }

    res = LCR_CMD_DisplayPattern(patternNumber);
    HandleResult(res,"LCR_CMD_DisplayPattern()")
}

void ProjectorLC3000::displayTexture(const unsigned char *tex, unsigned int texWidth, unsigned int texHeight){
//    // Set display to image/static color
//    res = LCR_CMD_SetDisplayMode(DISP_MODE_IMAGE);
//    HandleResult(res,"LCR_CMD_SetDisplayMode()")

//    // Tile texture
//    char texTiled[608][684][3];
//    for(unsigned int j=0; j<608; j++){
//        int jIdx = j%texWidth;
//        for(unsigned int i=0; i<684; i++){
//            int iIdx = i%texHeight;
//            texTiled[j][i][0] = tex[iIdx*texWidth*3 + jIdx*3 + 0];
//            texTiled[j][i][1] = tex[iIdx*texWidth*3 + jIdx*3 + 1];
//            texTiled[j][i][2] = tex[iIdx*texWidth*3 + jIdx*3 + 2];
//        }
//    }
//    // Bitmap version of texture
//    bitmap_image img(608, 684);
//    for(unsigned int j=0; j<608; j++)
//        for(unsigned int i=0; i<684; i++)
//            img.set_pixel(j, i, texTiled[j][i][0], texTiled[j][i][1], texTiled[j][i][2]);

//    // Save
//    img.save_image("tex.bmp");

//    // Display
//    res = LCR_CMD_DisplayStaticImage("tex.bmp");
//    HandleResult(res,"LCR_CMD_DisplayStaticImage()")

    patternSeqSettings.NumPatterns = 1;
    this->setPattern(0, tex, texWidth, texHeight);
    this->displayPattern(0);

}

void ProjectorLC3000::displayBlack(){
    // Set display to image/static color
    res = LCR_CMD_SetDisplayMode(DISP_MODE_IMAGE);
    HandleResult(res,"LCR_CMD_SetDisplayMode()")

    res = LCR_CMD_DisplayStaticColor(0);
    HandleResult(res,"LCR_CMD_DisplayStaticColor()")
}

void ProjectorLC3000::displayWhite(){
    // Set display to image/static color
    res = LCR_CMD_SetDisplayMode(DISP_MODE_IMAGE);
    HandleResult(res,"LCR_CMD_SetDisplayMode()")

    res = LCR_CMD_DisplayStaticColor((255 << 16) |  (255 << 8) | (255));
    HandleResult(res,"LCR_CMD_DisplayStaticColor()")
    this->ptn_seq_mode=false;
}

void ProjectorLC3000::getScreenRes(unsigned int *nx, unsigned int *ny){
    *nx = 608;
    *ny = 684;
}

ProjectorLC3000::~ProjectorLC3000(){
    res = LCR_CMD_Close();
    HandleResult(res,"LCR_CMD_Close()")
}

