/*
 * dlpc350_firmware.cpp
 *
 * This module handles building and parsing of firmware images
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

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include "dlpc350_common.h"
#include "dlpc350_error.h"
#include "dlpc350_firmware.h"



#ifdef _WIN32
#include <windows.h>
#else
typedef unsigned short WORD;
typedef unsigned int LONG;
typedef unsigned int DWORD;
typedef struct tagBITMAPFILEHEADER {
    WORD bfType;
    DWORD bfSize;
    WORD bfReserved1;
    WORD bfReserved2;
    DWORD bfOffBits;
} BITMAPFILEHEADER, *PBITMAPFILEHEADER;
typedef struct tagBITMAPINFOHEADER {
    DWORD biSize;
    LONG  biWidth;
    LONG  biHeight;
    WORD  biPlanes;
    WORD  biBitCount;
    DWORD biCompression;
    DWORD biSizeImage;
    LONG  biXPelsPerMeter;
    LONG  biYPelsPerMeter;
    DWORD biClrUsed;
    DWORD biClrImportant;
} BITMAPINFOHEADER, *PBITMAPINFOHEADER;
#endif

#define BMP_FILE_HEADER_SIZE        14

/* Do not change the order of entries. It is used in inisavewindow.cpp to populate the save window with default and gui defined enteries */
INIPARAM_INFO g_iniParam_Info[] =
{
    {"APPCONFIG.VERSION.SUBMINOR", {0x00}, {0x00}, 1, 1, false, 0, 1},//SK: Remove
    {"APPCONFIG.VERSION.MINOR", {0x00}, {0x00}, 1, 1, false, 1, 1}, //SK: Remove
    {"APPCONFIG.VERSION.MAJOR", {0x03}, {0x00}, 1, 1, false, 2, 1}, //SK:Remove
    //{"APPCONFIG.VERSION.RSERVED", {0x00}, {0x00}, 1, 1, true, 3, 1},
    {"DEFAULT.FIRMWARE_TAG", {0x44, 0x4C, 0x50}, {0x00}, 3, 1, true, 4, 32},
    {"DEFAULT.AUTOSTART", {0x00}, {0x00}, 1, 1, false, 36, 1},
    {"DEFAULT.DISPMODE", {0x00}, {0x00}, 1, 1, true, 37, 1},
    {"DEFAULT.SHORT_FLIP", {0x00}, {0x00}, 1, 1, true, 38, 1},
    {"DEFAULT.LONG_FLIP", {0x00}, {0x00}, 1, 1, true, 39, 1},
    {"DEFAULT.TRIG_OUT_1.POL", {0x00}, {0x00}, 1, 1, true, 104, 1},
    {"DEFAULT.TRIG_OUT_1.RDELAY", {0xBB}, {0xBB}, 1, 1, true, 105, 1},
    {"DEFAULT.TRIG_OUT_1.FDELAY", {0xBB}, {0xBB}, 1, 1, true, 106, 1},
    {"DEFAULT.TRIG_OUT_2.POL", {0x00}, {0x00}, 1, 1, true, 108, 1},
    {"DEFAULT.TRIG_OUT_2.WIDTH", {0xBB}, {0xBB}, 1, 1, true, 109, 1},
    {"DEFAULT.TRIG_IN_1.DELAY", {0x00}, {0x00}, 1, 1, true, 112, 4},
    //{"DEFAULT.TRIG_IN_1.POL", {0x00}, {0x00}, 1, 1, false, 116, 1},
    //{"DEFAULT.TRIG_IN_2.DELAY", {0x00}, {0x00}, 1, 1, false, 120, 4},
    {"DEFAULT.TRIG_IN_2.POL", {0x00}, {0x00}, 1, 1, true, 124, 1},
    {"DEFAULT.RED_STROBE.RDELAY", {0xBB}, {0xBB}, 1, 1, true, 128, 1},
    {"DEFAULT.RED_STROBE.FDELAY", {0xBB}, {0xBB}, 1, 1, true, 129, 1},
    {"DEFAULT.GRN_STROBE.RDELAY", {0xBB}, {0xBB}, 1, 1, true, 132, 1},
    {"DEFAULT.GRN_STROBE.FDELAY", {0xBB}, {0xBB}, 1, 1, true, 133, 1},
    {"DEFAULT.BLU_STROBE.RDELAY", {0xBB}, {0xBB}, 1, 1, true, 136, 1},
    {"DEFAULT.BLU_STROBE.FDELAY", {0xBB}, {0xBB}, 1, 1, true, 137, 1},
    {"DEFAULT.INVERTDATA", {0x00}, {0x00}, 1, 1, true, 140, 1},
    {"DEFAULT.TESTPATTERN", {0x08}, {0x08}, 1, 1, true, 141, 1},
    {"DEFAULT.LEDCURRENT_RED", {0x97}, {0x97}, 1, 1, true, 149, 1},
    {"DEFAULT.LEDCURRENT_GRN", {0x78}, {0x78}, 1, 1, true, 150, 1},
    {"DEFAULT.LEDCURRENT_BLU", {0x7D}, {0x7D}, 1, 1, true, 151, 1},
    {"DEFAULT.PATTERNCONFIG.PAT_EXPOSURE", {0x7A120}, {0x7A120}, 1, 1, true, 156, 4},
    {"DEFAULT.PATTERNCONFIG.PAT_PERIOD", {0x7A120}, {0x7A120}, 1, 1, true, 160, 4},
    {"DEFAULT.PATTERNCONFIG.PAT_MODE", {0x03}, {0x03}, 1, 1, true, 164, 1},
    {"DEFAULT.PATTERNCONFIG.TRIG_MODE", {0x1}, {0x1}, 1, 1, true, 165, 1},
    {"DEFAULT.PATTERNCONFIG.PAT_REPEAT", {0x1}, {0x1}, 1, 1, true, 166, 1},
    {"DEFAULT.PATTERNCONFIG.NUM_LUT_ENTRIES", {0x02}, {0x1}, 1, 1, true, 168, 2},
    {"DEFAULT.PATTERNCONFIG.NUM_PATTERNS", {0x02}, {0x1}, 1, 1, true, 170, 2},
    {"DEFAULT.PATTERNCONFIG.NUM_SPLASH", {0x00}, {0x00}, 1, 1, true, 172, 2},
    {"DEFAULT.SPLASHLUT", {0x01}, {0x0}, 1, 1, true, 176, 256},
    {"DEFAULT.SEQPATLUT", {0x00061800, 0x00022804, 0x00024808}, {0x0}, 3, 1, true, 432, 29184},
    {"DEFAULT.LED_ENABLE_MAN_MODE", {0x0}, {0x0}, 1, 1, true, 29616, 1},
    {"DEFAULT.MAN_ENABLE_RED_LED", {0x0}, {0x0}, 1, 1, true, 29617, 1},
    {"DEFAULT.MAN_ENABLE_GRN_LED", {0x0}, {0x0}, 1, 1, true, 29618, 1},
    {"DEFAULT.MAN_ENABLE_BLU_LED", {0x0}, {0x0}, 1, 1, true, 29619, 1},
    {"DEFAULT.PORTCONFIG.PORT",{0x0}, {0x0},1, 1, true, 40, 1},
    {"DEFAULT.PORTCONFIG.BPP", {0x1}, {0x1}, 1, 1, true, 41, 1},
    {"DEFAULT.PORTCONFIG.PIX_FMT", {0x0}, {0x0}, 1, 1, true, 42, 1},
    {"DEFAULT.PORTCONFIG.PORT_CLK", {0x0}, {0x0}, 1, 1, true, 43, 1},
    //{"DEFAULT.PORTCONFIG.CSC[0]", {0x0400, 0x0000, 0x0000, 0x0000, 0x0400, 0x0000, 0x0000, 0x0000, 0x0400}, {0}, 9, 1, false, 44, 18},
    //{"DEFAULT.PORTCONFIG.CSC[1]", {0x04A8, 0xFDC7, 0xFF26, 0x04A8, 0x0715, 0x0000, 0x04A8, 0x0000, 0x0875}, {0}, 9, 1, false, 62, 18},
    //{"DEFAULT.PORTCONFIG.CSC[2]", {0x04A8, 0xFCC0, 0xFE6F, 0x04A8, 0x0662, 0x0000, 0x04A8, 0x0000, 0x0812}, {0}, 9, 1, false, 80, 18},
    {"DEFAULT.PORTCONFIG.ABC_MUX", {0x4}, {0x4}, 1, 1, true, 100, 1},
    {"DEFAULT.PORTCONFIG.PIX_MODE", {0x1}, {0x1}, 1, 1, true, 101, 1},
    {"DEFAULT.PORTCONFIG.SWAP_POL", {0x1}, {0x1}, 1, 1 ,true, 102, 1},
    {"DEFAULT.PORTCONFIG.FLD_SEL", {0x0}, {0x0}, 1, 1, true, 103, 1},
    {"PERIPHERALS.I2CADDRESS[0]", {0x34}, {0x34}, 1, 1, false, 29649, 1},
    {"PERIPHERALS.I2CADDRESS[1]", {0x3A}, {0x3A}, 1, 1, false, 29650, 1},
    {"PERIPHERALS.I2CBUSYGPIO_ENABLE", {0x00}, {0x00}, 1, 1, true, 29651, 1},
    {"PERIPHERALS.I2CBUSYGPIO_SELECT", {0x00}, {0x00}, 1, 1, true, 29652, 1},
    //{"PERIPHERALS.USB_SRL[0]", {0x004C, 0x0043, 0x0052, 0x0032}, {0x0}, 4, 1, false, 29656, 8},
    //{"PERIPHERALS.USB_SRL[1]", {0x004C, 0x0043, 0x0052, 0x0033}, {0x0}, 4, 1, false, 29664, 8},
    {"DATAPATH.SPLASHSTARTUPTIMEOUT", {0x1388}, {0x1388}, 1, 1, false, 29678, 2},
    {"DATAPATH.SPLASHATSTARTUPENABLE", {0x01}, {0x1}, 1, 1, true, 29682, 1},
    {"MACHINE_DATA.COLORPROFILE_0_BRILLIANTCOLORLOOK", {0x0}, {0x0}, 1, 1, true, 29752, 1},
};


uint32 FLASH_TABLE_ADDRESS = 0x00020000;

unsigned char *pFrmwImageArray;
unsigned char *splBuffer;
unsigned int splash_index;
unsigned int splash_data_start_flash_address;
unsigned int appl_config_data_flash_address;
int splash_count;

#define FLASH_THREE_ADDRESS					0xFB000000	// actually it is re map to 0xF8000000
#define FLASH_TWO_ADDRESS					0xFA000000
#define FLASH_BASE_ADDRESS					0xF9000000

//unsigned int ChipSelectSize[3] = {0x01000000, 0x01000000, 0x01000000};
unsigned int ChipSelectSize[3] = {0x00000000, 0x01000000, 0x01000000};  // LightCrafter 4500 does not have third chip
unsigned int ChipSelectEnd[3]  = {0xFC000000, 0xFA000000, 0xFB000000};
unsigned int ChipSelectBase[3] = {0xFB000000, 0xF9000000, 0xFA000000};

static int SPLASH_PerformLineCompression(unsigned char *SourceAddr, int ImageWidth, int ImageHeight, uint32 *compressed_size, uint8 numLines)
{
    uint16 Row, Col;
    uint32 lineLength, bytesPerPixel = 3, imageHeight, pixelIndex;
    unsigned char *line1Data, *line2Data;

    lineLength =  ImageWidth * bytesPerPixel;

    if(lineLength % 4 != 0)
    {
        lineLength = (lineLength / 4 + 1) * 4;
    }

    if(numLines > 8)
    {
        *compressed_size = 0xFFFFFFFF;
        return 1;
    }

    imageHeight = (ImageHeight / numLines) * numLines;

    for (Row = 0; Row < (imageHeight - numLines); Row++)
    {
        line1Data = (unsigned char *)(SourceAddr + (lineLength *  Row));
        line2Data = (unsigned char *)(SourceAddr + (lineLength * (Row + numLines)));

        for (Col = 0; Col < ImageWidth; Col++)
        {
            pixelIndex = Col * bytesPerPixel;

            if( (line1Data[pixelIndex  ] != line2Data[pixelIndex  ]) ||
                    (line1Data[pixelIndex+1] != line2Data[pixelIndex+1]) ||
                    (line1Data[pixelIndex+2] != line2Data[pixelIndex+2]))
            {
                *compressed_size = 0xFFFFFFFF;
                return 1;
            }
        }
    }

    *compressed_size = numLines * lineLength;
    return 0;
}

static int SPLASH_PerformRLECompression(unsigned char *SourceAddr, unsigned char *DestinationAddr, int ImageWidth, int ImageHeight, uint32 *compressed_size)
{
    uint16 Row, Col;
    BOOL   FirstPixel = TRUE;
    uint8 Repeat = 1;
    uint32 Pixel = 0, S, D;
    uint32 LastColor = 0, pad;
    uint8 count = 0;

    //    uint8 *SourceAddr		= (uint08 *)(SplashRLECfg->SourceAddr);
    //    uint8 *DestinationAddr = (uint08 *)(SplashRLECfg->DestinationAddr);
    uint32 PixelSize		= 3;
    
    S = 0;
    D = 0;

    /* RLE Encode the Splash Image*/
    for (Row = 0; Row < ImageHeight; Row++)
    {
        for (Col = 0; Col < ImageWidth; Col++)
        {

            memcpy(&Pixel, SourceAddr + S, PixelSize);

            S += PixelSize;

            /* if this is the first Pixel, remember it and move on... */
            if (FirstPixel)
            {
                LastColor  = Pixel;
                Repeat  = 1;
                FirstPixel = FALSE;
                count = 0;
            }
            else
            {
                if (Pixel == LastColor)
                {
                    if (count)
                    {
                        if(count > 1) DestinationAddr[D++] = 0;
                        DestinationAddr[D++] = count;
                        memcpy(DestinationAddr + D, SourceAddr + (S - ((count + 2) * PixelSize)) , count * PixelSize);
                        D += (count * PixelSize);
                        count = 0;
                    }

                    Repeat++;

                    if (Repeat == 255)
                    {
                        DestinationAddr[D++] = Repeat;
                        memcpy(DestinationAddr + D, &LastColor, PixelSize);
                        D += PixelSize;
                        FirstPixel = TRUE;
                    }
                }
                else
                {
                    if (Repeat == 1)
                    {
                        count++;
                        if (count == 255)
                        {
                            if(count > 1) DestinationAddr[D++] = 0;
                            DestinationAddr[D++] = count;
                            memcpy(DestinationAddr + D, SourceAddr + (S - ((count + 1) * PixelSize)), count * PixelSize);
                            D += (count * PixelSize);
                            count = 0;
                        }
                    }
                    else
                    {
                        DestinationAddr[D++] = Repeat;
                        memcpy(DestinationAddr + D, &LastColor, PixelSize);
                        D += PixelSize;
                        Repeat = 1;
                    }
                    LastColor = Pixel;
                }
            }

            /* Last Pixel of the line*/
            if (Col == (ImageWidth-1) && Repeat != 255 && count != 255)
            {
                if (count)
                {
                    DestinationAddr[D++] = 0;
                    DestinationAddr[D++] = count + 1;
                    memcpy(DestinationAddr + D, SourceAddr + (S - ((count + 1) * PixelSize)), (count + 1) * PixelSize);
                    D += ((count + 1) * PixelSize);
                    count = 0;
                }
                else
                {
                    DestinationAddr[D++] = Repeat;
                    memcpy(DestinationAddr + D, &LastColor, PixelSize);
                    D += PixelSize;
                }
                FirstPixel = TRUE;
            }
        }
        // END OF LINE
        DestinationAddr[D++] = 0;
        DestinationAddr[D++] = 0;

        /* Scan lines are always padded out to next 32-bit boundary */
        if(D % 4 != 0)
        {
            pad = 4 - (D % 4);
            memset(DestinationAddr + D, 0, pad);
            D += pad;
        }
    }

    /* End of file: Control Byte = 0 & Color Byte = 1 */
    DestinationAddr[D++] = 0;
    DestinationAddr[D++] = 1;

    /* End of file should be padded out till 128-bit boundary */
    if(D % 16 != 0)
    {
        pad = 16 - (D % 16);
        memset(DestinationAddr + D, 0, pad);
        D += pad;
    }

    // update flash size
    *compressed_size = D;

    return 0;
}

static int SPLASH_PerformRLEUnCompression(unsigned char *SourceAddr, unsigned char *DestinationAddr, uint32 *size)
{
    uint32 PixelSize= 3, S = 0, D = 0;
    int i;

    while (S < *size)
    {
        uint32 ctrl_byte, color_byte;

        ctrl_byte = SourceAddr[S];
        color_byte = SourceAddr[S + 1];
        if (ctrl_byte == 0)
        {
            if (color_byte == 1)	// End of image
                break;
            else if (color_byte == 0)	// End of Line.
            {
                i = 0;
                S +=2;
                if (S % 4 != 0)
                {
                    int pad = 4 - (S % 4);
                    S += pad;
                }
            }
            else if (color_byte >= 2)
            {
                S +=2;
                memcpy(DestinationAddr + D, SourceAddr + S, color_byte * PixelSize);
                D += color_byte * PixelSize;
                S += color_byte * PixelSize;
            }
            else
                return -1;
        }
        else if (ctrl_byte > 0)
        {
            S++;
            for (i = 0; (unsigned int)i < ctrl_byte; i++)
            {
                memcpy(DestinationAddr + D, SourceAddr + S, PixelSize);
                D += PixelSize;
            }
            S += PixelSize;
        }
        else
            return -1;
    }
    *size = D;
    return 0;
}

int DLPC350_Frmw_CopyAndVerifyImage(const unsigned char *pByteArray, int size)
{
    FLASH_TABLE *flash_table;

    if (pFrmwImageArray != NULL)
    {
        free(pFrmwImageArray);
        pFrmwImageArray = NULL;
        splash_data_start_flash_address = 0;
        appl_config_data_flash_address = 0;
    }


    pFrmwImageArray = (unsigned char *)malloc(size);
    if (pFrmwImageArray == NULL)
        return ERROR_NO_MEM_FOR_MALLOC;

    memcpy(pFrmwImageArray, pByteArray, size);

    flash_table = (FLASH_TABLE *)(pFrmwImageArray + FLASH_TABLE_ADDRESS);

    if(flash_table->Signature != FLASHTABLE_APP_SIGNATURE)
    {
        FLASH_TABLE_ADDRESS = 0x00008000;

        flash_table = (FLASH_TABLE *)(pFrmwImageArray + FLASH_TABLE_ADDRESS);

        if(flash_table->Signature != FLASHTABLE_APP_SIGNATURE)
        {
            FLASH_TABLE_ADDRESS = 0x00020000;

            flash_table = (FLASH_TABLE *)(pFrmwImageArray + FLASH_TABLE_ADDRESS);

            if(flash_table->Signature != FLASHTABLE_APP_SIGNATURE)
                return ERROR_FRMW_FLASH_TABLE_SIGN_MISMATCH;
        }
    }
    splash_data_start_flash_address = flash_table->Splash_Data[FLASH_TABLE_SPLASH_INDEX].Address;
    appl_config_data_flash_address = flash_table->APPL_Config_Data[0].Address;

    return 0;
}

unsigned int DLPC350_Frmw_GetVersionNumber()
{
    uint32 version_number;

    if (!appl_config_data_flash_address)
        return ERROR_INIT_NOT_DONE_PROPERLY;
    memcpy(&version_number, pFrmwImageArray + (appl_config_data_flash_address - FLASH_BASE_ADDRESS), sizeof(version_number));

    return version_number;
}

int DLPC350_Frmw_WriteApplConfigData(char *token, uint32 *params, int numParams)
{
    int i;
    int j;
    int index = -1;
    int offset;
    char toUpperToken[128];
    static int trigMode = -1;

    uint32 appl_config_data_start_address = appl_config_data_flash_address - FLASH_BASE_ADDRESS;
    uint8 *app_data = (uint8 *)(pFrmwImageArray + appl_config_data_start_address);

    if (!appl_config_data_flash_address)
        return ERROR_INIT_NOT_DONE_PROPERLY;

    //Convert everything to uppercase for string comparison
    i = 0;
    while(token[i] != '\0')
    {
        toUpperToken[i] = toupper(token[i]);
        i++;
    }
    toUpperToken[i] = '\0';

    for (index = 0; index < NR_INI_TOKENS; index++)
        if(strcmp(&g_iniParam_Info[index].token[0],&toUpperToken[0]) == 0)
            break;

    if (index == NR_INI_TOKENS) //if no match found in the above search
        return ERROR_WRONG_PARAMS;

    offset = g_iniParam_Info[index].frmw_offset;

    if(strcmp("DEFAULT.PORTCONFIG.CSC",&toUpperToken[0]) == 0)
    {
        if(numParams != 9)
            return ERROR_WRONG_PARAMS;

        for(i = 0; i < 9; i++)
        {
            *((uint16*) (app_data + offset)) = (uint16)params[i];
            offset += 2;
        }
    }
    else if (strcmp("DEFAULT.SPLASHLUT",&toUpperToken[0]) == 0)
    {
        //legacy trigger mode
        if(trigMode >= 0 && trigMode <= 2)
        {
            //Legacy pattern sequence maxsize = 64
            if((numParams < 1) || (numParams > MAX_IMAGE_LUT_ENTRIES))
                return ERROR_WRONG_PARAMS;
        }
        else
        {
            //Variable exposure trigger mode
            if(trigMode > 2 && trigMode <= 4)
            {
                //Variable exposure pattern sequence maxsize = 256
                if((numParams < 1) || (numParams > MAX_VAR_EXP_IMAGE_LUT_ENTRIES))
                    return ERROR_WRONG_PARAMS;
            }
            else
                return ERROR_WRONG_PARAMS;
        }

        //Special case for two entries
        if(numParams == 2)
        {
            *((uint8*) (app_data + offset)) = (uint8)params[1];
            offset ++;

            *((uint8*) (app_data + offset)) = (uint8)params[0];
            offset ++;
        }
        else
        {
            for (i = 0; i < numParams; i++)
            {
                *((uint8*) (app_data + offset)) = (uint8)params[i];
                offset ++;
            }
        }
    }
    else if (strcmp("DEFAULT.SEQPATLUT",&toUpperToken[0]) == 0)
    {

        //legacy trigger mode
        if(trigMode >= 0 && trigMode <= 2)
        {
            //Legacy pattern sequence maxsize = 128
            if((numParams < 1) || (numParams > MAX_PAT_LUT_ENTRIES))
                return ERROR_WRONG_PARAMS;

            //16-BYTES space available per entry
            for (i = 0; i < numParams; i++)
            {
                //BYTE - 3 Rsvd
                *((uint8*) (app_data + offset)) = (uint8)((params[i] >> 24) & 0xFF);
                offset ++;

                //BYTE - 0
                *((uint8*) (app_data + offset)) = (uint8)((params[i] >> 0) & 0xFF);
                offset ++;

                //BYTE - 1
                *((uint8*) (app_data + offset)) = (uint8)((params[i] >> 8) & 0xFF);
                offset ++;

                //BYTE - 2
                *((uint8*) (app_data + offset)) = (uint8)((params[i] >> 16) & 0xFF);
                offset ++;

                //fill remaining 12 - bytes with zeros
                for(j = 0; j < 12; j++)
                {
                    *((uint8*) (app_data + offset)) = (uint8) 0x00;
                    offset ++;
                }
            }
        }
        else
        {
            //Variable exposure trigger mode
            if(trigMode > 2 && trigMode <= 4)
            {
                //Variable exposure pattern sequence maxsize = 1824
                if((numParams < 1) || (numParams > (MAX_VAR_EXP_PAT_LUT_ENTRIES*3)))
                    return ERROR_WRONG_PARAMS;
            }
            else
                return ERROR_WRONG_PARAMS;

            //16-BYTES space available per entry - maximum can be 1824x3 32bit params
            for (i = 0; i < (numParams/3); i++)
            {
                //BYTE - 3 Rsvd
                *((uint8*) (app_data + offset)) = (uint8)((params[i*3] >> 24) & 0xFF);
                offset ++;

                //BYTE - 0
                *((uint8*) (app_data + offset)) = (uint8)((params[i*3] >> 0) & 0xFF);
                offset ++;

                //BYTE - 1
                *((uint8*) (app_data + offset)) = (uint8)((params[i*3] >> 8) & 0xFF);
                offset ++;

                //BYTE - 2
                *((uint8*) (app_data + offset)) = (uint8)((params[i*3] >> 16) & 0xFF);
                offset ++;

                //Fill Pattern exposure 32-bit information
                //LSB
                *((uint8*) (app_data + offset)) = (uint8)((params[i*3+1] >> 0) & 0xFF);
                offset ++;

                //LSB+1
                *((uint8*) (app_data + offset)) = (uint8)((params[i*3+1] >> 8) & 0xFF);
                offset ++;

                //LSB+2
                *((uint8*) (app_data + offset)) = (uint8)((params[i*3+1] >> 16) & 0xFF);
                offset ++;

                //LSB+3
                *((uint8*) (app_data + offset)) = (uint8)((params[i*3+1] >> 24) & 0xFF);
                offset ++;

                //Fill total Pattern period 32-bit information
                //LSB
                *((uint8*) (app_data + offset)) = (uint8)((params[i*3+2] >> 0) & 0xFF);
                offset ++;

                //LSB+1
                *((uint8*) (app_data + offset)) = (uint8)((params[i*3+2] >> 8) & 0xFF);
                offset ++;

                //LSB+2
                *((uint8*) (app_data + offset)) = (uint8)((params[i*3+2] >> 16) & 0xFF);
                offset ++;

                //LSB+3
                *((uint8*) (app_data + offset)) = (uint8)((params[i*3+2] >> 24) & 0xFF);
                offset ++;

                //fill remaining 4 - bytes with zeros
                for(j = 0; j < 4; j++)
                {
                    *((uint8*) (app_data + offset)) = (uint8) 0x00;
                    offset ++;
                }
           }
        }
    }
    else if (strcmp("PERIPHERALS.USB_SRL",&toUpperToken[0]) == 0)
    {
        if(numParams != 4)
            return ERROR_WRONG_PARAMS;

        for(int i = 0; i < 4; i++)
        {
            *((uint16*) (app_data + offset)) = (uint16)params[i];
            offset += 2;
        }
    }
    else if (strcmp("DEFAULT.FIRMWARE_TAG", &toUpperToken[0]) == 0)
    {
        if((numParams > 32))
            return ERROR_WRONG_PARAMS;

        for (i=0; i < numParams; i++)
        {
            *((uint8*) (app_data + offset)) = (uint8)(params[i] & 0xFF);
            offset ++;
        }

        //Append '\0' character if string length less than 32
        if(i < 32)
            *((uint8*) (app_data + offset)) = '\0';
    }
    else
    {
        //Store the trigger Mode
        if (strcmp("DEFAULT.PATTERNCONFIG.TRIG_MODE", &toUpperToken[0]) == 0)
        {
            trigMode = (uint8) params[0];
        }

        if (numParams > 1)
            return ERROR_WRONG_PARAMS;

        switch(g_iniParam_Info[index].frmw_size)
        {
        case 1:
            *((uint8 *)(app_data + offset)) = (uint8) params[0];
            break;
        case 2:
            *((uint16 *)(app_data + offset)) = (uint16) params[0];
            break;
        case 3:
            *((uint8 *)(app_data + offset + 0)) = (uint8) ((params[0] >> 0) & 0xFF);
            *((uint8 *)(app_data + offset + 1)) = (uint8) ((params[0] >> 8) & 0xFF);
            *((uint8 *)(app_data + offset + 2)) = (uint8) ((params[0] >> 16) & 0xFF);
            break;
        case 4:
            *((uint32 *)(app_data + offset)) = (uint32) params[0];
            break;
        default:
            return ERROR_WRONG_PARAMS;
        }
    }
    return 0;
}

int DLPC350_Frmw_GetSplashCount()
{
    uint32 splash_data_start_address = splash_data_start_flash_address - FLASH_BASE_ADDRESS;
    SPLASH_SUPER_BINARY_INFO binary_info;

    memcpy(&binary_info, pFrmwImageArray + splash_data_start_address, sizeof(binary_info));

    /* The GUI supports pattern image display for firmware images built with DLPC350_CONFIG.exe alone */
    if ((binary_info.Sig1 == 0x12345678) && (binary_info.Sig2 == 0x87654321))
        return binary_info.BlobCount;
    else
        return -1;
}

unsigned int DLPC350_Frmw_GetSPlashFlashStartAddress()
{
    return splash_data_start_flash_address;
}

int DLPC350_Frmw_GetSpashImage(unsigned char *pImageBuffer, int index)
{
    SPLASH_BLOB_INFO blob_info;
    uint32 blob_address, splash_image_address, splash_image_size;
    SPLASH_HEADER splash_header;
    unsigned char *image_buffer, *lineData;
    int i, j, lineLength;
    uint32 splash_data_start_address = splash_data_start_flash_address - FLASH_BASE_ADDRESS;

    blob_address = splash_data_start_address + sizeof(SPLASH_SUPER_BINARY_INFO) + index * sizeof(blob_info);
    memcpy(&blob_info, pFrmwImageArray + blob_address, sizeof(blob_info));

    if (blob_info.BlobOffset == 0xffffffff)
        return ERROR_NO_SPLASH_IMAGE;

    if(blob_info.BlobOffset < FLASH_BASE_ADDRESS)
    {
        blob_info.BlobOffset += 0x03000000;
    }

    splash_image_address = blob_info.BlobOffset - FLASH_BASE_ADDRESS;
    memcpy(&splash_header, pFrmwImageArray + splash_image_address, sizeof(splash_header));
    splash_image_address += sizeof(splash_header);

    splash_image_size = blob_info.BlobSize - sizeof(splash_header);

    if (splash_header.Compression == SPLASH_4LINE_COMPRESSION)
    {
        unsigned char *fourLine_buffer;

        fourLine_buffer = (unsigned char *)malloc(splash_image_size);
        if(fourLine_buffer == NULL)
            return ERROR_NO_MEM_FOR_MALLOC;

        memcpy(fourLine_buffer, pFrmwImageArray + splash_image_address, splash_image_size);

        image_buffer =  (unsigned char *)malloc(splash_header.Image_width * splash_header.Image_height * 3); // We only support 24 bit format.
        if(image_buffer == NULL)
            return ERROR_NO_MEM_FOR_MALLOC;

        for (i = 0; i < splash_header.Image_height; i+=4)
            memcpy((image_buffer + (splash_header.Image_width * 3 * i)), fourLine_buffer, splash_image_size);

        splash_image_size = splash_header.Image_width * splash_header.Image_height * 3;
    }
    else if (splash_header.Compression == SPLASH_RLE_COMPRESSION)
    {
        unsigned char* rle_buffer;

        rle_buffer = (unsigned char *)malloc(splash_image_size);
        if(rle_buffer == NULL)
            return ERROR_NO_MEM_FOR_MALLOC;

        image_buffer =  (unsigned char *)malloc(splash_header.Image_width * splash_header.Image_height * 3); // We only support 24 bit format.
        if(image_buffer == NULL)
            return ERROR_NO_MEM_FOR_MALLOC;

        memcpy(rle_buffer, pFrmwImageArray + splash_image_address, splash_image_size);

        SPLASH_PerformRLEUnCompression(rle_buffer, image_buffer, &splash_image_size);
        free(rle_buffer);
    }
    else if (splash_header.Compression == SPLASH_UNCOMPRESSED)
    {
        image_buffer = (unsigned char *)malloc(splash_image_size);
        if(image_buffer == NULL)
            return ERROR_NO_MEM_FOR_MALLOC;

        memcpy(image_buffer, pFrmwImageArray + splash_image_address, splash_image_size);
    }

    lineLength = splash_header.Image_width * 3;

    lineData = (unsigned char *) malloc(lineLength);
    for (i = 0; i < splash_header.Image_height; i++)
    {
        memcpy(lineData, image_buffer + (lineLength * i), lineLength);
        for (j = 0;j < splash_header.Image_width; j++)
        {
            unsigned char tempByte;

            tempByte = lineData[j * 3 + 2];
            lineData[j * 3 + 2] = lineData[j * 3 + 1];
            lineData[j * 3 + 1] = tempByte;
        }
        memcpy(image_buffer + (lineLength * i), lineData, lineLength);
    }

    free(lineData);
    memcpy(pImageBuffer, image_buffer, splash_image_size);
    free(image_buffer);

    return 0;
}

int DLPC350_Frmw_SPLASH_InitBuffer(int numSplash)
{
    SPLASH_SUPER_BINARY_INFO	binary_info;
    SPLASH_BLOB_INFO		blob_info;
    uint32 i;

    if(numSplash > MAX_SPLASH_IMAGES)
        return -1;

    if(splBuffer != NULL)
    {
        free(splBuffer);
        splBuffer = NULL;
    }
    splash_index = 0;
    splash_count = 0;

    binary_info.Sig1 = 0x12345678;
    binary_info.Sig2 = 0x87654321;
    binary_info.BlobCount = numSplash;

    blob_info.BlobOffset = 0xFFFFFFFF;
    blob_info.BlobSize   = 0xFFFFFFFF;

    //Allocate memory for splash images
    splBuffer = (unsigned char *) malloc(sizeof(binary_info) + (sizeof(blob_info) * MAX_SPLASH_IMAGES));

    memset(splBuffer,0x00,(sizeof(binary_info) + (sizeof(blob_info) * MAX_SPLASH_IMAGES)));

    memcpy(splBuffer + splash_index, &binary_info, sizeof(binary_info));
    splash_index += sizeof(binary_info);

    for(i = 0; i < (unsigned int)MAX_SPLASH_IMAGES; i++)
    {
        memcpy(splBuffer + splash_index, &blob_info, sizeof(blob_info));
        splash_index += sizeof(blob_info);
    }

    return 0;
}

int DLPC350_Frmw_SPLASH_AddSplash(unsigned char *pImageBuffer, uint8 *compression, uint32 *compSize)
{
    uint32 lineCompSize, rleCompSize;

    BITMAPINFOHEADER headerInfo;
    unsigned char *bitmapImage, *line1Data, *line2Data, *splashImage;
    int lineLength, bytesPerPixel, i, j;
    uint32 splashSize;
    SPLASH_HEADER splash_header;
    SPLASH_BLOB_INFO *blob_info;
    unsigned short bfType;
    unsigned int bfSize, bfOffBits;

    if((!splBuffer || !splash_data_start_flash_address))
        return ERROR_INIT_NOT_DONE_PROPERLY;

    memcpy(&bfType, pImageBuffer, sizeof(bfType));
    memcpy(&bfSize, pImageBuffer + sizeof(bfType), sizeof(bfSize));
    memcpy(&bfOffBits, pImageBuffer + 3*sizeof(bfType) + sizeof(bfSize), sizeof(bfOffBits));
    memcpy(&headerInfo, pImageBuffer + BMP_FILE_HEADER_SIZE, sizeof(headerInfo));

    if (bfType != 0x4D42)
        return ERROR_NOT_BMP_FILE;
    if(headerInfo.biBitCount != 24)// && headerInfo.biBitCount != 32)
    {
        return ERROR_NOT_24bit_BMP_FILE;
    }

    bitmapImage = (unsigned char *)malloc(bfSize - bfOffBits);
    if (!bitmapImage)
        return ERROR_NO_MEM_FOR_MALLOC;

    memcpy(bitmapImage, pImageBuffer + bfOffBits, bfSize - bfOffBits);

    bytesPerPixel = headerInfo.biBitCount / 8;

    lineLength    =  headerInfo.biWidth * bytesPerPixel;

    if(lineLength % 4 != 0)
    {
        lineLength = (lineLength / 4 + 1) * 4;
    }
    line1Data = (unsigned char *)malloc(lineLength);

    if(line1Data == NULL)
    {
        free(bitmapImage);
        return ERROR_NO_MEM_FOR_MALLOC;
    }

    line2Data = (unsigned char *)malloc(lineLength);

    if(line2Data == NULL)
    {
        free(line1Data);
        free(bitmapImage);
        return ERROR_NO_MEM_FOR_MALLOC;
    }

    // vertically flip the bitmap image
    for(i = 0; i < (headerInfo.biHeight / 2); i++)
    {
        memcpy(line1Data, bitmapImage + (lineLength * i), lineLength);
        memcpy(line2Data, bitmapImage + (lineLength * (headerInfo.biHeight - i - 1)), lineLength);

        unsigned char tempbyte;

        for(j = 0; j < headerInfo.biWidth; j++)
        {
            
            tempbyte = line1Data[j * 3 + 2];
            line1Data[j * 3 + 2] = line1Data[j * 3 + 1];
            line1Data[j * 3 + 1] = tempbyte;

            tempbyte = line2Data[j * 3 + 2];
            line2Data[j * 3 + 2] = line2Data[j * 3 + 1];
            line2Data[j * 3 + 1] = tempbyte;
        }

        memcpy(bitmapImage + (lineLength * (headerInfo.biHeight - i - 1)), line1Data, lineLength);
        memcpy(bitmapImage + (lineLength * i), line2Data, lineLength);
    }

    free(line1Data);
    free(line2Data);

    unsigned char *rleBuffer = (unsigned char *)malloc((((headerInfo.biHeight * headerInfo.biWidth * bytesPerPixel) +
                                                         ((headerInfo.biHeight * headerInfo.biWidth * 4) / 255) + (headerInfo.biWidth * 2) + 15) - 1));

    if (rleBuffer == NULL)
    {
        free(bitmapImage);
        return ERROR_NO_MEM_FOR_MALLOC;
    }

    switch(*compression)
    {
    case 0: // force uncompress
        splashSize  = headerInfo.biHeight * lineLength;
        splashImage = bitmapImage;
        break;

    case 1: // force rle compress
        SPLASH_PerformRLECompression(bitmapImage, rleBuffer, headerInfo.biWidth, headerInfo.biHeight, &splashSize);
        splashImage = rleBuffer;
        break;

    case 4: // force 4 line compress
        splashSize  = 4 * lineLength;
        splashImage = bitmapImage;
        break;

    default: // auto compression

        SPLASH_PerformLineCompression(bitmapImage, headerInfo.biWidth, headerInfo.biHeight, &lineCompSize, 4);
        SPLASH_PerformRLECompression(bitmapImage, rleBuffer, headerInfo.biWidth, headerInfo.biHeight, &rleCompSize);

        splashSize  = headerInfo.biHeight * lineLength;

        if(lineCompSize < splashSize)
        {
            splashSize  = 4 * lineLength;
            splashImage = bitmapImage;
            *compression = 4;
        }
        else if(rleCompSize < splashSize)
        {
            splashSize  = rleCompSize;
            splashImage = rleBuffer;
            *compression    = 1;
        }
        else
        {
            splashSize  = headerInfo.biHeight * lineLength;
            splashImage = bitmapImage;
            *compression    = 0;
        }

        break;
    }

    splash_header.Signature		= 0x636C7053;
    splash_header.Image_width	= (uint16)headerInfo.biWidth;
    splash_header.Image_height	= (uint16)headerInfo.biHeight;
    splash_header.Pixel_format	= 1; // 24-bit packed
    splash_header.Subimg_offset = -1;
    splash_header.Subimg_end	= -1;
    splash_header.Bg_color		= 0;
    splash_header.ByteOrder		= 1;
    splash_header.ChromaOrder	= 0;
    splash_header.Byte_count	= splashSize;
    splash_header.Compression	= *compression;

    blob_info = (SPLASH_BLOB_INFO *)(splBuffer + sizeof(SPLASH_SUPER_BINARY_INFO) + (splash_count * sizeof(SPLASH_BLOB_INFO)));

    uint32 FlashEnd, currChipSelect;
    int nextChipSelect;

    if(ChipSelectSize[0] != 0)
        FlashEnd = FLASH_THREE_ADDRESS + ChipSelectSize[0];
    else if(ChipSelectSize[2] != 0)
        FlashEnd = FLASH_TWO_ADDRESS   + ChipSelectSize[2];
    else if(ChipSelectSize[1] != 0)
        FlashEnd = FLASH_BASE_ADDRESS  + ChipSelectSize[1];

    ChipSelectEnd[0] = FLASH_THREE_ADDRESS + ChipSelectSize[0];
    ChipSelectEnd[1] = FLASH_BASE_ADDRESS  + ChipSelectSize[1];
    ChipSelectEnd[2] = FLASH_TWO_ADDRESS   + ChipSelectSize[2];

    if((splash_index + splash_data_start_flash_address + sizeof(splash_header) + splashSize) < FlashEnd)
    {
        nextChipSelect = -1;
        if((splash_index + splash_data_start_flash_address) < ChipSelectEnd[1] &&
                (splash_index + splash_data_start_flash_address + sizeof(splash_header) + splashSize) > ChipSelectEnd[1] &&
                (ChipSelectSize[1] != 0x01000000))
        {
            currChipSelect = 1;

            if(ChipSelectSize[2] != 0)
                nextChipSelect = 2;
            else if(ChipSelectSize[0] != 0)
                nextChipSelect = 0;
        }

        if((splash_index + splash_data_start_flash_address) < ChipSelectEnd[2] &&
                (splash_index + splash_data_start_flash_address + sizeof(splash_header) + splashSize) > ChipSelectEnd[2])
        {
            currChipSelect = 2;
            nextChipSelect = 0;
        }

        if(nextChipSelect != -1)
        {
            //printf("OVERFLOW FLASH_CS%d, MOVING SPLASH [%d] DATA TO FLASH_CS%d \n", currChipSelect, splash_count, nextChipSelect);
            //printf("BYTES UNUSED IN FLASH_CS%d = 0x%08X <%d> bytes\n", currChipSelect, ChipSelectEnd[currChipSelect] -
            //       splash_data_start_flash_address - splash_index, ChipSelectEnd[currChipSelect] - splash_data_start_flash_address - splash_index);

            splBuffer = (unsigned char *)realloc(splBuffer, ChipSelectBase[nextChipSelect] - splash_data_start_flash_address);

            memset(splBuffer + splash_index, 0xFF, ChipSelectBase[nextChipSelect] - splash_data_start_flash_address - splash_index);

            splash_index = ChipSelectBase[nextChipSelect] - splash_data_start_flash_address;

            blob_info = (SPLASH_BLOB_INFO *)(splBuffer + sizeof(SPLASH_SUPER_BINARY_INFO) + (splash_count * sizeof(SPLASH_BLOB_INFO)));
        }
        blob_info->BlobOffset = splash_index + splash_data_start_flash_address;
        blob_info->BlobSize   = sizeof(splash_header) + splashSize;

        /* check if it is crossing the 3rd chipselect, if yes remap it to 0th chip select */
        if(blob_info->BlobOffset >= FLASH_THREE_ADDRESS)
        {
            blob_info->BlobOffset -= 0x03000000;
        }

        splBuffer = (unsigned char *)realloc(splBuffer, splash_index + sizeof(splash_header) + splashSize);

        memcpy(splBuffer + splash_index, &splash_header, sizeof(splash_header));

        splash_index += sizeof(splash_header);

        memcpy(splBuffer + splash_index, splashImage, splashSize);

        splash_index += splashSize;

        splash_count++;
    }
    else
    {
        splash_count++;
        // printf("NO SPACE LEFT IN THE FLASH CAN'T WRITE SPLASH [%d]\n", splash_count);
        return ERROR_NO_SPACE_IN_FRMW;
    }

    free(rleBuffer);
    free(bitmapImage);
    *compSize = splashSize;
    return 0;
}

void DLPC350_Frmw_Get_NewFlashImage(unsigned char **newFrmwbuffer, uint32 *newFrmwsize)
{
    uint32 newfrmFileInLen = (splash_data_start_flash_address - FLASH_BASE_ADDRESS) + splash_index;

    pFrmwImageArray	= (unsigned char *)realloc(pFrmwImageArray, newfrmFileInLen);
    memcpy(pFrmwImageArray + (splash_data_start_flash_address - FLASH_BASE_ADDRESS), splBuffer, splash_index);

    *newFrmwbuffer = pFrmwImageArray;
    *newFrmwsize = newfrmFileInLen;
}

void DLPC350_Frmw_Get_NewSplashBuffer(unsigned char **newSplashBuffer, uint32 *newSplashSize)
{
    *newSplashBuffer = splBuffer;
    *newSplashSize = splash_index;
}

void DLPC350_Frmw_UpdateFlashTableSplashAddress(unsigned char *flashTableSectorBuffer, uint32 address_offset)
{
    FLASH_TABLE *flash_table;
    unsigned char *temp_flashTableSector = (unsigned char*) malloc(128 * 1024);

    splash_data_start_flash_address = address_offset + FLASH_BASE_ADDRESS;
    memcpy(temp_flashTableSector, pFrmwImageArray + FLASH_TABLE_ADDRESS, 128 * 1024);
    flash_table = (FLASH_TABLE *)(temp_flashTableSector);

    flash_table->Splash_Data[FLASH_TABLE_SPLASH_INDEX].Address = address_offset + FLASH_BASE_ADDRESS;
    memcpy(flashTableSectorBuffer, temp_flashTableSector, 128 * 1024);

    free(temp_flashTableSector);
}

char firstIniToken[128];
uint32 iniParams[MAX_VAR_EXP_PAT_LUT_ENTRIES*3];
uint32 numIniParams;

int DLPC350_Frmw_ParseIniLines(char *line)
{
    const char space[2] = " ";
    char *token;
    unsigned int tmpIntVar;
    bool isParamNameRcvd = false;

    numIniParams = 0;

    /* get the first token */
    token = strtok(&line[0], &space[0]);

    /* walk through other tokens */
    while( token != NULL )
    {

        //if end of data to be interpreted
        if(token[0] == ';')
            break;

        //check if it token is string
        if(sscanf(token,"%i",&tmpIntVar))
        {
            if(isParamNameRcvd)
            {
                iniParams[numIniParams] = tmpIntVar;
                numIniParams++;
            }
        }
        else
        {
            //Interpret the token only if it is a valid string
            if((*token == '\0') || (*token == '\t') || (*token == '\n') || (*token == ' '))
            {
               continue;
            }
            else
            {
                //Copy the param name
                strcpy(firstIniToken,token);

                isParamNameRcvd = true;

                //Remove spaces or tabs in the firstIniToken
                char *a = firstIniToken;

                while(1)
                {
                 if((*a == '\0') || (*a == '\t') || (*a == '\n') || (*a == ' '))
                     break;
                 else
                     a++;
                }

                *a = '\0';
            }
        }

        token = strtok(NULL, space);
    }

    //If there is no valid token in the line
    if(strcmp(firstIniToken," ") == 0)
        return -1;

#if 0
    unsigned int i;
    printf("Token = %s\n",str);
    printf("Number of params = %d\n",numIniParams);
    for(i=0;i<numIniParams;i++)
        printf("0x%06X\t",iniParams[i]);
#endif

    return 0;
}

//Below function is called after DLPC350_Frmw_ParseIniLines() function
void DLPC350_Frmw_GetCurrentIniLineParam(char *token, uint32 *params, int *numParams)
{
    unsigned int i = 0;

    //Point to the firstIniToken
    strcpy(token,&firstIniToken[0]);

    for (i = 0; i < numIniParams; i++)
        params[i] = iniParams[i];

    *numParams = numIniParams;

    return;
}

