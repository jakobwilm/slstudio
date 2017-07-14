/*
 * dlpc350_BMPParser.h
 *
 * This module handles parsing BMP Image content
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

#ifndef BMPPARSER_H_
#define BMPPARSER_H_

#include "dlpc350_error.h"
#include "dlpc350_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32 Width;
    uint32 Height;
    uint16 NumColors;
    uint8 BitDepth;
} BMP_Image_t;

typedef ErrorCode_t (BMP_DataFunc_t)(void *Param, uint8 *Data, uint32 Size);
typedef ErrorCode_t (BMP_PixelFunc_t)(void *Param, uint32 X, uint32 Y, 
                                      uint8 *PixValue, uint32 Count);

ErrorCode_t BMP_ParseImage(BMP_DataFunc_t *GetData, void *DataParam,
                           BMP_PixelFunc_t *DrawPixels, void *DrawParam,
                           uint8 OutBitDepth);

ErrorCode_t BMP_InitImage(BMP_Image_t *Image, uint32 Width, uint32 Height, uint8 BitDepth);

ErrorCode_t BMP_StoreImage(BMP_Image_t *Image, BMP_DataFunc_t *PutData, void *DataParam,
                           BMP_PixelFunc_t *GetPixels, void *PixelParam);

uint32 BMP_ImageSize(BMP_Image_t *Image);



#ifdef __cplusplus
}
#endif

#endif /* BMPPARSER_H_ */
