/*
 * dlpc350_common.h
 *
 * This module provides the common defines used by all the modules.
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

#ifndef COMMON_H
#define COMMON_H

#include <stdlib.h>
#include <string.h>
#include <ctype.h>


#ifdef __cplusplus
extern "C" {
#endif


#define TRUE                    1
#define FALSE                   0

/* Bit masks. */
#define BIT0        0x01
#define BIT1        0x02
#define BIT2        0x04
#define BIT3        0x08
#define BIT4        0x10
#define BIT5        0x20
#define BIT6        0x40
#define BIT7        0x80
#define BIT8      0x0100
#define BIT9      0x0200
#define BIT10     0x0400
#define BIT11     0x0800
#define BIT12     0x1000
#define BIT13     0x2000
#define BIT14     0x4000
#define BIT15     0x8000
#define BIT16 0x00010000
#define BIT17 0x00020000
#define BIT18 0x00040000
#define BIT19 0x00080000
#define BIT20 0x00100000
#define BIT21 0x00200000
#define BIT22 0x00400000
#define BIT23 0x00800000
#define BIT24 0x01000000
#define BIT25 0x02000000
#define BIT26 0x04000000
#define BIT27 0x08000000
#define BIT28 0x10000000
#define BIT29 0x20000000
#define BIT30 0x40000000
#define BIT31 0x80000000


typedef int BOOL;
typedef unsigned uint32;
typedef unsigned char uint8;
typedef unsigned short uint16;

#ifndef MIN
#define MIN(a, b)					((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b)					((a) > (b) ? (a) : (b))
#endif

#define ALIGN_BYTES_PREV(x, b)      ((x) & ~(uint32)((b) - 1))
#define ALIGN_BYTES_NEXT(x, b)      (((x) + ((b)-1)) & ~(uint32)((b) - 1))

#define GET_BYTE0(x)				((x) & 0xFF)
#define GET_BYTE1(x)				(((x) >> 8) & 0xFF)
#define GET_BYTE2(x)				(((x) >> 16) & 0xFF)
#define GET_BYTE3(x)				(((x) >> 24) & 0xFF)

#define PARSE_WORD16_LE(Arr)        MAKE_WORD16((Arr)[1], (Arr)[0])
#define PARSE_WORD16_BE(Arr)        MAKE_WORD16((Arr)[0], (Arr)[1])
#define PARSE_WORD24_LE(Arr)        MAKE_WORD32(0, (Arr)[2], (Arr)[1], (Arr)[0])
#define PARSE_WORD24_BE(Arr)        MAKE_WORD32(0, (Arr)[0], (Arr)[1], (Arr)[2])
#define PARSE_WORD32_LE(Arr)        MAKE_WORD32((Arr)[3], (Arr)[2], (Arr)[1], (Arr)[0])
#define PARSE_WORD32_BE(Arr)        MAKE_WORD32((Arr)[0], (Arr)[1], (Arr)[2], (Arr)[3])

#define MAKE_WORD16(b1, b0)         (((b1) << 8) | (b0))
#define MAKE_WORD32(b3, b2, b1, b0) (((uint32)(b3)<<24)|((uint32)(b2)<<16)|((uint32)(b1)<<8)|(b0))

#define ARRAY_SIZE(x)               (sizeof(x)/sizeof(*x))
#define DIV_ROUND(x, y)             (((x)+(y)/2)/(y))
#define DIV_CEIL(x, y)              (((x)+(y)-1)/(y))
#define POW_OF_2(x)                 (1ul << (x))

#define IS_POW_OF_2(x)              (((x) & ((x)-1)) == 0)

// Generate bit mask of n bits starting from s bit
#define GEN_BIT_MASK(s, n)          (((1ul << (n)) - 1) << (s))

// Merge bits b into a according to mask
#define MERGE_BITS(a, b, mask)      ((a) ^ (((a) ^ (b)) & (mask)))

#define STRUCT_CLEAR(s)				(memset(&(s), 0, sizeof(s)))
#define BIT_REVERSE(x)				CMN_BitRevLUT[(uint8)(x)]

#define STR(x)	STR_(x)
#define STR_(x)	#x


#define HID_MESSAGE_MAX_SIZE        512

#define MAX_PATTERNS                24
#define PTN_WIDTH                   912
#define PTN_HEIGHT                  1140
#define BYTES_PER_PIXEL             3
#define MAX_PAT_LUT_ENTRIES         128
#define MAX_VAR_EXP_PAT_LUT_ENTRIES     1824
#define MAX_IMAGE_LUT_ENTRIES       64
#define MAX_VAR_EXP_IMAGE_LUT_ENTRIES   256

#define STAT_BIT_FLASH_BUSY         BIT3

typedef enum
{
    SUCCESS = 0,
    FAIL,
    ERR_OUT_OF_RESOURCE,
    ERR_INVALID_PARAM,
    ERR_NULL_PTR,
    ERR_NOT_INITIALIZED,
    ERR_DEVICE_FAIL,
    ERR_DEVICE_BUSY,
    ERR_FORMAT_ERROR,
    ERR_TIMEOUT,
    ERR_NOT_SUPPORTED,
    ERR_NOT_FOUND
} ErrorCode_t;

typedef enum
{
    IMAGE_PIX_FORMAT_RGB32,
    IMAGE_PIX_FORMAT_GREY8,
    IMAGE_PIX_FORMAT_GREY10,
    IMAGE_PIX_FORMAT_UYVY16,
    IMAGE_PIX_FORMAT_RGB16,
    IMAGE_PIX_FORMAT_SBGGR,
    IMAGE_PIX_FORMAT_RGB24,
} ImagePixFormat_t;

typedef struct
{
    unsigned char *Buffer;
    unsigned Width;
    unsigned Height;
    unsigned LineWidth;
    ImagePixFormat_t PixFormat;
}Image_t;

uint32 GetImagePixel(Image_t *Image, unsigned int x, unsigned int y);

uint32 Next2Power(uint32 Value);
unsigned Hex2BinArray(char *HexStr, unsigned Size, uint8 *BinArray);
int TrimString(char const *Input, char *Output);
ErrorCode_t WriteTextToFile(char const *File, int Num, char const *Text);
ErrorCode_t  ReadTextFromFile(char const *File, int Num, char *Text, int Len);
BOOL FileExist(char const *File, int Num);


#ifdef __cplusplus
}
#endif


#endif /* COMMON_H_ */
