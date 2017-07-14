/*
 * dlpc350_BMPParser.cpp
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

#include <time.h>
#include "dlpc350_common.h"
#include "dlpc350_error.h"

#include "dlpc350_BMPParser.h"

/******************************* MACROS **************************************/
#define BMP_FILE_HEADER_SIZE        14
#define BMP_DIB_HEADER_SIZE         40
#define BMP_DIB_HEADER_SIZE1        108

#define BMP_SIGNATURE       MAKE_WORD16('M', 'B')

#define BMP_DATA_SKIP(Skip, Data, Index)  (Index) += (Skip)

#define BMP_DATA_PARSE1(Value, Data, Index)  \
    do { (Value) = (Data)[Index]; (Index) += 1; } while(0)

#define BMP_DATA_PARSE2(Value, Data, Index)  \
    do { (Value) = PARSE_WORD16_LE(Data + Index); (Index)+=2; } while(0)

#define BMP_DATA_PARSE4(Value, Data, Index)  \
    do { (Value) = PARSE_WORD32_LE(Data + Index); (Index)+=4; } while(0)

#define BMP_ConvertEndianLE(Arr, Size)  (SUCCESS)

#define GET_LINE_BYTES(Image)	(ALIGN_BYTES_NEXT((Image)->Width * (Image)->BitDepth, 32)/8)

/**************************** LOCAL TYPES ************************************/
typedef struct
{
    uint32 Width;
    uint32 Height;
    uint16 BitDepth;
    uint32 Compression;
    uint32 PixelOffset;
    uint32 PaletteOffset;
    uint32 PaletteSize;
    uint32 FileSize;
} BMP_ImageHeader_t;


/********************* LOCAL FUNCTION PROTOTYPES *****************************/
static ErrorCode_t BMP_ParseHeader(BMP_ImageHeader_t *Image, uint8 *Data, uint32 DataSize);
/****************************** VARIABLES ************************************/

/************************ FUNCTION DEFINITIONS*******************************/

/**
*  This function parses the BMP image data received from GetData() function
*  and draws the pixels using DrawPixels() function
*
*  @param GetData - Function pointer for receiving BMP Data
*  @param DataParam - Parameter to be passed for GetData function
*  @param DrawPixels - Function pointer for drawing pixels
*  @param DrawParam - Parameter to be passed for DrawPixels function
*  @param InduxOnly - Only pass the palette index to the DrawPixels function
*					  If true the passed parameter one pixel per byte
*					  other wise it is one pixel per three bytes 
*					  (the order is RGB)
*  return SUCCESS, FAIL
*/
ErrorCode_t BMP_ParseImage(BMP_DataFunc_t *GetData, void *DataParam,
                           BMP_PixelFunc_t *DrawPixels, void *DrawParam,
                           uint8 OutBitDepth)
{
    ErrorCode_t Error = SUCCESS;
    uint8 Header[BMP_FILE_HEADER_SIZE + BMP_DIB_HEADER_SIZE];
    BMP_ImageHeader_t Image;
    uint32 ReadIndex;
    uint8 *LineData = NULL;
    uint8 *LineOutput = NULL;
    uint32 *Palette = NULL;
    uint32 LineWidth;
    uint32 OutLineWidth;
    int Shift;
    unsigned int PixelsPerByte;
    unsigned int x;
    unsigned int y;
    unsigned int i;
    unsigned int j;

    ERR_BLOCK_BEGIN
    {
        if(GetData(DataParam, Header, sizeof(Header)))
            ERR_THROW_MSG(Error = FAIL, "Error while reading header");

        ReadIndex = sizeof(Header);

        if(BMP_ParseHeader(&Image, Header, sizeof(Header)))
            ERR_THROW(Error = FAIL);

        DEBUG_MSG("Image FileSize = %d\n", Image.FileSize);
        DEBUG_MSG("Image Width = %d\n", Image.Width);
        DEBUG_MSG("Image Height = %d\n", Image.Height);
        DEBUG_MSG("Image BitDepth = %d\n", Image.BitDepth);
        DEBUG_MSG("Image PixelOffset = %d\n", Image.PixelOffset);
        DEBUG_MSG("Image PaletteOffset = %d\n", Image.PaletteOffset);

        if(GetData(DataParam, NULL, Image.PaletteOffset - ReadIndex))
            ERR_THROW_MSG(Error = FAIL, "Error while reading palette");

        ReadIndex = Image.PaletteOffset;

        if(Image.BitDepth != 1 && Image.BitDepth != 2 && Image.BitDepth != 4 &&
                Image.BitDepth != 8 && Image.BitDepth != 16 && Image.BitDepth != 24)
            ERR_THROW_MSG(Error = ERR_NOT_SUPPORTED, "Error un supported bit depth");

        if(Image.BitDepth <= 8)
        {
            if(Image.PixelOffset - ReadIndex < Image.PaletteSize * 4)
                ERR_THROW_MSG(Error = ERR_FORMAT_ERROR, "BMP File format error");
#if 1
            Palette = (uint32 *)malloc(Image.PaletteSize * 4);
            if(Palette == NULL)
                ERR_THROW_MSG(Error = ERR_OUT_OF_RESOURCE, "Unable to allocate memory for palette");

            if(GetData(DataParam, (uint8*)Palette, Image.PaletteSize * 4))
                ERR_THROW_MSG(Error = FAIL, "Error while reading palette");

            if(OutBitDepth == 16)
            {
                for(i = 0; i < Image.PaletteSize; i++)
                {
                    uint32 color = Palette[i];
                    Palette[i] = ((color & 0x0000F8) >> 3) |
                            ((color & 0x00FC00) >> 5) |
                            ((color & 0xF80000) >> 8) ;
                }
            }
            else if(OutBitDepth <= 8)
            {
                for(i = 0; i < Image.PaletteSize; i++)
                {
                    uint32 color = Palette[i];
                    Palette[i] = (GET_BYTE0(color) |
                                  GET_BYTE1(color) |
                                  GET_BYTE2(color) ) >> (8 - OutBitDepth);
                }
            }

#else
            int i;
            for(i = 0; i < Image.PaletteSize; i++)
            {
                uint32 PixVal;
                if(GetData(DataParam, (uint8*)&PixVal, 4))
                    ERR_THROW_MSG(Error = FAIL, "Error while reading palette");
                DEBUG_MSG("Palette %d: %08X\n", i, PixVal);
            }
#endif
            ReadIndex += Image.PaletteSize * 4;
        }

        if(Image.PixelOffset < ReadIndex)
            ERR_THROW_MSG(Error = ERR_FORMAT_ERROR, "BMP File format error");

        /* Ignore the Palette */
        if(GetData(DataParam, NULL, Image.PixelOffset - ReadIndex))
            ERR_THROW_MSG(Error = FAIL, "Error while reading palette");

        LineWidth = GET_LINE_BYTES(&Image);
        LineData = (uint8 *)malloc(LineWidth);
        if(LineData == NULL)
            ERR_THROW(Error = ERR_OUT_OF_RESOURCE);

        if(OutBitDepth >= 24)
            OutLineWidth = 3 * Image.Width;
        else if(OutBitDepth == 16)
            OutLineWidth = 2 * Image.Width;
        else
            OutLineWidth = Image.Width;

        LineOutput = (uint8 *)malloc(OutLineWidth);
        if(LineOutput == NULL)
            ERR_THROW(Error = ERR_OUT_OF_RESOURCE);

        if(Image.BitDepth <= 8)
        {
            Shift = 8 - Image.BitDepth;
            PixelsPerByte = 8 / Image.BitDepth;
            for(y = Image.Height; y-- > 0;)
            {
                if(GetData(DataParam, LineData, LineWidth))
                    ERR_THROW_MSG(Error = FAIL, "Error while reading pixels");

                if(OutBitDepth <= 8)
                {
                    for(x = 0, i = 0; i < LineWidth; i++)
                    {
                        uint8 PixData = LineData[i];
                        for(j = 0; j < PixelsPerByte && x < Image.Width; j++, x++)
                        {
                            LineOutput[x] = Palette[PixData >> Shift];
                            PixData <<= Image.BitDepth;
                        }
                    }
                }
                else if(OutBitDepth >= 24)
                {
                    for(x = 0, i = 0; i < LineWidth; i++)
                    {
                        uint8 PixData = LineData[i];
                        for(j = 0; j < PixelsPerByte && x < OutLineWidth; j++)
                        {
                            uint32 Color = Palette[PixData >> Shift];
                            LineOutput[x++] = GET_BYTE0(Color);
                            LineOutput[x++] = GET_BYTE1(Color);
                            LineOutput[x++] = GET_BYTE2(Color);
                            PixData <<= Image.BitDepth;
                        }
                    }
                }
                else if(OutBitDepth == 16)
                {
                    for(x = 0, i = 0; i < LineWidth; i++)
                    {
                        uint8 PixData = LineData[i];
                        for(j = 0; j < PixelsPerByte && x < OutLineWidth; j++)
                        {
                            uint32 Color = Palette[PixData >> Shift];
                            LineOutput[x++] = GET_BYTE0(Color);
                            LineOutput[x++] = GET_BYTE1(Color);
                            PixData <<= Image.BitDepth;
                        }
                    }
                }

                if(DrawPixels(DrawParam, 0, y, LineOutput, Image.Width))
                    ERR_THROW_MSG(Error = FAIL, "Error while drawing pixel");

            }
        }
        else if(Image.BitDepth == 24)
        {
            Shift = 8 - OutBitDepth;

            for(y = Image.Height; y-- > 0;)
            {
                if(GetData(DataParam, LineData, LineWidth))
                    ERR_THROW_MSG(Error = FAIL, "Error while reading pixel");

                if(OutBitDepth <= 8)
                {
                    unsigned int x1;
                    unsigned int x2;
                    for(x1 = 0, x2 = 0; x1 < Image.Width; x1++, x2+=3)
                    {
                        LineData[x1] = (LineData[x2] | LineData[x2 + 1] |
                                LineData[x2 + 2]) >> Shift;
                    }
                }
                else if(OutBitDepth == 16)
                {
                    unsigned int x1;
                    unsigned int x2;
                    for(x1 = 0, x2 = 0; x1 < Image.Width; x1++, x2+=3)
                    {
                        uint32 Color = LineData[x2] | (LineData[x2 + 1] << 8) | (LineData[x2 + 2] << 16);
                        ((uint16 *)LineData)[x1] = ((Color & 0xF8) >> 3) | ((Color & 0xFC00) >> 5) | ((Color & 0xF80000) >> 8);
                    }
                }

                if(DrawPixels(DrawParam, 0, y, LineData, Image.Width))
                    ERR_THROW_MSG(Error = FAIL, "Error while drawing pixel");
            }
        }
        else if(Image.BitDepth == 16)
        {
            uint8 *Ptr;

            if(OutBitDepth == 16)
                Ptr = LineData;
            else
                Ptr = LineOutput;

            for(y = Image.Height; y-- > 0;)
            {
                if(GetData(DataParam, LineData, LineWidth))
                    ERR_THROW_MSG(Error = FAIL, "Error while reading pixel");

                if(OutBitDepth <= 8)
                {
                    unsigned int x1;
                    for(x1 = 0; x1 < Image.Width; x1++)
                    {
                        uint16 color = ((uint16 *)LineData)[x1];
                        color = ((color << 3) & 0xF8) | ((color >> 3) & 0xFC) | ((color >> 8) & 0xF8);
                        Ptr[x1] = color & 0xFF;
                    }
                }
                else if(OutBitDepth >= 24)
                {
                    unsigned int x1;
                    unsigned int x2;

                    for(x1 = x2 = 0; x1 < Image.Width; x1++, x2 += 3)
                    {
                        uint16 color = ((uint16 *)LineData)[x1];

                        Ptr[x2 + 0] = (color << 3) & 0xF8;
                        Ptr[x2 + 1] = (color >> 3) & 0xFC;
                        Ptr[x2 + 2] = (color >> 8) & 0xF8;
                    }
                }

                if(DrawPixels(DrawParam, 0, y, Ptr, Image.Width))
                    ERR_THROW_MSG(Error = FAIL, "Error while drawing pixel");
            }
        }
    }
    ERR_BLOCK_END;

    free(Palette);
    free(LineData);
    free(LineOutput);

    return Error;
}

/**
* This function initializes the image structure for a new image
* @param Image - Image structure to be filled
* @param Width - Width of the image
* @param Height - Height of the image
* @param BitDepth - Bit depth of the image
*
*  return SUCCESS, FAIL
*/
ErrorCode_t BMP_InitImage(BMP_Image_t *Image, uint32 Width, uint32 Height, uint8 BitDepth)
{
    if(Image == NULL)
        return ERR_INVALID_PARAM;

    Image->NumColors = POW_OF_2(BitDepth);

    if(BitDepth <= 1)
        BitDepth = 1;
    else if(BitDepth <= 4)
        BitDepth = 4;
    else if(BitDepth <= 8)
        BitDepth = 8;
    else if(BitDepth == 16)
        BitDepth = 16;
    else if(BitDepth == 24)
        BitDepth = 24;
    else
        return ERR_INVALID_PARAM;

    Image->Width = Width;
    Image->Height = Height;
    Image->BitDepth = BitDepth;

    return SUCCESS;
}

static ErrorCode_t BMP_PutData(BMP_DataFunc_t *PutData, void *DataParam, uint32 Value, uint8 Size)
{
    uint8 Data[4];
    uint8 i;
    for(i = 0; i < Size; i++)
    {
        Data[i] = Value & 0xFF;
        Value >>= 8;
    }
    return PutData(DataParam, Data, Size);
}

/**
* This function returns the BMP file size fo the given image
* @param Image - Image structure
*
*  return Image file size
*/
uint32 BMP_ImageSize(BMP_Image_t *Image)
{
    uint32 PixOffset = Image->NumColors * 4 + BMP_FILE_HEADER_SIZE + BMP_DIB_HEADER_SIZE;
    uint32 DataSize = GET_LINE_BYTES(Image) * Image->Height;
    uint32 FileSize =  DataSize + PixOffset;

    return FileSize;
}

/**
*  This function stores the image formation received from GetPixels function and
*  stores it in BMP file format using PutData function
*
*  @param Image - Image structure
*  @param PutData - Function pointer for store the BMP file data
*  @param DataParam - Parameter to be passed for PutData function
*  @param GetPixels - Function pointer to get the image pixels
*  @param PixelParam - Parameter to be passed for GetPixels function
*
*  return SUCCESS, FAIL
*/
ErrorCode_t BMP_StoreImage(BMP_Image_t *Image, BMP_DataFunc_t *PutData, void *DataParam,
                           BMP_PixelFunc_t *GetPixels, void *PixelParam)
{
    ErrorCode_t Error = SUCCESS;
    int HeaderSize = Image->BitDepth == 16 ? BMP_DIB_HEADER_SIZE1 : BMP_DIB_HEADER_SIZE;
    uint32 PixOffset = Image->NumColors * 4 + BMP_FILE_HEADER_SIZE + HeaderSize;
    uint32 DataSize = GET_LINE_BYTES(Image) * Image->Height;
    uint32 FileSize =  DataSize + PixOffset;
    uint8 *Buffer = NULL;
    uint8 *LineOut = NULL;
    unsigned int x;
    unsigned int y;
    unsigned i;
    unsigned j;
    unsigned Mask;
    unsigned LineWidth;
    unsigned PixelsPerByte;

    ERR_BLOCK_BEGIN
    {
        if(BMP_PutData(PutData, DataParam, BMP_SIGNATURE, 2))
            ERR_THROW(Error = FAIL);
        if(BMP_PutData(PutData, DataParam, FileSize, 4))
            ERR_THROW(Error = FAIL);
        if(BMP_PutData(PutData, DataParam, 0, 4)) /* Reserved */
            ERR_THROW(Error = FAIL);
        if(BMP_PutData(PutData, DataParam, PixOffset, 4))
            ERR_THROW(Error = FAIL);
        if(BMP_PutData(PutData, DataParam, HeaderSize, 4))
            ERR_THROW(Error = FAIL);
        if(BMP_PutData(PutData, DataParam, Image->Width, 4))
            ERR_THROW(Error = FAIL);
        if(BMP_PutData(PutData, DataParam, Image->Height, 4))
            ERR_THROW(Error = FAIL);
        if(BMP_PutData(PutData, DataParam, 1, 2)) /* Number of color planes */
            ERR_THROW(Error = FAIL);
        if(BMP_PutData(PutData, DataParam, Image->BitDepth, 2))
            ERR_THROW(Error = FAIL);
        if(BMP_PutData(PutData, DataParam, Image->BitDepth == 16 ? 3 : 0, 4)) /* Compression = None */
            ERR_THROW(Error = FAIL);
        if(BMP_PutData(PutData, DataParam, DataSize, 4))
            ERR_THROW(Error = FAIL);
        if(BMP_PutData(PutData, DataParam, 2835, 4)) /* H Res pix/meter */
            ERR_THROW(Error = FAIL);
        if(BMP_PutData(PutData, DataParam, 2835, 4)) /* V Res pix/meter */
            ERR_THROW(Error = FAIL);
        if(BMP_PutData(PutData, DataParam, Image->NumColors, 4)) /* Palette Size */
            ERR_THROW(Error = FAIL);
        if(BMP_PutData(PutData, DataParam, 0, 4)) /* Important colors = All */
            ERR_THROW(Error = FAIL);

        for(i = 0; i < Image->NumColors; i++)
        {
            uint8 Val = (0xFF * i)/(Image->NumColors-1);
            uint32 Color = MAKE_WORD32(255, Val, Val, Val);

            if(BMP_PutData(PutData, DataParam, Color, 4))
                ERR_THROW(Error = FAIL);
        }

        if(Image->BitDepth == 16)
        {
            if(BMP_PutData(PutData, DataParam, 0xF800, 4))
                ERR_THROW(Error = FAIL);

            if(BMP_PutData(PutData, DataParam, 0x07E0, 4))
                ERR_THROW(Error = FAIL);

            if(BMP_PutData(PutData, DataParam, 0x001F, 4))
                ERR_THROW(Error = FAIL);

            if(BMP_PutData(PutData, DataParam, 0, 4))
                ERR_THROW(Error = FAIL);

            if(BMP_PutData(PutData, DataParam, 0x57696E20, 4))
                ERR_THROW(Error = FAIL);

            for(i = 0; i < 12; i++)
            {
                if(BMP_PutData(PutData, DataParam, 0, 4))
                    ERR_THROW(Error = FAIL);
            }
        }

        if(Image->BitDepth <= 8)
        {
            Buffer = (uint8 *)malloc(Image->Width);
            if(Buffer == NULL)
                ERR_THROW(Error = ERR_OUT_OF_RESOURCE);
        }

        Mask = GEN_BIT_MASK(0, Image->BitDepth);
        LineWidth = GET_LINE_BYTES(Image);
        LineOut = (uint8 *)malloc(LineWidth);
        if(LineOut == NULL)
            ERR_THROW(Error = ERR_OUT_OF_RESOURCE);


        PixelsPerByte = 8/Image->BitDepth;

        for(y = Image->Height; y-- > 0;)
        {
            if(Image->BitDepth <= 8)
            {
                if(GetPixels(PixelParam, 0, y, Buffer, Image->Width))
                    ERR_THROW(Error = FAIL);

                for(i = 0, x = 0; x < LineWidth; x++)
                {
                    uint8 Byte = 0;
                    for(j = 0; j < PixelsPerByte; j++)
                    {
                        Byte <<= Image->BitDepth;
                        if(i < Image->Width)
                        {
                            Byte |= Buffer[i++] & Mask;
                        }
                    }
                    LineOut[x] = Byte;
                }
                if(PutData(DataParam, LineOut, LineWidth))
                    ERR_THROW_MSG(Error = FAIL, "Error while drawing pixel");
            }
            else
            {
                if(GetPixels(PixelParam, 0, y, LineOut, Image->Width))
                    ERR_THROW(Error = FAIL);

                if(PutData(DataParam, LineOut, LineWidth))
                    ERR_THROW_MSG(Error = FAIL, "Error while drawing pixel");
            }
        }
    }
    ERR_BLOCK_END;

    free(LineOut);
    free(Buffer);

    return Error;
}


/**
*  This function parses the BMP header present in memroy
*
*  @param Image - The BMP Image header structure to be filled
*  @param Data - The pointer to the BMP header data
*  @param DataSize - The BMP Data size
*
*  return SUCCESS, FAIL
*/
static ErrorCode_t BMP_ParseHeader(BMP_ImageHeader_t *Image, uint8 *Data, uint32 DataSize)
{
    ErrorCode_t Error = SUCCESS;
    uint32 Value;
    uint32 Index = 0;

    ERR_BLOCK_BEGIN
    {
        if(Image == NULL || Data == NULL ||
                DataSize < BMP_FILE_HEADER_SIZE + BMP_DIB_HEADER_SIZE)
            ERR_THROW(Error = ERR_INVALID_PARAM);

        BMP_DATA_PARSE2(Value, Data, Index);
        if(Value  != BMP_SIGNATURE)
            ERR_THROW_MSG(Error = ERR_FORMAT_ERROR, "BMP File signature mismatch");

        BMP_DATA_PARSE4(Image->FileSize, Data, Index); /* File Size */

        BMP_DATA_SKIP(4, Data, Index);

        BMP_DATA_PARSE4(Image->PixelOffset, Data, Index); /* Offset to pixel data */
        if(Image->PixelOffset >= Image->FileSize)
            ERR_THROW_MSG(Error = ERR_FORMAT_ERROR, "Pixel start error");

        BMP_DATA_PARSE4(Value, Data, Index); /* DIB Header Size */
        if(Value < BMP_DIB_HEADER_SIZE)
            ERR_THROW_MSG(Error = ERR_FORMAT_ERROR, "DIB Header Size error");

        Image->PaletteOffset = Value + BMP_FILE_HEADER_SIZE;

        BMP_DATA_PARSE4(Image->Width, Data, Index);
        BMP_DATA_PARSE4(Image->Height, Data, Index);

        BMP_DATA_PARSE2(Value, Data, Index); /* Num Color planes */
        if(Value != 1)
            ERR_THROW_MSG(Error = ERR_FORMAT_ERROR, "Color planes error");

        BMP_DATA_PARSE2(Image->BitDepth, Data, Index);

        BMP_DATA_PARSE4(Image->Compression, Data, Index);

        if(Image->Compression != 0 && Image->Compression != 3)
            ERR_THROW_MSG(Error = ERR_NOT_SUPPORTED, "Image compression not supported");

        BMP_DATA_SKIP(12, Data, Index);

        BMP_DATA_PARSE4(Image->PaletteSize, Data, Index);

        if(Image->PaletteSize == 0)
            Image->PaletteSize = 1 << Image->BitDepth;

        (void)Index;
    }
    ERR_BLOCK_END;

    return Error;
}
