/*****************************************************************************
**             TEXAS INSTRUMENTS PROPRIETARY INFORMATION
**
**  (c) Copyright, Texas Instruments Incorporated, 2012-2013.
**      All Rights Reserved.
**
**  Property of Texas Instruments Incorporated. Restricted Rights -
**  Use, duplication, or disclosure is subject to restrictions set
**  forth in TI's program license agreement and associated documentation.
******************************************************************************/
/**
*
* @file    lcr_cmd.c
*
* @brief	This file contatins DM365 Command Interfaces implementation.
*			As per the command definition the the command packets will be
*			created.
**/
/*****************************************************************************/


#include "common.h"
#include "error.h"
#include "lcr_packetizer.h"
#include "lcr_cmd.h"

static ErrorCode_t readCommand(uint16 command, uint8 *data, unsigned int size)
{
	if(LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_READ, command) ||
		LCR_CMD_PKT_SendCommand() || LCR_CMD_PKT_GetData(data, size))
		return FAIL;
	return SUCCESS;
}

static uint32 readInteger(const uint8 *data, unsigned int size)
{
	uint32 value = 0;
	for(unsigned int index = 0; index < size; ++index)
		value |= static_cast<uint32>(data[index]) << (index * 8);
	return value;
}

/* Connects to the DLP LightCrafter(TM) Hardware */
ErrorCode_t LCR_CMD_Open(void)
{
	/* Open tcp connection with LCr */
	return LCR_CMD_PKT_ConnectToLCR();
}

/*Close the TCP Connection*/
ErrorCode_t LCR_CMD_Close(void)
{
	LCR_CMD_PKT_DisconnectLCR();

	return SUCCESS;
}

/* Change the LightCrafter IP address */
ErrorCode_t LCR_CMD_SetIPAddress(uint32 Address)
{
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0800);
	/* Address bytes are sent least-significant first; byte 0 is the first IPv4 octet. */
	if(LCR_CMD_PKT_PutInt(Address, 4))
		return FAIL;
	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;
}

/* Read revision of MSP430, DM365 and FPGA */
ErrorCode_t LCR_CMD_GetRevision(LCR_Revision_t Which, char *VersionStr)
{
	//Frame the packet
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_READ, 0x0100);
	if(Which != REV_DM365 && Which != REV_FPGA && Which != REV_MSP430)
	{
		return FAIL;
	}
	LCR_CMD_PKT_PutInt(Which,1);
	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	if(LCR_CMD_PKT_GetData((uint8*)VersionStr,LCR_CMD_VERSION_STR_LEN))
		return FAIL;

	return SUCCESS;
}

/* Change the Display Mode */
ErrorCode_t LCR_CMD_SetDisplayMode(LCR_DisplayMode_t Mode)
{

	if(Mode > DISP_MODE_PTN_SEQ)
		return FAIL;

	/* Generate packet */
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0101);
	LCR_CMD_PKT_PutInt((int)Mode,1);
	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;

}

/* Returns the Display Mode */
ErrorCode_t LCR_CMD_GetDisplayMode(LCR_DisplayMode_t *Mode)
{
	uint8 data;
	if(readCommand(0x0101, &data, 1))
		return FAIL;
	*Mode = (LCR_DisplayMode_t)data;
	return SUCCESS;
}

/* Change the Power Mode from/to STANDBY or NORMAL */
ErrorCode_t LCR_CMD_SetPowerMode(LCR_PowerMode_t Mode)
{
	//TBD
	return SUCCESS;
}

/* Read the current system power mode */
LCR_PowerMode_t LCR_CMD_GetPowerMode(void)
{
	uint8 data = 0;
	//TBD
	return (LCR_PowerMode_t) data;
}

/* Set the Tespattern as defined in LCR_TestPattern_t list */
ErrorCode_t LCR_CMD_SetTestPattern(LCR_TestPattern_t TestPtn)
{
	if(TestPtn > TEST_PTN_ANXI_CHECKER)
		return FAIL;

	/* Generate packet */
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0103);
	LCR_CMD_PKT_PutInt((int)TestPtn,1);
	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;
}

/* Returned set test pattern */
ErrorCode_t LCR_CMD_GetTestPattern(LCR_TestPattern_t *TestPtn)
{
	uint8 data;
	if(readCommand(0x0103, &data, 1))
		return FAIL;
	*TestPtn = (LCR_TestPattern_t)data;
	return SUCCESS;
}

/* Set the R,G,B LED current */
ErrorCode_t LCR_CMD_SetLEDCurrent(LCR_LEDCurrent_t *LEDSetting)
{
	/* Write LED current */
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0104);
	LCR_CMD_PKT_PutInt(LEDSetting->Red, 2);
	LCR_CMD_PKT_PutInt(LEDSetting->Green, 2);
	LCR_CMD_PKT_PutInt(LEDSetting->Blue, 2);
	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;
}

/* Read the R,G,B LED current */
ErrorCode_t LCR_CMD_GetLEDCurrent(LCR_LEDCurrent_t *LEDSetting)
{
	uint8 data[6];
	if(readCommand(0x0104, data, sizeof(data)))
		return FAIL;
	LEDSetting->Red = readInteger(data, 2);
	LEDSetting->Green = readInteger(data + 2, 2);
	LEDSetting->Blue = readInteger(data + 4, 2);

	return SUCCESS;
}

/* Download a 24bpp .BMP image */
ErrorCode_t LCR_CMD_DisplayStaticImage(char const *fileNameWithPath)
{
	/* Generate packet */
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0105);

	/*TBD - Check for return error*/
	LCR_CMD_PKT_PutFile(fileNameWithPath);

	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;
}

/* Displays solid filed color image */
ErrorCode_t LCR_CMD_DisplayStaticColor(uint32 Color)
{
	/* Generate packet */
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0106);
	LCR_CMD_PKT_PutInt((uint32)Color,4);
	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;
}

/* Configures the displayed image on the DMD*/
ErrorCode_t LCR_CMD_SetDisplaySetting(LCR_DisplaySetting_t const *Setting)
{
	/* Generate packet */
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0107);
	LCR_CMD_PKT_PutInt((int)Setting->LongAxisFlip,1);
	LCR_CMD_PKT_PutInt((int)Setting->ShortAxisFlip,1);
	LCR_CMD_PKT_PutInt((int)Setting->Rotate,1);
	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;
}


/* Returns the existing display settings */
ErrorCode_t LCR_CMD_GetDisplaySetting(LCR_DisplaySetting_t *Setting)
{
	uint8 data[3];
	if(readCommand(0x0107, data, sizeof(data)))
		return FAIL;
	Setting->LongAxisFlip = data[0];
	Setting->ShortAxisFlip = data[1];
	Setting->Rotate = data[2];

	return SUCCESS;
}

/* Configures the input video source settings */
ErrorCode_t LCR_CMD_SetVideoSetting(LCR_VideoSetting_t const *Setting)
{

	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0200);
	LCR_CMD_PKT_PutInt(Setting->ResolutionX,2);
	LCR_CMD_PKT_PutInt(Setting->ResolutionY,2);
	LCR_CMD_PKT_PutInt(Setting->FirstPix,2);
	LCR_CMD_PKT_PutInt(Setting->FirstLine,2);
	LCR_CMD_PKT_PutInt(Setting->ActiveWidth,2);
	LCR_CMD_PKT_PutInt(Setting->ActiveHeight,2);
	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;
}

/* Returns currently set video settings */
ErrorCode_t LCR_CMD_GetVideoSetting(LCR_VideoSetting_t *Setting)
{
	uint8 data[12];
	if(readCommand(0x0200, data, sizeof(data)))
		return FAIL;
	Setting->ResolutionX = readInteger(data, 2);
	Setting->ResolutionY = readInteger(data + 2, 2);
	Setting->FirstPix = readInteger(data + 4, 2);
	Setting->FirstLine = readInteger(data + 6, 2);
	Setting->ActiveWidth = readInteger(data + 8, 2);
	Setting->ActiveHeight = readInteger(data + 10, 2);

	return SUCCESS;
}

/* Set Video Mode */
ErrorCode_t LCR_CMD_SetVideoMode(LCR_VideoModeSetting_t *Setting)
{
	/*Frame the packet*/
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0201);
	LCR_CMD_PKT_PutInt((int)Setting->FrameRate,1);
	LCR_CMD_PKT_PutInt((int)Setting->BitDepth,1);
	LCR_CMD_PKT_PutInt((int)Setting->RGB,1);

	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;
}


/* Return current video mode settings */
ErrorCode_t LCR_CMD_GetVideoMode(LCR_VideoModeSetting_t *Setting)
{
	uint8 data[3];
	if(readCommand(0x0201, data, sizeof(data)))
		return FAIL;
	Setting->FrameRate = data[0];
	Setting->BitDepth = data[1];
	Setting->RGB = data[2];

	return SUCCESS;
}

/* Configures pattern interleaving */
ErrorCode_t LCR_CMD_SetInterleavePatternOrder(uint8 NumPatterns, uint8 const *PatternOrder)
{
	/*TBD*/
	return SUCCESS;
}

/* Returns pattern interleave order */
ErrorCode_t LCR_CMD_GetInterleavePatternOrder(uint8 *NumPatterns, uint8 *PatternOrder)
{
	/*TBD*/
	return SUCCESS;
}

/* Defines pattern sequence */
ErrorCode_t LCR_CMD_SetPatternSeqSetting(LCR_PatternSeqSetting_t const *Setting)
{
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0480); //The old cmd 0x0400 is kept for backward compatibility only use the new one
	LCR_CMD_PKT_PutInt((int)Setting->BitDepth,1);
	LCR_CMD_PKT_PutInt((int)Setting->NumPatterns,2);
	LCR_CMD_PKT_PutInt((int)Setting->PatternType,1);
	LCR_CMD_PKT_PutInt((int)Setting->InputTriggerType,1);
	LCR_CMD_PKT_PutInt((uint32)Setting->InputTriggerDelay,4);
	LCR_CMD_PKT_PutInt((uint32)Setting->AutoTriggerPeriod,4);
	LCR_CMD_PKT_PutInt((uint32)Setting->ExposureTime,4);
	LCR_CMD_PKT_PutInt((int)Setting->LEDSelect,1);
	LCR_CMD_PKT_PutInt((int)Setting->Repeat,1);

	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;
}


/* Return currently set pattern sequence */
ErrorCode_t LCR_CMD_GetPatternSeqSetting(LCR_PatternSeqSetting_t *Setting)
{
	uint8 data[19];
	if(readCommand(0x0480, data, sizeof(data)))
		return FAIL;
	Setting->BitDepth = data[0];
	Setting->NumPatterns = readInteger(data + 1, 2);
	Setting->PatternType = (LCR_PatternType_t)data[3];
	Setting->InputTriggerType = (LCR_TriggerType_t)data[4];
	Setting->InputTriggerDelay = readInteger(data + 5, 4);
	Setting->AutoTriggerPeriod = readInteger(data + 9, 4);
	Setting->ExposureTime = readInteger(data + 13, 4);
	Setting->LEDSelect = (LCR_LEDSelect_t)data[17];
	Setting->Repeat = data[18];

	return SUCCESS;
}

/* Downloads a bmp file in pattern sequence mode */
ErrorCode_t LCR_CMD_DefinePatternBMP(LCR_PatternCount_t PatternNum, char const *fileNameWithPath)
{
	/* Generate packet */
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0401);
	LCR_CMD_PKT_PutInt((int)PatternNum, 1);
	LCR_CMD_PKT_PutFile(fileNameWithPath);
	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;
}

/* Reads and stores a pattern from the DLP LightCrafter(TM) in the .bmp format with <file_name>*/
ErrorCode_t LCR_CMD_ReadPattern(LCR_PatternCount_t PatternNum, char *fileName)
{
	unsigned long int numbytes;

	//Determine the bit depth
	LCR_PatternSeqSetting_t Setting;
	LCR_CMD_GetPatternSeqSetting(&Setting);
	switch(Setting.BitDepth)
	{
	case 1:
		numbytes = ONE_BPP_PTN_SIZE;
		break;
	case 2:
		numbytes = TWO_BPP_PTN_SIZE;
		break;
	case 3:
		numbytes = THREE_BPP_PTN_SIZE;
		break;
	case 4:
		numbytes = FOUR_BPP_PTN_SIZE;
		break;
	case 5:
		numbytes = FIVE_BPP_PTN_SIZE;
		break;
	case 6:
		numbytes = SIX_BPP_PTN_SIZE;
		break;
	case 7:
		numbytes = SEVEN_BPP_PTN_SIZE;
		break;
	case 8:
		numbytes = EIGHT_BPP_PTN_SIZE;
		break;
	default:
		return FAIL;
	}

	/* Generate packet */
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_READ, 0x0401);
	LCR_CMD_PKT_PutData((uint8*) &PatternNum, 1);
	LCR_CMD_PKT_SendCommand();
	LCR_CMD_PKT_GetFile(fileName,numbytes);

	return SUCCESS;
}

/* Starts/Stop of pattern sequence */
ErrorCode_t LCR_CMD_StartPatternSeq(uint8 Start)
{
	/* Generate packet */
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0402);
	LCR_CMD_PKT_PutInt(Start,1);
	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;
}

/* Displays next pattern in pattern sequence applicable only if Input Trigger Type = Command Trigger (0x00) */
ErrorCode_t LCR_CMD_AdvancePatternSeq(void)
{
	/* Generate packet */
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0403);
	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;
}

/* Returns the configuration of the CameraTrigger signal */
ErrorCode_t LCR_CMD_GetCamTriggerSetting(LCR_CamTriggerSetting_t *Setting)
{
	uint8 data[11];
	if(readCommand(0x0404, data, sizeof(data)))
		return FAIL;
	Setting->Enable = data[0];
	Setting->Source = data[1];
	Setting->Polarity = data[2];
	Setting->Delay = readInteger(data + 3, 4);
	Setting->PulseWidth = readInteger(data + 7, 4);

	return SUCCESS;
}

/* Updates the camera trigger configuration */
ErrorCode_t LCR_CMD_SetCamTriggerSetting(LCR_CamTriggerSetting_t *Setting)
{
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0404);
	LCR_CMD_PKT_PutInt((int)Setting->Enable,1);
	LCR_CMD_PKT_PutInt((int)Setting->Source,1);
	LCR_CMD_PKT_PutInt((int)Setting->Polarity,1);
	LCR_CMD_PKT_PutInt((uint32)Setting->Delay,4);
	LCR_CMD_PKT_PutInt((uint32)Setting->PulseWidth,4);
	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;
}

/*Send the HW Pattern Sequence Definition information */
ErrorCode_t LCR_CMD_DefineHWPatSequence(LCR_HWPatternSeqDef_t *hwPatSeqDef)
{
	int i;
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0406);
	LCR_CMD_PKT_PutInt((int)hwPatSeqDef->index,1);
	LCR_CMD_PKT_PutInt((int)hwPatSeqDef->numOfPatn,1);
	for(i=0; i<hwPatSeqDef->numOfPatn;i++)
	{
		LCR_CMD_PKT_PutInt((int)hwPatSeqDef->hwPatArray[i].Number,1);
		LCR_CMD_PKT_PutInt((int)hwPatSeqDef->hwPatArray[i].Invert,1);
	}
	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;
}

/* Save current setting and Display mode as solution */
ErrorCode_t LCR_CMD_SaveSolution(char *SolutionName)
{
	uint8 tempName[LCR_CMD_SOLUTION_NAME_LEN];

	//copy the string into temporary buffer
	strcpy((char*)&tempName[0],SolutionName);

	//Frame the packet
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0600);
	LCR_CMD_PKT_PutData((uint8*)&tempName[0],LCR_CMD_SOLUTION_NAME_LEN);
	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;
}

/* Retrieve the details of the solution stored */
ErrorCode_t LCR_CMD_GetSolutionNames(uint8 *Count, uint8 *DefaultSolution, char *SolutionName)
{
	//Frame the packet
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_READ, 0x0600);
	LCR_CMD_PKT_SendCommand();

	LCR_CMD_PKT_GetData(Count,1);
	LCR_CMD_PKT_GetData(DefaultSolution,1);
	LCR_CMD_PKT_GetData((uint8*)SolutionName,((*Count)*LCR_CMD_SOLUTION_NAME_LEN));

	return SUCCESS;
}

/* Allows Loading, Deleting and Setting an exisitng solution in the system */
ErrorCode_t LCR_CMD_ManageSolution(LCR_SolutionCommand_t Cmd, char *SolutionName)
{
	uint8 tempName[LCR_CMD_SOLUTION_NAME_LEN];

	//copy the string into temporary buffer
	strcpy((char*)&tempName[0],SolutionName);

	//Frame the packet
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0601);
	LCR_CMD_PKT_PutData((uint8*)&Cmd,1);
	LCR_CMD_PKT_PutData((uint8*)&tempName[0],LCR_CMD_SOLUTION_NAME_LEN);
	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;
}

/*Downloads the new Sequence LUT into the DLPC300 controller*/
ErrorCode_t LCR_CMD_LoadCustomSequence(char *seqBinFileName)
{
	/* Generate packet */
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0A00);

	/*TBD - Check for return error*/
	LCR_CMD_PKT_PutFile(seqBinFileName);

	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;
}

/*Setup the vectors*/
ErrorCode_t LCR_CMD_SetupCustomSequencevectors(uint8 startVector, uint8 numOfvectors)
{
	/* Generate packet */
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0A01);
	LCR_CMD_PKT_PutData((uint8*)&startVector,1);
	LCR_CMD_PKT_PutData((uint8*)&numOfvectors,1);

	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	return SUCCESS;
}
