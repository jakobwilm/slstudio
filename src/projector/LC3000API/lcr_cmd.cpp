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

	LCR_CMD_PKT_GetData((uint8*)VersionStr,LCR_CMD_VERSION_STR_LEN);

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
LCR_DisplayMode_t LCR_CMD_GetDisplayMode(void)
{
	uint8 data;

	/* Read display mode */
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_READ, 0x0101);
	LCR_CMD_PKT_SendCommand();
	LCR_CMD_PKT_GetData((uint8*)&data,1);

	return (LCR_DisplayMode_t)data;
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
LCR_TestPattern_t LCR_CMD_GetTestPattern(void)
{

	uint8 data;

	/* Read Test Pattern set */
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_READ, 0x0103);
	LCR_CMD_PKT_SendCommand();
	LCR_CMD_PKT_GetData((uint8*)&data,1);

	return (LCR_TestPattern_t)data;
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
	/* Read LED current */
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_READ, 0x0104);
	LCR_CMD_PKT_SendCommand();

	LEDSetting->Red = LCR_CMD_PKT_GetInt(2);
	LEDSetting->Green = LCR_CMD_PKT_GetInt(2);
	LEDSetting->Blue = LCR_CMD_PKT_GetInt(2);

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
	uint8 data;

	/* Display Settings */
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_READ, 0x0107);
	LCR_CMD_PKT_SendCommand();
	LCR_CMD_PKT_GetData(&data,1);
	Setting->LongAxisFlip = data;
	LCR_CMD_PKT_GetData(&data,1);
	Setting->ShortAxisFlip = data;
	LCR_CMD_PKT_GetData(&data,1);
	Setting->Rotate = data;

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
	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_READ, 0x0200);
	LCR_CMD_PKT_SendCommand();

	//Assign to the structures
	Setting->ResolutionX = LCR_CMD_PKT_GetInt(2);
	Setting->ResolutionY = LCR_CMD_PKT_GetInt(2);
	Setting->FirstPix = LCR_CMD_PKT_GetInt(2);
	Setting->FirstLine = LCR_CMD_PKT_GetInt(2);
	Setting->ActiveWidth = LCR_CMD_PKT_GetInt(2);
	Setting->ActiveHeight = LCR_CMD_PKT_GetInt(2);

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
	uint8 data;

	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_READ, 0x0201);
	LCR_CMD_PKT_SendCommand();

	LCR_CMD_PKT_GetData(&data,1);
	Setting->FrameRate = data;
	LCR_CMD_PKT_GetData(&data,1);
	Setting->BitDepth = data;
	LCR_CMD_PKT_GetData(&data,1);
	Setting->RGB = data;

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
	uint8  data;
	uint32 data32;

	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_READ, 0x0480);
	if(LCR_CMD_PKT_SendCommand())
		return FAIL;

	LCR_CMD_PKT_GetData(&data,1);
	Setting->BitDepth = data;

	Setting->NumPatterns = LCR_CMD_PKT_GetInt(2);

	LCR_CMD_PKT_GetData(&data,1);
	Setting->PatternType = (LCR_PatternType_t) data;

	LCR_CMD_PKT_GetData(&data,1);
	Setting->InputTriggerType = (LCR_TriggerType_t)data;

	LCR_CMD_PKT_GetData((uint8*)&data32,4);
	Setting->InputTriggerDelay = data32;

	LCR_CMD_PKT_GetData((uint8*)&data32,4);
	Setting->AutoTriggerPeriod = data32;

	LCR_CMD_PKT_GetData((uint8*)&data32,4);
	Setting->ExposureTime = data32;

	LCR_CMD_PKT_GetData(&data,1);
	Setting->LEDSelect = (LCR_LEDSelect_t) data;

	LCR_CMD_PKT_GetData(&data,1);
	Setting->Repeat = data;

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
	uint8  data;
	uint32 data32;

	LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_READ, 0x0404);
	LCR_CMD_PKT_SendCommand();

	LCR_CMD_PKT_GetData(&data,1);
	Setting->Enable = data;
	LCR_CMD_PKT_GetData(&data,1);
	Setting->Source = data;
	LCR_CMD_PKT_GetData(&data,1);
	Setting->Polarity = data;
	LCR_CMD_PKT_GetData((uint8*)&data32,4);
	Setting->Delay = data32;
	LCR_CMD_PKT_GetData((uint8*)&data32,4);
	Setting->PulseWidth = data32;

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
