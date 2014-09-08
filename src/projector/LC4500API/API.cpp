/*
 * API.cpp
 *
 * This module provides C callable APIs for each of the command supported by LightCrafter4500 platform and detailed in the programmer's guide.
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
*/

#include "API.h"
#include "string.h"
#include "usb.h"
#include "Common.h"
#include <stdlib.h>

extern unsigned char OutputBuffer[];
extern unsigned char InputBuffer[];

CmdFormat CmdList[255] =
{
    {   0x1A,  0x00,  0x01   },      //SOURCE_SEL,
    {   0x1A,  0x02,  0x01   },      //PIXEL_FORMAT,
    {   0x1A,  0x03,  0x01   },      //CLK_SEL,
    {   0x1A,  0x37,  0x01   },      //CHANNEL_SWAP,
    {   0x1A,  0x04,  0x01   },      //FPD_MODE,
    {   0,  0,  0   },      //CURTAIN_COLOR,
    {   0x02,  0x00,  0x01   },      //POWER_CONTROL,
    {   0x10,  0x08,  0x01   },      //FLIP_LONG,
    {   0x10,  0x09,  0x01   },      //FLIP_SHORT,
    {   0x12,  0x03,  0x01   },      //TPG_SEL,
    {   0x1A,  0x05,  0x01   },      //PWM_INVERT,
    {   0x1A,  0x07,  0x01   },      //LED_ENABLE,
    {   0x02,  0x05,  0x00   },      //GET_VERSION,
    {   0x08,  0x02,  0x00   },      //SW_RESET,
    {   0,  0,  0   },      //DMD_PARK,
    {   0,  0,  0   },      //BUFFER_FREEZE,
    {   0x1A,  0x0A,  0x00   },      //STATUS_HW,
    {   0x1A,  0x0B,  0x00   },      //STATUS_SYS,
    {   0x1A,  0x0C,  0x00   },      //STATUS_MAIN,
    {   0,  0,  0   },      //CSC_DATA,
    {   0,  0,  0   },      //GAMMA_CTL,
    {   0,  0,  0   },      //BC_CTL,
    {   0x1A,  0x10,  0x01   },      //PWM_ENABLE,
    {   0x1A,  0x11,  0x06   },      //PWM_SETUP,
    {   0x1A,  0x12,  0x05   },      //PWM_CAPTURE_CONFIG,
    {   0x1A,  0x38,  0x02   },      //GPIO_CONFIG,
    {   0x0B,  0x01,  0x03   },      //LED_CURRENT,
    {   0x10,  0x00,  0x10   },      //DISP_CONFIG,
    {   0,  0,  0   },      //TEMP_CONFIG,
    {   0,  0,  0   },      //TEMP_READ,
    {   0x1A,  0x16,  0x09   },      //MEM_CONTROL,
    {   0,  0,  0   },      //I2C_CONTROL,
    {   0x1A,  0x1A,  0x01   },      //LUT_VALID,
    {   0x1A,  0x1B,  0x01   },      //DISP_MODE,
    {   0x1A,  0x1D,  0x03   },      //TRIG_OUT1_CTL,
    {   0x1A,  0x1E,  0x03   },      //TRIG_OUT2_CTL,
    {   0x1A,  0x1F,  0x02   },      //RED_STROBE_DLY,
    {   0x1A,  0x20,  0x02   },      //GRN_STROBE_DLY,
    {   0x1A,  0x21,  0x02   },      //BLU_STROBE_DLY,
    {   0x1A,  0x22,  0x01   },      //PAT_DISP_MODE,
    {   0x1A,  0x23,  0x01   },      //PAT_TRIG_MODE,
    {   0x1A,  0x24,  0x01   },      //PAT_START_STOP,
    {   0,  0,  0   },      //BUFFER_SWAP,
    {   0,  0,  0   },      //BUFFER_WR_DISABLE,
    {   0,  0,  0   },      //CURRENT_RD_BUFFER,
    {   0x1A,  0x29,  0x08   },      //PAT_EXPO_PRD,
    {   0x1A,  0x30,  0x01   },      //INVERT_DATA,
    {   0x1A,  0x31,  0x04   },      //PAT_CONFIG,
    {   0x1A,  0x32,  0x01   },      //MBOX_ADDRESS,
    {   0x1A,  0x33,  0x01   },      //MBOX_CONTROL,
    {   0x1A,  0x34,  0x00   },      //MBOX_DATA,
    {   0x1A,  0x35,  0x04   },      //TRIG_IN1_DELAY,
    {   0,  0,  0   },      //TRIG_IN2_CONTROL,
    {   0x1A,  0x39,  0x01   },      //SPLASH_LOAD,
    {   0x1A,  0x3A,  0x02   },      //SPLASH_LOAD_TIMING,
    {   0x08,  0x07,  0x03   },      //GPCLK_CONFIG,
    {   0,  0,  0   },      //PULSE_GPIO_23,
    {   0,  0,  0   },      //ENABLE_LCR_DEBUG,
    {   0x12,  0x04,  0x0C   },      //TPG_COLOR,
    {   0x1A,  0x13,  0x05   },     //PWM_CAPTURE_READ,
    {   0x30,  0x01,  0x00   },     //PROG_MODE,
    {   0x00,  0x00,  0x00   },     //BL_STATUS
    {   0x00,  0x23,  0x01   },     //BL_SPL_MODE
    {   0x00,  0x15,  0x01   },     //BL_GET_MANID,
    {   0x00,  0x15,  0x01   },     //BL_GET_DEVID,
    {   0x00,  0x15,  0x01   },     //BL_GET_CHKSUM,
    {   0x00,  0x29,  0x04   },     //BL_SETSECTADDR,
    {   0x00,  0x28,  0x00   },     //BL_SECT_ERASE,
    {   0x00,  0x2C,  0x04   },     //BL_SET_DNLDSIZE,
    {   0x00,  0x25,  0x00   },     //BL_DNLD_DATA,
    {   0x00,  0x2F,  0x01   },     //BL_FLASH_TYPE,
    {   0x00,  0x26,  0x00   },     //BL_CALC_CHKSUM,
    {   0x00,  0x30,  0x01   }     //BL_PROG_MODE,
};

static unsigned char seqNum=0;
static unsigned int PatLut[128] = {0};
static unsigned int PatLutIndex = 0;

int LCR_Write()
{
    return USB_Write();
}

int LCR_Read()
{
    int ret_val;
    hidMessageStruct *pMsg = (hidMessageStruct *)InputBuffer;
    if(USB_Write() > 0)
    {
        ret_val =  USB_Read();

        if((pMsg->head.flags.nack == 1) || (pMsg->head.length == 0))
            return -2;
        else
            return ret_val;
    }
    return -1;
}

int LCR_ContinueRead()
{
    return USB_Read();
}

int LCR_SendMsg(hidMessageStruct *pMsg)
{
    int maxDataSize = USB_MAX_PACKET_SIZE-sizeof(pMsg->head);
    int dataBytesSent = MIN(pMsg->head.length, maxDataSize);    //Send all data or max possible

    OutputBuffer[0]=0; // First byte is the report number
    memcpy(&OutputBuffer[1], pMsg, (sizeof(pMsg->head) + dataBytesSent));

    if(LCR_Write() < 0)
        return -1;

    //dataBytesSent = maxDataSize;

    while(dataBytesSent < pMsg->head.length)
    {
        memcpy(&OutputBuffer[1], &pMsg->text.data[dataBytesSent], USB_MAX_PACKET_SIZE);
        if(LCR_Write() < 0)
            return -1;
        dataBytesSent += USB_MAX_PACKET_SIZE;
    }
    return dataBytesSent+sizeof(pMsg->head);
}

int LCR_PrepReadCmd(LCR_CMD cmd)
{
    hidMessageStruct msg;

    msg.head.flags.rw = 1; //Read
    msg.head.flags.reply = 1; //Host wants a reply from device
    msg.head.flags.dest = 0; //Projector Control Endpoint
    msg.head.flags.reserved = 0;
    msg.head.flags.nack = 0;
    msg.head.seq = 0;

    msg.text.cmd = (CmdList[cmd].CMD2 << 8) | CmdList[cmd].CMD3;
    msg.head.length = 2;

    if(cmd == BL_GET_MANID)
    {
        msg.text.data[2] = 0x0C;
        msg.head.length += 1;
    }
    else if (cmd == BL_GET_DEVID)
    {
        msg.text.data[2] = 0x0D;
        msg.head.length += 1;
    }
    else if (cmd == BL_GET_CHKSUM)
    {
        msg.text.data[2] = 0x00;
        msg.head.length += 1;
    }

    OutputBuffer[0]=0; // First byte is the report number
    memcpy(&OutputBuffer[1], &msg, (sizeof(msg.head)+sizeof(msg.text.cmd) + msg.head.length));
    return 0;
}

int LCR_PrepReadCmdWithParam(LCR_CMD cmd, unsigned char param)
{
    hidMessageStruct msg;

    msg.head.flags.rw = 1; //Read
    msg.head.flags.reply = 1; //Host wants a reply from device
    msg.head.flags.dest = 0; //Projector Control Endpoint
    msg.head.flags.reserved = 0;
    msg.head.flags.nack = 0;
    msg.head.seq = 0;

    msg.text.cmd = (CmdList[cmd].CMD2 << 8) | CmdList[cmd].CMD3;
    msg.head.length = 3;

    msg.text.data[2] = param;

    OutputBuffer[0]=0; // First byte is the report number
    memcpy(&OutputBuffer[1], &msg, (sizeof(msg.head)+sizeof(msg.text.cmd) + msg.head.length));
    return 0;
}

int LCR_PrepMemReadCmd(unsigned int addr)
{
    hidMessageStruct msg;

    msg.head.flags.rw = 1; //Read
    msg.head.flags.reply = 1; //Host wants a reply from device
    msg.head.flags.dest = 0; //Projector Control Endpoint
    msg.head.flags.reserved = 0;
    msg.head.flags.nack = 0;
    msg.head.seq = 0;

    msg.text.cmd = (CmdList[MEM_CONTROL].CMD2 << 8) | CmdList[MEM_CONTROL].CMD3;
    msg.head.length = 6;

    msg.text.data[2] = addr;
    msg.text.data[3] = addr >>8;
    msg.text.data[4] = addr >>16;
    msg.text.data[5] = addr >>24;

    OutputBuffer[0]=0; // First byte is the report number
    memcpy(&OutputBuffer[1], &msg, (sizeof(msg.head)+sizeof(msg.text.cmd) + msg.head.length));
    return 0;
}

int LCR_PrepWriteCmd(hidMessageStruct *pMsg, LCR_CMD cmd)
{
    pMsg->head.flags.rw = 0; //Write
    pMsg->head.flags.reply = 0; //Host wants a reply from device
    pMsg->head.flags.dest = 0; //Projector Control Endpoint
    pMsg->head.flags.reserved = 0;
    pMsg->head.flags.nack = 0;
    pMsg->head.seq = seqNum++;

    pMsg->text.cmd = (CmdList[cmd].CMD2 << 8) | CmdList[cmd].CMD3;
    pMsg->head.length = CmdList[cmd].len + 2;

    return 0;
}

int LCR_GetVersion(unsigned int *pApp_ver, unsigned int *pAPI_ver, unsigned int *pSWConfig_ver, unsigned int *pSeqConfig_ver)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(GET_VERSION);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *pApp_ver = *(unsigned int *)&msg.text.data[0];
        *pAPI_ver = *(unsigned int *)&msg.text.data[4];
        *pSWConfig_ver = *(unsigned int *)&msg.text.data[8];
        *pSeqConfig_ver = *(unsigned int *)&msg.text.data[12];
        return 0;
    }
    return -1;
}

int LCR_GetLedEnables(bool *pSeqCtrl, bool *pRed, bool *pGreen, bool *pBlue)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(LED_ENABLE);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        if(msg.text.data[0] & BIT0)
            *pRed = true;
        else
            *pRed = false;

        if(msg.text.data[0] & BIT1)
            *pGreen = true;
        else
            *pGreen = false;

        if(msg.text.data[0] & BIT2)
            *pBlue = true;
        else
            *pBlue = false;

        if(msg.text.data[0] & BIT3)
            *pSeqCtrl = true;
        else
            *pSeqCtrl = false;
        return 0;
    }
    return -1;
}


int LCR_SetLedEnables(bool SeqCtrl, bool Red, bool Green, bool Blue)
{
    hidMessageStruct msg;
    unsigned char Enable=0;

    if(SeqCtrl)
        Enable |= BIT3;
    if(Red)
        Enable |= BIT0;
    if(Green)
        Enable |= BIT1;
    if(Blue)
        Enable |= BIT2;

    msg.text.data[2] = Enable;
    LCR_PrepWriteCmd(&msg, LED_ENABLE);

    return LCR_SendMsg(&msg);
}

int LCR_GetLedCurrents(unsigned char *pRed, unsigned char *pGreen, unsigned char *pBlue)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(LED_CURRENT);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *pRed = msg.text.data[0];
        *pGreen = msg.text.data[1];
        *pBlue = msg.text.data[2];

        return 0;
    }
    return -1;
}


int LCR_SetLedCurrents(unsigned char RedCurrent, unsigned char GreenCurrent, unsigned char BlueCurrent)
{
    hidMessageStruct msg;

    msg.text.data[2] = RedCurrent;
    msg.text.data[3] = GreenCurrent;
    msg.text.data[4] = BlueCurrent;

    LCR_PrepWriteCmd(&msg, LED_CURRENT);

    return LCR_SendMsg(&msg);
}

bool LCR_GetLongAxisImageFlip(void)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(FLIP_LONG);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        if ((msg.text.data[0] & BIT0) == BIT0)
            return true;
        else
            return false;
    }
    return false;
}

bool LCR_GetShortAxisImageFlip(void)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(FLIP_SHORT);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        if ((msg.text.data[0] & BIT0) == BIT0)
            return true;
        else
            return false;
    }
    return false;
}


int LCR_SetLongAxisImageFlip(bool Flip)
{
    hidMessageStruct msg;

    if(Flip)
        msg.text.data[2] = BIT0;
    else
        msg.text.data[2] = 0;

    LCR_PrepWriteCmd(&msg, FLIP_LONG);

    return LCR_SendMsg(&msg);
}

int LCR_SetShortAxisImageFlip(bool Flip)
{
    hidMessageStruct msg;

    if(Flip)
        msg.text.data[2] = BIT0;
    else
        msg.text.data[2] = 0;

    LCR_PrepWriteCmd(&msg, FLIP_SHORT);

    return LCR_SendMsg(&msg);
}

int LCR_SetProgrammingMode(bool EnterProgMode)
{
    hidMessageStruct msg;

    if(EnterProgMode)
        msg.text.data[2] = 1;
    else
        msg.text.data[2] = 2;

    LCR_PrepWriteCmd(&msg, PROG_MODE);

    return LCR_SendMsg(&msg);
}

int LCR_ExitProgrammingMode(void)
{
    hidMessageStruct msg;

    msg.text.data[2] = 2;
    LCR_PrepWriteCmd(&msg, BL_PROG_MODE);

    return LCR_SendMsg(&msg);
}


int LCR_GetProgrammingMode(bool *ProgMode)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(PROG_MODE);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        if(msg.text.data[0] == 1)
            *ProgMode = true;
        else
            *ProgMode = false;
        return 0;
    }
    return -1;
}

int LCR_GetFlashManID(unsigned short *pManID)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(BL_GET_MANID);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *pManID = msg.text.data[6];
        *pManID |= (unsigned short)msg.text.data[7] << 8;
        return 0;
    }
    return -1;
}

int LCR_GetFlashDevID(unsigned long long *pDevID)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(BL_GET_DEVID);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *pDevID = msg.text.data[6];
        *pDevID |= (unsigned long long)msg.text.data[7] << 8;
        *pDevID |= (unsigned long long)msg.text.data[8] << 16;
        *pDevID |= (unsigned long long)msg.text.data[9] << 24;
        *pDevID |= (unsigned long long)msg.text.data[12] << 32;
        *pDevID |= (unsigned long long)msg.text.data[13] << 40;
        *pDevID |= (unsigned long long)msg.text.data[14] << 48;
        *pDevID |= (unsigned long long)msg.text.data[15] << 56;
        return 0;
    }
    return -1;
}

int LCR_GetBLStatus(unsigned char *BL_Status)
{
    hidMessageStruct msg;

    /* For some reason BL_STATUS readback is not working properly.
     * However, after going through the bootloader code, I have ascertained that any
     * readback is fine - Byte 0 is always teh bootloader status */
    LCR_PrepReadCmd(BL_GET_CHKSUM);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *BL_Status = msg.text.data[0];
        return 0;
    }
    return -1;
}

int LCR_BLSpecialMode(unsigned int Mode)
{
    hidMessageStruct msg;

    msg.text.data[2] = Mode;

    LCR_PrepWriteCmd(&msg, BL_SPL_MODE);

    return LCR_SendMsg(&msg);
}

int LCR_SetFlashAddr(unsigned int Addr)
{
    hidMessageStruct msg;

    msg.text.data[2] = Addr;
    msg.text.data[3] = Addr >> 8;
    msg.text.data[4] = Addr >> 16;
    msg.text.data[5] = Addr >> 24;

    LCR_PrepWriteCmd(&msg, BL_SET_SECTADDR);

    return LCR_SendMsg(&msg);
}

 int LCR_FlashSectorErase(void)
 {
     hidMessageStruct msg;

     LCR_PrepWriteCmd(&msg, BL_SECT_ERASE);
     return LCR_SendMsg(&msg);
 }

int LCR_SetDownloadSize(unsigned int dataLen)
{
    hidMessageStruct msg;

    msg.text.data[2] = dataLen;
    msg.text.data[3] = dataLen >> 8;
    msg.text.data[4] = dataLen >> 16;
    msg.text.data[5] = dataLen >> 24;

    LCR_PrepWriteCmd(&msg, BL_SET_DNLDSIZE);

    return LCR_SendMsg(&msg);
}

int LCR_DownloadData(unsigned char *pByteArray, unsigned int dataLen)
{
    hidMessageStruct msg;
    int retval;
    unsigned int sendSize;

    sendSize = HID_MESSAGE_MAX_SIZE - sizeof(msg.head)- sizeof(msg.text.cmd) - 2;//The last -2 is to workaround a bug in bootloader.

    if(dataLen > sendSize)
        dataLen = sendSize;

    CmdList[BL_DNLD_DATA].len = dataLen;
    memcpy(&msg.text.data[2], pByteArray, dataLen);

    LCR_PrepWriteCmd(&msg, BL_DNLD_DATA);

    retval = LCR_SendMsg(&msg);
    if(retval > 0)
        return dataLen;

    return -1;
}

void LCR_WaitForFlashReady()
{
    unsigned char BLstatus=STAT_BIT_FLASH_BUSY;

    do
    {
        LCR_GetBLStatus(&BLstatus);
    }
    while((BLstatus & STAT_BIT_FLASH_BUSY) == STAT_BIT_FLASH_BUSY);//Wait for flash busy flag to go off
}

int LCR_SetFlashType(unsigned char Type)
{
    hidMessageStruct msg;

    msg.text.data[2] = Type;

    LCR_PrepWriteCmd(&msg, BL_FLASH_TYPE);

    return LCR_SendMsg(&msg);
}

int LCR_CalculateFlashChecksum(void)
{
    hidMessageStruct msg;

    LCR_PrepWriteCmd(&msg, BL_CALC_CHKSUM);

    if(LCR_SendMsg(&msg) <= 0)
        return -1;

    return 0;

}

int LCR_GetFlashChecksum(unsigned int*checksum)
{
    hidMessageStruct msg;
#if 0
    LCR_PrepWriteCmd(&msg, BL_CALC_CHKSUM);

    if(LCR_SendMsg(&msg) <= 0)
        return -1;

    LCR_WaitForFlashReady();
#endif
    LCR_PrepReadCmd(BL_GET_CHKSUM);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *checksum = msg.text.data[6];
        *checksum |= (unsigned int)msg.text.data[7] << 8;
        *checksum |= (unsigned int)msg.text.data[8] << 16;
        *checksum |= (unsigned int)msg.text.data[9] << 24;
        return 0;
    }
    return -1;
}

int LCR_GetStatus(unsigned char *pHWStatus, unsigned char *pSysStatus, unsigned char *pMainStatus)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(STATUS_HW);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *pHWStatus = msg.text.data[0];
    }
    else
        return -1;

    LCR_PrepReadCmd(STATUS_SYS);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *pSysStatus = msg.text.data[0];
    }
    else
        return -1;

    LCR_PrepReadCmd(STATUS_MAIN);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *pMainStatus = msg.text.data[0];
    }
    else
        return -1;

    return 0;
}

int LCR_SoftwareReset(void)
{
    hidMessageStruct msg;

    LCR_PrepWriteCmd(&msg, SW_RESET);

    return LCR_SendMsg(&msg);
}

int LCR_SetMode(bool SLmode)
{
    hidMessageStruct msg;

    msg.text.data[2] = SLmode;
    LCR_PrepWriteCmd(&msg, DISP_MODE);

    return LCR_SendMsg(&msg);
}

int LCR_GetMode(bool *pMode)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(DISP_MODE);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pMode = (msg.text.data[0] != 0);
        return 0;
    }
    return -1;
}

int LCR_SetPowerMode(bool Standby)
{
    hidMessageStruct msg;

    msg.text.data[2] = Standby;
    LCR_PrepWriteCmd(&msg, POWER_CONTROL);

    return LCR_SendMsg(&msg);
}

int LCR_SetRedLEDStrobeDelay(unsigned char rising, unsigned char falling)
{
    hidMessageStruct msg;

    msg.text.data[2] = rising;
    msg.text.data[3] = falling;

    LCR_PrepWriteCmd(&msg, RED_STROBE_DLY);

    return LCR_SendMsg(&msg);
}

int LCR_SetGreenLEDStrobeDelay(unsigned char rising, unsigned char falling)
{
    hidMessageStruct msg;

    msg.text.data[2] = rising;
    msg.text.data[3] = falling;

    LCR_PrepWriteCmd(&msg, GRN_STROBE_DLY);

    return LCR_SendMsg(&msg);
}

int LCR_SetBlueLEDStrobeDelay(unsigned char rising, unsigned char falling)
{
    hidMessageStruct msg;

    msg.text.data[2] = rising;
    msg.text.data[3] = falling;

    LCR_PrepWriteCmd(&msg, BLU_STROBE_DLY);

    return LCR_SendMsg(&msg);
}

int LCR_GetRedLEDStrobeDelay(unsigned char *pRising, unsigned char *pFalling)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(RED_STROBE_DLY);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pRising = msg.text.data[0];
        *pFalling = msg.text.data[1];
        return 0;
    }
    return -1;
}

int LCR_GetGreenLEDStrobeDelay(unsigned char *pRising, unsigned char *pFalling)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(GRN_STROBE_DLY);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pRising = msg.text.data[0];
        *pFalling = msg.text.data[1];
        return 0;
    }
    return -1;
}

int LCR_GetBlueLEDStrobeDelay(unsigned char *pRising, unsigned char *pFalling)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(BLU_STROBE_DLY);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pRising = msg.text.data[0];
        *pFalling = msg.text.data[1];
        return 0;
    }
    return -1;
}

int LCR_SetInputSource(unsigned int source, unsigned int portWidth)
{
    hidMessageStruct msg;

    msg.text.data[2] = source;
    msg.text.data[2] |= portWidth << 3;
    LCR_PrepWriteCmd(&msg, SOURCE_SEL);

    return LCR_SendMsg(&msg);
}

int LCR_GetInputSource(unsigned int *pSource, unsigned int *pPortWidth)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(SOURCE_SEL);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pSource = msg.text.data[0] & (BIT0 | BIT1 | BIT2);
        *pPortWidth = msg.text.data[0] >> 3;
        return 0;
    }
    return -1;
}

int LCR_SetPatternDisplayMode(bool external)
{
    hidMessageStruct msg;

    if(external)
        msg.text.data[2] = 0;
    else
        msg.text.data[2] = 3;

    LCR_PrepWriteCmd(&msg, PAT_DISP_MODE);

    return LCR_SendMsg(&msg);
}

int LCR_GetPatternDisplayMode(bool *external)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(PAT_DISP_MODE);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        if(msg.text.data[0] == 0)
            *external = true;
        else
            *external = false;
        return 0;
    }
    return -1;
}

int LCR_SetPixelFormat(unsigned int format)
{
    hidMessageStruct msg;

    msg.text.data[2] = format;
    LCR_PrepWriteCmd(&msg, PIXEL_FORMAT);

    return LCR_SendMsg(&msg);
}

int LCR_GetPixelFormat(unsigned int *pFormat)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(PIXEL_FORMAT);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pFormat = msg.text.data[0] & (BIT0 | BIT1 | BIT2);
        return 0;
    }
    return -1;
}

int LCR_SetPortClock(unsigned int clock)
{
    hidMessageStruct msg;

    msg.text.data[2] = clock;
    LCR_PrepWriteCmd(&msg, CLK_SEL);

    return LCR_SendMsg(&msg);
}

int LCR_GetPortClock(unsigned int *pClock)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(CLK_SEL);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pClock = msg.text.data[0] & (BIT0 | BIT1 | BIT2);
        return 0;
    }
    return -1;
}

int LCR_SetDataChannelSwap(unsigned int port, unsigned int swap)
{
    hidMessageStruct msg;

    msg.text.data[2] = port << 7;
    msg.text.data[2] |= swap & (BIT0 | BIT1 | BIT2);
    LCR_PrepWriteCmd(&msg, CHANNEL_SWAP);

    return LCR_SendMsg(&msg);
}

int LCR_GetDataChannelSwap(unsigned int *pPort, unsigned int *pSwap)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(CHANNEL_SWAP);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pSwap = msg.text.data[0] & (BIT0 | BIT1 | BIT2);
        if(msg.text.data[0] & BIT7)
            *pPort = 1;
        else
            *pPort = 0;
        return 0;
    }
    return -1;
}

int LCR_SetFPD_Mode_Field(unsigned int PixelMappingMode, bool SwapPolarity, unsigned int FieldSignalSelect)
{
    hidMessageStruct msg;

    msg.text.data[2] = PixelMappingMode << 6;
    msg.text.data[2] |= FieldSignalSelect & (BIT0 | BIT1 | BIT2);
    if(SwapPolarity)
        msg.text.data[2] |= BIT3;
    LCR_PrepWriteCmd(&msg, FPD_MODE);

    return LCR_SendMsg(&msg);
}

int LCR_GetFPD_Mode_Field(unsigned int *pPixelMappingMode, bool *pSwapPolarity, unsigned int *pFieldSignalSelect)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(FPD_MODE);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pFieldSignalSelect = msg.text.data[0] & (BIT0 | BIT1 | BIT2);
        if(msg.text.data[0] & BIT3)
            *pSwapPolarity = 1;
        else
            *pSwapPolarity = 0;
        *pPixelMappingMode = msg.text.data[0] >> 6;
        return 0;
    }
    return -1;
}

int LCR_SetTPGSelect(unsigned int pattern)
{
    hidMessageStruct msg;

    msg.text.data[2] = pattern;
    LCR_PrepWriteCmd(&msg, TPG_SEL);

    return LCR_SendMsg(&msg);
}

int LCR_GetTPGSelect(unsigned int *pPattern)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(TPG_SEL);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pPattern = msg.text.data[0] & (BIT0 | BIT1 | BIT2 | BIT3);
        return 0;
    }
    return -1;
}

int LCR_LoadSplash(unsigned int index)
{
    hidMessageStruct msg;

    msg.text.data[2] = index;
    LCR_PrepWriteCmd(&msg, SPLASH_LOAD);

    return LCR_SendMsg(&msg);
}

int LCR_GetSplashIndex(unsigned int *pIndex)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(SPLASH_LOAD);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pIndex = msg.text.data[0];
        return 0;
    }
    return -1;
}

int LCR_SetDisplay(rectangle croppedArea, rectangle displayArea)
{
    hidMessageStruct msg;

    msg.text.data[2] = croppedArea.firstPixel & 0xFF;
    msg.text.data[3] = croppedArea.firstPixel >> 8;
    msg.text.data[4] = croppedArea.firstLine & 0xFF;
    msg.text.data[5] = croppedArea.firstLine >> 8;
    msg.text.data[6] = croppedArea.pixelsPerLine & 0xFF;
    msg.text.data[7] = croppedArea.pixelsPerLine >> 8;
    msg.text.data[8] = croppedArea.linesPerFrame & 0xFF;
    msg.text.data[9] = croppedArea.linesPerFrame >> 8;
    msg.text.data[10] = displayArea.firstPixel & 0xFF;
    msg.text.data[11] = displayArea.firstPixel >> 8;
    msg.text.data[12] = displayArea.firstLine & 0xFF;
    msg.text.data[13] = displayArea.firstLine >> 8;
    msg.text.data[14] = displayArea.pixelsPerLine & 0xFF;
    msg.text.data[15] = displayArea.pixelsPerLine >> 8;
    msg.text.data[16] = displayArea.linesPerFrame & 0xFF;
    msg.text.data[17] = displayArea.linesPerFrame >> 8;

    LCR_PrepWriteCmd(&msg, DISP_CONFIG);

    return LCR_SendMsg(&msg);
}

int LCR_GetDisplay(rectangle *pCroppedArea, rectangle *pDisplayArea)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(DISP_CONFIG);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        pCroppedArea->firstPixel = msg.text.data[0] | msg.text.data[1] << 8;
        pCroppedArea->firstLine = msg.text.data[2] | msg.text.data[3] << 8;
        pCroppedArea->pixelsPerLine = msg.text.data[4] | msg.text.data[5] << 8;
        pCroppedArea->linesPerFrame = msg.text.data[6] | msg.text.data[7] << 8;
        pDisplayArea->firstPixel = msg.text.data[8] | msg.text.data[9] << 8;
        pDisplayArea->firstLine = msg.text.data[10] | msg.text.data[11] << 8;
        pDisplayArea->pixelsPerLine = msg.text.data[12] | msg.text.data[13] << 8;
        pDisplayArea->linesPerFrame = msg.text.data[14] | msg.text.data[15] << 8;

        return 0;
    }
    return -1;
}

int LCR_SetTPGColor(unsigned short redFG, unsigned short greenFG, unsigned short blueFG, unsigned short redBG, unsigned short greenBG, unsigned short blueBG)
{
    hidMessageStruct msg;

    msg.text.data[2] = (char)redFG;
    msg.text.data[3] = (char)(redFG >> 8);
    msg.text.data[4] = (char)greenFG;
    msg.text.data[5] = (char)(greenFG >> 8);
    msg.text.data[6] = (char)blueFG;
    msg.text.data[7] = (char)(blueFG >> 8);
    msg.text.data[8] = (char)redBG;
    msg.text.data[9] = (char)(redBG >> 8);
    msg.text.data[10] = (char)greenBG;
    msg.text.data[11] = (char)(greenBG >> 8);
    msg.text.data[12] = (char)blueBG;
    msg.text.data[13] = (char)(blueBG >> 8);

    LCR_PrepWriteCmd(&msg, TPG_COLOR);

    return LCR_SendMsg(&msg);
}

int LCR_GetTPGColor(unsigned short *pRedFG, unsigned short *pGreenFG, unsigned short *pBlueFG, unsigned short *pRedBG, unsigned short *pGreenBG, unsigned short *pBlueBG)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(TPG_COLOR);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pRedFG = msg.text.data[0] | msg.text.data[1] << 8;
        *pGreenFG = msg.text.data[2] | msg.text.data[3] << 8;
        *pBlueFG = msg.text.data[4] | msg.text.data[5] << 8;
        *pRedBG = msg.text.data[6] | msg.text.data[7] << 8;
        *pGreenBG = msg.text.data[8] | msg.text.data[9] << 8;
        *pBlueBG = msg.text.data[10] | msg.text.data[11] << 8;

        return 0;
    }
    return -1;
}

int LCR_ClearPatLut(void)
{
    PatLutIndex = 0;
    return 0;
}

int LCR_AddToPatLut(int TrigType, int PatNum,int BitDepth,int LEDSelect,bool InvertPat, bool InsertBlack,bool BufSwap, bool trigOutPrev)
{
    unsigned int lutWord = 0;

    lutWord = TrigType & 3;
    if(PatNum > 24)
        return -1;

    lutWord |= ((PatNum & 0x3F) << 2);
    if( (BitDepth > 8) || (BitDepth <= 0))
        return -1;
    lutWord |= ((BitDepth & 0xF) << 8);
    if(LEDSelect > 7)
        return -1;
    lutWord |= ((LEDSelect & 0x7) << 12);
    if(InvertPat)
        lutWord |= BIT16;
    if(InsertBlack)
        lutWord |= BIT17;
    if(BufSwap)
        lutWord |= BIT18;
    if(trigOutPrev)
        lutWord |= BIT19;

    PatLut[PatLutIndex++] = lutWord;
    return 0;
}

int LCR_GetPatLutItem(int index, int *pTrigType, int *pPatNum,int *pBitDepth,int *pLEDSelect,bool *pInvertPat, bool *pInsertBlack,bool *pBufSwap, bool *pTrigOutPrev)
{
    unsigned int lutWord;

    lutWord = PatLut[index];

    *pTrigType = lutWord & 3;
    *pPatNum = (lutWord >> 2) & 0x3F;
    *pBitDepth = (lutWord >> 8) & 0xF;
    *pLEDSelect = (lutWord >> 12) & 7;
    *pInvertPat = ((lutWord & BIT16) == BIT16);
    *pInsertBlack = ((lutWord & BIT17) == BIT17);
    *pBufSwap = ((lutWord & BIT18) == BIT18);
    *pTrigOutPrev = ((lutWord & BIT19) == BIT19);

    return 0;
}

int LCR_OpenMailbox(int MboxNum)
{
    hidMessageStruct msg;

    msg.text.data[2] = MboxNum;
    LCR_PrepWriteCmd(&msg, MBOX_CONTROL);

    return LCR_SendMsg(&msg);
}

int LCR_CloseMailbox(void)
{
    hidMessageStruct msg;

    msg.text.data[2] = 0;
    LCR_PrepWriteCmd(&msg, MBOX_CONTROL);

    return LCR_SendMsg(&msg);
}

int LCR_MailboxSetAddr(int Addr)
{
    hidMessageStruct msg;

    if(Addr > 127)
        return -1;

    msg.text.data[2] = Addr;
    LCR_PrepWriteCmd(&msg, MBOX_ADDRESS);

    return LCR_SendMsg(&msg);
}

int LCR_SendPatLut(void)
{
    hidMessageStruct msg;
    int bytesToSend=PatLutIndex*3;
    unsigned int i;

    if(LCR_OpenMailbox(2) < 0)
        return -1;
    LCR_MailboxSetAddr(0);

    CmdList[MBOX_DATA].len = bytesToSend;
    LCR_PrepWriteCmd(&msg, MBOX_DATA);

    for(i=0; i<PatLutIndex; i++)
    {
        msg.text.data[2+3*i] = PatLut[i];
        msg.text.data[2+3*i+1] = PatLut[i]>>8;
        msg.text.data[2+3*i+2] = PatLut[i]>>16;
    }

    LCR_SendMsg(&msg);
    LCR_CloseMailbox();
    return 0;
}

int LCR_SendSplashLut(unsigned char *lutEntries, unsigned int numEntries)
{
    hidMessageStruct msg;
    unsigned int i;

    if(numEntries < 1 || numEntries > 64)
        return -1;

    LCR_OpenMailbox(1);
    LCR_MailboxSetAddr(0);

    // Check for special case of 2 entries
    if( numEntries == 2)
    {
         msg.text.data[2+0] = lutEntries[1];
         msg.text.data[2+1] = lutEntries[0];
    }
    else
    {
        for(i=0; i < numEntries; i++)
        {
            msg.text.data[2+i] = lutEntries[i];
        }
    }

    CmdList[MBOX_DATA].len = numEntries;
    LCR_PrepWriteCmd(&msg, MBOX_DATA);
    LCR_SendMsg(&msg);
    LCR_CloseMailbox();

    return 0;
}


int LCR_GetPatLut(int numEntries)
{
    hidMessageStruct msg;
    unsigned int lutWord = 0;
    int numBytes, i;
    unsigned char *readBuf;

    if(numEntries > 128)
        return -1;

    if(LCR_OpenMailbox(2) < 0)
        return -1;

    if(LCR_MailboxSetAddr(0) < 0)
        return -1;

    numBytes = sizeof(msg.head)+numEntries*3;
    readBuf = (unsigned char *)&msg;
    LCR_PrepReadCmd(MBOX_DATA);


    if(LCR_Read() > 0)
    {
        memcpy(readBuf, InputBuffer, MIN(numBytes,64));
        readBuf+=64;
        numBytes -=64;
    }
    else
    {
        LCR_CloseMailbox();
        return -1;
    }
    /* If packet is greater than 64 bytes, continue to read */
    while(numBytes > 0)
    {
        LCR_ContinueRead();
        memcpy(readBuf, InputBuffer, MIN(numBytes,64));
        readBuf+=64;
        numBytes -=64;
    }

    LCR_ClearPatLut();
    for(i=0; i<numEntries*3; i+=3)
    {
        lutWord = msg.text.data[i] | msg.text.data[i+1] << 8 | msg.text.data[i+2] << 16;
        PatLut[PatLutIndex++] = lutWord;
    }

    if(LCR_CloseMailbox() < 0)
        return -1;

    return 0;
}

int LCR_GetSplashLut(unsigned char *pLut, int numEntries)
{
    hidMessageStruct *pMsg;
    if(LCR_OpenMailbox(1) < 0)
        return -1;

    if(LCR_MailboxSetAddr(0) < 0)
        return -1;

    LCR_PrepReadCmd(MBOX_DATA);

    if(LCR_Read() > 0)
    {
        memcpy(pLut, InputBuffer+sizeof(pMsg->head), MIN(numEntries,64-sizeof(pMsg->head)));
        pLut+= (64-sizeof(pMsg->head));
        numEntries -= (64-sizeof(pMsg->head));
    }
    else
    {
        LCR_CloseMailbox();
        return -1;
    }

    /* If packet is greater than 64 bytes, continue to read */
    while(numEntries > 0)
    {
        LCR_ContinueRead();
        memcpy(pLut, InputBuffer, MIN(numEntries,64));
        pLut+=64;
        numEntries -= 64;
    }

    if(LCR_CloseMailbox() < 0)
        return -1;

    return 0;
}

int LCR_SetPatternTriggerMode(bool Vsync_or_Generated)
{
    hidMessageStruct msg;

    msg.text.data[2] = Vsync_or_Generated;
    LCR_PrepWriteCmd(&msg, PAT_TRIG_MODE);

    return LCR_SendMsg(&msg);
}

int LCR_GetPatternTriggerMode(bool *Vsync_or_Generated)
{    hidMessageStruct msg;

     LCR_PrepReadCmd(PAT_TRIG_MODE);

     if(LCR_Read() > 0)
     {
         memcpy(&msg, InputBuffer, 65);
         *Vsync_or_Generated = (msg.text.data[0] != 0);
         return 0;
     }
     return -1;
}


int LCR_PatternDisplay(int Action)
{
    hidMessageStruct msg;

    msg.text.data[2] = Action;
    LCR_PrepWriteCmd(&msg, PAT_START_STOP);

    return LCR_SendMsg(&msg);
}

int LCR_SetPatternConfig(unsigned int numLutEntries, bool repeat, unsigned int numPatsForTrigOut2, unsigned int numSplash)
{
    hidMessageStruct msg;

    msg.text.data[2] = numLutEntries-1; /* -1 because the firmware command takes 0-based indices (0 means 1) */
    msg.text.data[3] = repeat;
    msg.text.data[4] = numPatsForTrigOut2 - 1;  /* -1 because the firmware command takes 0-based indices (0 means 1) */
    msg.text.data[5] = numSplash - 1;   /* -1 because the firmware command takes 0-based indices (0 means 1) */
    LCR_PrepWriteCmd(&msg, PAT_CONFIG);

    return LCR_SendMsg(&msg);
}

int LCR_GetPatternConfig(unsigned int *pNumLutEntries, bool *pRepeat, unsigned int *pNumPatsForTrigOut2, unsigned int *pNumSplash)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(PAT_CONFIG);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pNumLutEntries = msg.text.data[0] + 1; /* +1 because the firmware gives 0-based indices (0 means 1) */
        *pRepeat = (msg.text.data[1] != 0);
        *pNumPatsForTrigOut2 = msg.text.data[2]+1;    /* +1 because the firmware gives 0-based indices (0 means 1) */
        *pNumSplash = msg.text.data[3]+1;     /* +1 because the firmware gives 0-based indices (0 means 1) */
        return 0;
    }
    return -1;
}

int LCR_SetExpsosure_FramePeriod(unsigned int exposurePeriod, unsigned int framePeriod)
{
        hidMessageStruct msg;

        msg.text.data[2] = exposurePeriod;
        msg.text.data[3] = exposurePeriod>>8;
        msg.text.data[4] = exposurePeriod>>16;
        msg.text.data[5] = exposurePeriod>>24;
        msg.text.data[6] = framePeriod;
        msg.text.data[7] = framePeriod>>8;
        msg.text.data[8] = framePeriod>>16;
        msg.text.data[9] = framePeriod>>24;
        LCR_PrepWriteCmd(&msg, PAT_EXPO_PRD);

        return LCR_SendMsg(&msg);
}

int LCR_GetExposure_FramePeriod(unsigned int *pExposure, unsigned int *pFramePeriod)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(PAT_EXPO_PRD);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pExposure = msg.text.data[0] | msg.text.data[1] << 8 | msg.text.data[2] << 16 | msg.text.data[3] << 24;
        *pFramePeriod = msg.text.data[4] | msg.text.data[5] << 8 | msg.text.data[6] << 16 | msg.text.data[7] << 24;
        return 0;
    }
    return -1;
}


int LCR_SetTrigOutConfig(unsigned int trigOutNum, bool invert, unsigned int rising, unsigned int falling)
{
    hidMessageStruct msg;

    msg.text.data[2] = invert;
    msg.text.data[3] = rising;
    msg.text.data[4] = falling;
    if(trigOutNum == 1)
        LCR_PrepWriteCmd(&msg, TRIG_OUT1_CTL);
    else if(trigOutNum==2)
        LCR_PrepWriteCmd(&msg, TRIG_OUT2_CTL);

    return LCR_SendMsg(&msg);
}

int LCR_GetTrigOutConfig(unsigned int trigOutNum, bool *pInvert,unsigned int *pRising, unsigned int *pFalling)
{
    hidMessageStruct msg;

    if(trigOutNum == 1)
        LCR_PrepReadCmd(TRIG_OUT1_CTL);
    else if(trigOutNum==2)
        LCR_PrepReadCmd(TRIG_OUT2_CTL);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pInvert = (msg.text.data[0] != 0);
        *pRising = msg.text.data[1];
        *pFalling = msg.text.data[2];
        return 0;
    }
    return -1;
}

int LCR_ValidatePatLutData(unsigned int *pStatus)
{
    hidMessageStruct msg;

    LCR_PrepWriteCmd(&msg, LUT_VALID);    
    if(LCR_SendMsg(&msg) < 0)
        return -1;

    LCR_PrepReadCmd(LUT_VALID);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pStatus = msg.text.data[0];
        return 0;
    }
    return -1;
}


int LCR_SetTrigIn1Delay(unsigned int Delay)
{
    hidMessageStruct msg;

    msg.text.data[2] = Delay;
    msg.text.data[3] = Delay >> 8;
    msg.text.data[4] = Delay >> 16;
    msg.text.data[5] = Delay >> 24;
    LCR_PrepWriteCmd(&msg, TRIG_IN1_DELAY);

    return LCR_SendMsg(&msg);
}

int LCR_GetTrigIn1Delay(unsigned int *pDelay)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(TRIG_IN1_DELAY);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pDelay = msg.text.data[0] | msg.text.data[1]<<8 | msg.text.data[2]<<16 | msg.text.data[3]<<24;
        return 0;
    }
    return -1;
}

int LCR_SetInvertData(bool invert)
{
    hidMessageStruct msg;

    msg.text.data[2] = invert;
    LCR_PrepWriteCmd(&msg, INVERT_DATA);

    return LCR_SendMsg(&msg);
}

int LCR_SetPWMConfig(unsigned int channel, unsigned int pulsePeriod, unsigned int dutyCycle)
{
    hidMessageStruct msg;

    msg.text.data[2] = channel;
    msg.text.data[3] = pulsePeriod;
    msg.text.data[4] = pulsePeriod >> 8;
    msg.text.data[5] = pulsePeriod >> 16;
    msg.text.data[6] = pulsePeriod >> 24;
    msg.text.data[7] = dutyCycle;

    LCR_PrepWriteCmd(&msg, PWM_SETUP);

    return LCR_SendMsg(&msg);
}

int LCR_GetPWMConfig(unsigned int channel, unsigned int *pPulsePeriod, unsigned int *pDutyCycle)
{
    hidMessageStruct msg;

    LCR_PrepReadCmdWithParam(PWM_SETUP, (unsigned char)channel);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pPulsePeriod = msg.text.data[1] | msg.text.data[2] << 8 | msg.text.data[3] << 16 | msg.text.data[4] << 24;
        *pDutyCycle = msg.text.data[5];
        return 0;
    }
    return -1;
}

int LCR_SetPWMEnable(unsigned int channel, bool Enable)
{
    hidMessageStruct msg;
    unsigned char value = 0;

    if(Enable)
        value = BIT7;

    if(channel == 2)
        value |= 2;
    else if (channel != 0)
        return -1;

    msg.text.data[2] = value;
    LCR_PrepWriteCmd(&msg, PWM_ENABLE);

    return LCR_SendMsg(&msg);
}

int LCR_GetPWMEnable(unsigned int channel, bool *pEnable)
{
    hidMessageStruct msg;

    LCR_PrepReadCmdWithParam(PWM_ENABLE, (unsigned char)channel);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);       
        if(msg.text.data[0] & BIT7)
            *pEnable =  true;
        else
            *pEnable = false;

        return 0;
    }
    return -1;
}

int LCR_SetPWMCaptureConfig(unsigned int channel, bool enable, unsigned int sampleRate)
{
    hidMessageStruct msg;
    unsigned char value = 0;

    value = channel & 1;

    if(enable)
        value |= BIT7;

    msg.text.data[2] = value;
    msg.text.data[3] = sampleRate;
    msg.text.data[4] = sampleRate >> 8;
    msg.text.data[5] = sampleRate >> 16;
    msg.text.data[6] = sampleRate >> 24;
    LCR_PrepWriteCmd(&msg, PWM_CAPTURE_CONFIG);

    return LCR_SendMsg(&msg);
}

int LCR_GetPWMCaptureConfig(unsigned int channel, bool *pEnabled, unsigned int *pSampleRate)
{
    hidMessageStruct msg;

    LCR_PrepReadCmdWithParam(PWM_CAPTURE_CONFIG, (unsigned char)channel);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        if(msg.text.data[0] & BIT7)
            *pEnabled =  true;
        else
            *pEnabled = false;

        *pSampleRate = msg.text.data[1] | msg.text.data[2] << 8 | msg.text.data[3] << 16 | msg.text.data[4] << 24;

        return 0;
    }
    return -1;
}

int LCR_PWMCaptureRead(unsigned int channel, unsigned int *pLowPeriod, unsigned int *pHighPeriod)
{
    hidMessageStruct msg;

    LCR_PrepReadCmdWithParam(PWM_CAPTURE_READ, (unsigned char)channel);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pLowPeriod = msg.text.data[1] | msg.text.data[2] << 8;
        *pHighPeriod = msg.text.data[3] | msg.text.data[4] << 8;
        return 0;
    }
    return -1;
}

int LCR_SetGPIOConfig(unsigned int pinNum, bool enAltFunc, bool altFunc1, bool dirOutput, bool outTypeOpenDrain, bool pinState)
{
    hidMessageStruct msg;
    unsigned char value = 0;

    if(enAltFunc)
        value |= BIT7;
    if(altFunc1)
        value |= BIT6;
    if(dirOutput)
        value |= BIT5;
    if(outTypeOpenDrain)
        value |= BIT4;
    if(pinState)
        value |= BIT3;

    msg.text.data[2] = pinNum;
    msg.text.data[3] = value;
    LCR_PrepWriteCmd(&msg, GPIO_CONFIG);

    return LCR_SendMsg(&msg);
}

int LCR_GetGPIOConfig(unsigned int pinNum, bool *pEnAltFunc, bool *pAltFunc1, bool *pDirOutput, bool *pOutTypeOpenDrain, bool *pState)
{
    hidMessageStruct msg;

    LCR_PrepReadCmdWithParam(GPIO_CONFIG, (unsigned char)pinNum);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pEnAltFunc = ((msg.text.data[1] & BIT7) == BIT7);
        *pAltFunc1 = ((msg.text.data[1] & BIT6) == BIT6);
        *pDirOutput = ((msg.text.data[1] & BIT5) == BIT5);
        *pOutTypeOpenDrain = ((msg.text.data[1] & BIT4) == BIT4);
        if(*pDirOutput)
            *pState = ((msg.text.data[1] & BIT3) == BIT3);
        else
            *pState = ((msg.text.data[1] & BIT2) == BIT2);
        return 0;
    }
    return -1;
}

int LCR_SetGeneralPurposeClockOutFreq(unsigned int clkId, bool enable, unsigned int clkDivider)
{
    hidMessageStruct msg;

    msg.text.data[2] = clkId;
    msg.text.data[3] = enable;
    msg.text.data[4] = clkDivider;
    LCR_PrepWriteCmd(&msg, GPCLK_CONFIG);

    return LCR_SendMsg(&msg);
}

int LCR_GetGeneralPurposeClockOutFreq(unsigned int clkId, bool *pEnabled, unsigned int *pClkDivider)
{
    hidMessageStruct msg;

    LCR_PrepReadCmdWithParam(GPCLK_CONFIG, (unsigned char)clkId);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pEnabled = (msg.text.data[0] != 0);
        *pClkDivider = msg.text.data[1];
        return 0;
    }
    return -1;
}

int LCR_SetPWMInvert(bool invert)
{
    hidMessageStruct msg;

    msg.text.data[2] = invert;
    LCR_PrepWriteCmd(&msg, PWM_INVERT);

    return LCR_SendMsg(&msg);
}

int LCR_GetPWMInvert(bool *inverted)
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(PWM_INVERT);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *inverted = (msg.text.data[0] != 0);
        return 0;
    }
    return -1;
}

int LCR_MemRead(unsigned int addr, unsigned int *readWord)
{
    hidMessageStruct msg;

    LCR_PrepMemReadCmd(addr);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *readWord = msg.text.data[0] | msg.text.data[1] << 8 | msg.text.data[2] << 16 | msg.text.data[3] << 24;
        //*readWord = msg.text.data[3] | msg.text.data[2] << 8 | msg.text.data[1] << 16 | msg.text.data[0] << 24; //MSB first
        return 0;
    }
    return -1;
}

int LCR_MemWrite(unsigned int addr, unsigned int data)
{
    hidMessageStruct msg;

    msg.text.data[2] = 0; //absolute write
    msg.text.data[3] = addr >> 24; //MSB first
    msg.text.data[4] = addr >> 16;
    msg.text.data[5] = addr >> 8;
    msg.text.data[6] = addr;
    msg.text.data[7] = data >> 24; //MSB first
    msg.text.data[8] = data >> 16;
    msg.text.data[9] = data >> 8;
    msg.text.data[10] = data;
    LCR_PrepWriteCmd(&msg, MEM_CONTROL);

    return LCR_SendMsg(&msg);
}

 int LCR_MeasureSplashLoadTiming(unsigned int startIndex, unsigned int numSplash)
 {
     hidMessageStruct msg;

     msg.text.data[2] = startIndex;
     msg.text.data[3] = numSplash;
     LCR_PrepWriteCmd(&msg, SPLASH_LOAD_TIMING);

     return LCR_SendMsg(&msg);
 }

 int LCR_ReadSplashLoadTiming(unsigned int *pTimingData)
 {
     hidMessageStruct msg;
     int i;

     LCR_PrepReadCmd(SPLASH_LOAD_TIMING);

     if(LCR_Read() > 0)
     {
         memcpy(&msg, InputBuffer, 65);
         for(i=0; i<12; i++)
         {
             *(pTimingData+i) = (msg.text.data[4*i+0] | msg.text.data[4*i+1] << 8 | msg.text.data[4*i+2] << 16 | msg.text.data[4*i+3] << 24);
         }
         return 0;
     }
     return -1;
 }
