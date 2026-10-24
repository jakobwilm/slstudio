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
 * @file    lcr_packetizer.h
 *
 * @brief	This file is responsible for packetizing the data the functions
 *          defined in this file are called by lcr_cmd file
 **/
/*****************************************************************************/

#ifndef LCR_PACKETIZER_H_
#define LCR_PACKETIZER_H_

#include "common.h"
#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HEADER_SIZE			6
#define MAX_PACKET_SIZE		0xFFFF
#define CHECKSUM_SIZE		1

#define LCR_CMD_IP			"192.168.1.100"
#define LCR_CMD_PORT		0x5555

    enum LCR_PacketType {
        PKT_TYPE_BUSY,
        PKT_TYPE_ERROR,
        PKT_TYPE_WRITE,
        PKT_TYPE_WRITE_RESP,
        PKT_TYPE_READ,
        PKT_TYPE_READ_RESP
    };

    typedef enum
    {
        LCR_CMD_PKT_TYPE_READ,
        LCR_CMD_PKT_TYPE_WRITE,
    } LCR_CommandType_t;

    ErrorCode_t LCR_CMD_PKT_ConnectToLCR(void);
    ErrorCode_t LCR_CMD_PKT_DisconnectLCR(void);

    int LCR_CMD_PKT_CommandInit(LCR_CommandType_t cmdType, uint16 cmd);
    int LCR_CMD_PKT_PutData(uint8 *data, unsigned long int size);
    int LCR_CMD_PKT_PutInt(uint32 value, unsigned int size);
    int LCR_CMD_PKT_GetData(uint8 *data, unsigned long int size);
    uint32 LCR_CMD_PKT_GetInt(unsigned int size);
    int LCR_CMD_PKT_PutFile(char const *fileName);
    int LCR_CMD_PKT_GetFile(char const *fileName,uint32 size);
    int LCR_CMD_PKT_SendCommand(void);

    int LCR_CMD_PKT_ReceivePacket(BOOL firstPkt);
    int LCR_CMD_PKT_SendPacket(BOOL more);

#ifdef __cplusplus
}
#endif

#endif /* LCR_PACKETIZER_H_ */

