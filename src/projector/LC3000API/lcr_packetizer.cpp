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
* @file    lcr_packetizer.c
*
* @brief	This file prepares and decodes the packet data for transaction with
*			with DM365 over TCP link
**/
/*****************************************************************************/


#include "common.h"
#include "error.h"
#include "tcp_client.h"
#include "lcr_packetizer.h"


static uint8 packetBuffer[HEADER_SIZE + MAX_PACKET_SIZE + CHECKSUM_SIZE];
static uint8 * const packetData = packetBuffer + HEADER_SIZE;
static uint8 LCR_PacketType;
static uint8 contFlag;
static uint8 recvFlag;
static uint16 commandId;
static uint16 dataLength;
static uint16 parseIndex;
static int LCR_PKT_Socket;

static uint8 LCR_CMD_PKT_CalcChecksum(void);



/**********************************************************/
/* Low level functions for transfer of data over TCP link */
/**********************************************************/

/* Connects to the DLP LightCrafter(TM) Hardware */
ErrorCode_t LCR_CMD_PKT_ConnectToLCR(void)
{
	/* Open tcp connection with LCr */
	LCR_PKT_Socket = TCP_Connect(LCR_CMD_IP, LCR_CMD_PORT);

	if(LCR_PKT_Socket < 0)
		return FAIL;

	return SUCCESS;
}

/*Close the TCP Connection*/
ErrorCode_t LCR_CMD_PKT_DisconnectLCR(void)
{
	TCP_Disconnect(LCR_PKT_Socket);
	return SUCCESS;
}

/**********************************************************/
/* Low level functions for transfer of data over TCP link */
/**********************************************************/

int LCR_CMD_ReadData(uint8 *data, uint32 size)
{
	return TCP_Receive(LCR_PKT_Socket, data, size);
}

int LCR_CMD_WriteData(uint8 *data, uint32 size)
{
	return TCP_Send(LCR_PKT_Socket, data, size);
}

/*****************************/
/* Packet Creation functions */
/*****************************/

/* Initialize the read/write command */
int LCR_CMD_PKT_CommandInit(LCR_CommandType_t cmdType, uint16 cmdId)
{
	if(cmdType == LCR_CMD_PKT_TYPE_WRITE)
		LCR_PacketType = PKT_TYPE_WRITE;
	else if(cmdType == LCR_CMD_PKT_TYPE_READ)
		LCR_PacketType = PKT_TYPE_READ;
	else
		return -1;

	contFlag = 0;
	commandId = cmdId;
	dataLength = 0;

	return 0;
}

/* Add raw data to the command */
int LCR_CMD_PKT_PutData(uint8 *data, unsigned long int size)
{
	if(data == NULL)
		return -1;

	while(size)
	{
		int  copySize = size;
		if(dataLength == MAX_PACKET_SIZE)
		{
			if(LCR_CMD_PKT_SendPacket(1))
				return -1;
			dataLength = 0;
		}

		if(dataLength + copySize > MAX_PACKET_SIZE)
		{
			copySize = MAX_PACKET_SIZE - dataLength;
		}

		memcpy(packetData + dataLength, data, copySize);
		dataLength += copySize;
		size -= copySize;
	}

	return 0;
}

/* Add the initiger data to the command */
int LCR_CMD_PKT_PutInt(uint32 value, unsigned int size)
{
	uint8 data[4];
	unsigned int i;

	if(size > 4)
		return -1;

	for(i = 0; i < size; i++)
	{
		data[i] = (value & 0xFF);
		value >>= 8;
	}

	if(LCR_CMD_PKT_PutData(data, size))
		return -1;

	return 0;
}

/* Add the file content to the command */
int LCR_CMD_PKT_PutFile(char const *fileName)
{
	int error = 0;
	FILE *fp;
	int copySize;

	fp = fopen(fileName, "rb");
	if(fp == NULL)
		return -1;

	while(!feof(fp))
	{
		if(dataLength == MAX_PACKET_SIZE)
		{
			if(LCR_CMD_PKT_SendPacket(1))
			{
				error = -1;
				break;
			}
		}

		copySize = MAX_PACKET_SIZE - dataLength;

		copySize = fread(packetData + dataLength,
			1, copySize, fp);
		if(copySize <= 0)
			break;

		dataLength += copySize;
	}

	fclose(fp);

	return error;
}

/*Copied returened data into the <fileName>*/
int LCR_CMD_PKT_GetFile(char const *fileName,uint32 size)
{
	FILE *fp;
	int ret = 0;
	uint32 remSize = size;

	if(packetBuffer[0] != PKT_TYPE_READ_RESP)
	{
		return -1;
	}

	fp = fopen(fileName, "wb");
	if(fp == NULL)
	{
		return -1;
	}

	if(remSize == 0)
		remSize = MAX_PACKET_SIZE;

	while(remSize)
	{
		unsigned int copySize = dataLength - parseIndex;

		if(copySize == 0)
		{
			if(LCR_CMD_PKT_ReceivePacket(0))
			{
				if(size != 0)
					ret = -1;
				break;
			}

			copySize = dataLength - parseIndex;
		}

		if(copySize >= remSize)
		{
			copySize = remSize;
		}

		if(fwrite(packetData + parseIndex, 1, copySize, fp) != copySize)
		{
			ret = -1;
			break;
		}

		parseIndex += copySize;

		if(size != 0)
			remSize -= copySize;
	}

	fclose(fp);

	return ret;
}


/* Get raw data from the received packet */
int LCR_CMD_PKT_GetData(uint8 *data, unsigned long int size)
{
	if(packetBuffer[0] != PKT_TYPE_READ_RESP)
	{
		return -1;
	}

	while(size)
	{
		unsigned int copySize = dataLength - parseIndex;

		if(copySize == 0)
		{
			if(LCR_CMD_PKT_ReceivePacket(0))
				return -1;

			copySize = dataLength - parseIndex;
		}

		if(copySize >= size)
		{
			copySize = size;
		}

		memcpy(data, packetData + parseIndex, copySize);
		parseIndex += copySize;
		size -= copySize;
		data += copySize;
	}

	return 0;
}

/* Get integer data from the received packet */
uint32 LCR_CMD_PKT_GetInt(unsigned int size)
{
	uint32 value = 0;
	uint8 data[4];
	unsigned int i;

	if(size > 4)
		return 0;

	if(LCR_CMD_PKT_GetData(data, size))
		return 0;

	for(i = 0; i < size; i++)
	{
		value |= data[i] << (i * 8);
	}

	return value;
}

/* Calculate the packet checksum to be added at the end */
uint8 LCR_CMD_PKT_CalcChecksum(void)
{
	int i;
	int sum = 0;

	for(i = 0; i < dataLength + HEADER_SIZE; i++)
	{
		sum += packetBuffer[i];
	}
	return (uint8)(sum & 0xFF);
}

/***********************************************************/
/* Sending and receiving the Packets after packet creation */
/***********************************************************/

/* Receive one packet of the responce */
int LCR_CMD_PKT_ReceivePacket(BOOL firstPkt)
{
	unsigned long int mask;
	int i;
	
	dataLength = 0;
	parseIndex = 0;

	if(firstPkt == 0)
	{
		if(recvFlag == 0 ||  recvFlag == 3)
		{
			return -1;
		}
	}

	if(LCR_CMD_ReadData(packetBuffer, HEADER_SIZE))
	{
		return -1;
	}

	dataLength = packetBuffer[4] | packetBuffer[5] << 8;

	if(LCR_CMD_ReadData(packetData, dataLength + 1))
	{
		return -1;
	}

	if(packetData[dataLength] != LCR_CMD_PKT_CalcChecksum())
	{
		printf("ERROR: Checksum failed in the commands response packet!!!\n");
		return -1;
	}

	if(packetBuffer[0] != LCR_PacketType + 1)
	{
		//Determine the response packet 
		switch(packetBuffer[0])
		{

		case 0x00:
			printf("INFO: Command NOT executed; DM365 is BUSY\n");
			break;

		case 0x01:
			printf("INFO: Command packet has ERROR in it\n");
			
			//Set flag based on the error #
			mask = 0; 
			i = 0;
			while(dataLength--)
			{
				if(packetBuffer[(6+i)] != 0)
				{
					mask |= (1<<(packetBuffer[(6+i)] - 1));
				}

				i++;
			}

			//print each error 
			if((mask & 0x01) == 0x01)
			{
				printf("COMMAND PACKET ERROR: Commmand Failed!!!\n");
			}
			else if((mask & 0x02) == 0x02)
			{
				printf("COMMAND PACKET ERROR: Unsupported Command!!!\n");
			}
			else if((mask & 0x04) == 0x04)
			{
				printf("COMMAND PACKET ERROR: Invalid Parameter!!!\n");
			}
			else if((mask & 0x08) == 0x08)
			{
				printf("COMMAND PACKET ERROR: Out Of Resource!!!\n");
			}
			else if((mask & 0x10) == 0x10)
			{
				printf("COMMAND PACKET ERROR: Device Failed!!!\n");
			}
			else if((mask & 0x20) == 0x20)
			{
				printf("COMMAND PACKET ERROR: Device Busy!!!\n");
			}
			else if((mask & 0x40) == 0x40)
			{
				printf("COMMAND PACKET ERROR: Not Initialized!!!\n");
			}
			else if((mask & 0x80) == 0x80)
			{
				printf("COMMAND PACKET ERROR: Not Found!!!\n");
			}
			else if((mask & 0x100) == 0x100)
			{
				printf("COMMAND PACKET ERROR: Checksum Error!!!\n");
			}
			else if((mask & 0x200) == 0x200)
			{
				printf("COMMAND PACKET ERROR: Packet Format Error!!!\n");
			}
			else if((mask & 0x400) == 0x400)
			{
				printf("COMMAND PACKET ERROR: Command Continuation Error!!!\n");
			}
			else
			{
				printf("COMMAND PACKET ERROR: DM365 returned undocumented Error!!!\n");
			}

			break;

		case 0x02:
		case 0x04:
			printf("COMMAND PACKET ERROR: DM365 sent back command type as Write = [0x02] or Read = [0x04] command!!!\n");	
			break;

		default: 
			printf("ERROR: Unkknown packet type information received in response packet!!!\n");
			break;
		}

		return -1;
	}

	recvFlag = packetBuffer[3];

	if(firstPkt != (recvFlag == 0 || recvFlag == 1))
	{
		return -1;
	}

	if(recvFlag == 3)
	{
		/* Command SUCCESS */
	}

	return 0;
}

/* Send the command paket just initialized */
int LCR_CMD_PKT_SendPacket(BOOL more)
{
	uint8 flag;

	packetBuffer[0] = LCR_PacketType;
	packetBuffer[1] = (commandId >> 8) & 0xFF;
	packetBuffer[2] = (commandId) & 0xFF;

	if(contFlag)
	{
		if(more)
			flag = 2;
		else
			flag = 3;
	}
	else
	{
		if(more)
			flag = 1;
		else
			flag = 0;
	}
	contFlag = more;

	packetBuffer[3] = flag;
	packetBuffer[4] = dataLength & 0xFF;
	packetBuffer[5] = (dataLength >> 8) & 0xFF;

	packetData[dataLength] = LCR_CMD_PKT_CalcChecksum();

	int length = dataLength + HEADER_SIZE + CHECKSUM_SIZE;

	if(LCR_CMD_WriteData(packetBuffer, length))
		return -1;

	if(LCR_CMD_PKT_ReceivePacket(1))
		return -1;

	if(more == 0 && recvFlag == 0)
	{
		/* SUCCESS */
	}

	return 0;
}

int LCR_CMD_PKT_SendCommand(void)
{
	return LCR_CMD_PKT_SendPacket(0);
}
