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
 * @file    tcp_client.h
 *
 * @brief	This file contatins functions related to TCP communication. Based on
 *			on the platform and development environment functions can be implemented.
 **/
/*****************************************************************************/

#ifndef TCP_CLIENT_H_
#define TCP_CLIENT_H_

#ifdef __cplusplus
extern "C" {
#endif

#define TRUE                    1
#define FALSE                   0


    int TCP_Connect(char *host, unsigned long int  port);
    int TCP_Send(int sock,  unsigned char *buffer, int length);
    int TCP_Receive(int sock, unsigned char *buffer, int length);
    int TCP_Disconnect(int sock);

#ifdef __cplusplus
}
#endif


#endif /* TCP_CLIENT_H_ */

