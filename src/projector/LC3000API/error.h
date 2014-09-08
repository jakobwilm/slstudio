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
 * @file    Error.h
 *
 * @brief	Errors returned from the DM365 Commands
 **/
/*****************************************************************************/

#ifndef ERROR_H_
#define ERROR_H_

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


#endif /* ERROR_H_ */
