/*
 * dlpc350_usb.cpp
 *
 * This module has the wrapper functions to access USB driver functions.
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

#include <stdio.h>
#include <stdlib.h>
#include "dlpc350_usb.h"
#ifdef Q_OS_WIN32
#include <setupapi.h>
#endif
#include "hidapi.h"

//#include "mainwindow.h"
//#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QTimer>

/***************************************************
*                  GLOBAL VARIABLES
****************************************************/

static hid_device *DeviceHandle;	//Handle to write
//In/Out buffers equal to HID endpoint size + 1
//First byte is for Windows internal use and it is always 0
unsigned char g_OutputBuffer[USB_MAX_PACKET_SIZE+1];
unsigned char g_InputBuffer[USB_MAX_PACKET_SIZE+1];


static int USBConnected = 0;      //Boolean true when device is connected

int DLPC350_USB_IsConnected()
{
    return USBConnected;
}

int DLPC350_USB_Init(void)
{
    return hid_init();
}

int DLPC350_USB_Exit(void)
{
    return hid_exit();
}

int DLPC350_USB_Open()
{
    // Open the device using the VID, PID,
    // and optionally the Serial number.
    DeviceHandle = hid_open(MY_VID, MY_PID, NULL);

    if(DeviceHandle == NULL)
    {
        USBConnected = 0;
        return -1;
    }

    USBConnected = 1;

    return 0;
}

int DLPC350_USB_Write()
{
    int bytesWritten;

    if(DeviceHandle == NULL)
        return -1;

    if((bytesWritten = hid_write(DeviceHandle, g_OutputBuffer, USB_MIN_PACKET_SIZE+1)) == -1)
    {
        hid_close(DeviceHandle);
        USBConnected = 0;
        return -1;
    }

    return bytesWritten;
}

int DLPC350_USB_Read()
{
    int bytesRead;

    if(DeviceHandle == NULL)
        return -1;

    //clear out the input buffer
    memset((void*)&g_InputBuffer[0],0x00,USB_MIN_PACKET_SIZE+1);

    if((bytesRead = hid_read_timeout(DeviceHandle, g_InputBuffer, USB_MIN_PACKET_SIZE+1, 2000)) == -1)
    {
        hid_close(DeviceHandle);
        USBConnected = 0;
        return -1;
    }

    return bytesRead;
}

int DLPC350_USB_Close()
{
    hid_close(DeviceHandle);
    USBConnected = 0;

    return 0;
}

