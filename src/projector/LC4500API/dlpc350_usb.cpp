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

#include "dlpc350_usb.h"
#include <hidapi.h>
#include <cstring>
#include <mutex>

namespace {
hid_device *device = nullptr;
std::mutex enumerationMutex;
}

unsigned char g_OutputBuffer[USB_MAX_PACKET_SIZE + 1]{};
unsigned char g_InputBuffer[USB_MAX_PACKET_SIZE + 1]{};

int DLPC350_USB_Init() { std::lock_guard<std::mutex> lock(enumerationMutex); return hid_init(); }
int DLPC350_USB_Exit() { std::lock_guard<std::mutex> lock(enumerationMutex); return hid_exit(); }
int DLPC350_USB_IsConnected() { return device != nullptr; }

int DLPC350_USB_IsPresent() {
  std::lock_guard<std::mutex> lock(enumerationMutex);
  auto *devices = hid_enumerate(MY_VID, MY_PID);
  bool present = false;
  for (auto *entry = devices; entry; entry = entry->next)
    if (entry->interface_number == 0 || entry->interface_number == -1) present = true;
  hid_free_enumeration(devices);
  return present;
}

int DLPC350_USB_Close() {
  if (device) hid_close(device);
  device = nullptr;
  return 0;
}

int DLPC350_USB_Open() {
  std::lock_guard<std::mutex> lock(enumerationMutex);
  if (device) return 0;
  auto *devices = hid_enumerate(MY_VID, MY_PID);
  for (auto *entry = devices; entry && !device; entry = entry->next)
    if (entry->interface_number == 0 || entry->interface_number == -1)
      device = hid_open_path(entry->path);
  hid_free_enumeration(devices);
  return device ? 0 : -1;
}

int DLPC350_USB_Write() {
  if (!device) return -1;
  const auto written = hid_write(device, g_OutputBuffer, USB_MIN_PACKET_SIZE + 1);
  if (written != USB_MIN_PACKET_SIZE + 1) {
    DLPC350_USB_Close();
    return -1;
  }
  return written;
}

int DLPC350_USB_Read() {
  if (!device) return -1;
  std::memset(g_InputBuffer, 0, sizeof(g_InputBuffer));
  const auto received = hid_read_timeout(device, g_InputBuffer, USB_MIN_PACKET_SIZE + 1, 2000);
  if (received <= 0) {
    DLPC350_USB_Close();
    return -1;
  }
  return received;
}
