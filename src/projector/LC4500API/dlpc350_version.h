/*
 * dlpc350_version.h
 *
 * This module has the version number info.
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

#ifndef VERSION_H
#define VERSION_H

#define GUI_VERSION_MAJOR 4
#define GUI_VERSION_MINOR 0
#define GUI_VERSION_BUILD 0

/* Version history
* 3.1.0 -
*         . Bug-fix:
*           . Play Once functionality not working correctly in Pattern display
*             mode
*           . Firmware download fails when attempting to download large binary
*             files
*           . GUI doesn't accept updated details from the FlashDeviceParameters.txt
*             file
*         . New feature: Fast flash update feature added in this new feature
*           GUI slectively erase and program the sectors which are changed between
*           previously programmed flash binary to the new flash binary file.
*           For this GUI stores copy of the last successful programmed binary as
*           cache file on the PC.
*
* 3.0.1 -
*         . Bug-fix: If the firmware file > 8MB the fimrware upload fails
*
* 3.0.0 -
*         . Variable Exposure Pattern Configuration support added
*         . Firmware Tagging feature added under Build Frimware option
*         . Firmware Tag Info displayed in the GUI
*         . Added option to select RGB Color or Single illumination system
*         . Fixed Issue in Create Images option for 5-bit and 6-bit pattern
*           stiching
*         . Fixed issue in the GUI related to storing images when RLE Compression
*           applied
*         . InvertPattern Option added while pattern selection
*         . Option added to read incoming Video singal info
*         . TabWidgets arranged in more meaningful way
*         . Source code is formatted with more meaninful variables and
*           functions name
*
* 2.0.0 -
*         . Added option to enable/disable 'Auto Status Update'; it is suggested
*           to disable when in Pattern Display Mode
*         . GUI Layout changed under Pattern Sequence/Sequence Settings,
*         . added explicit button for 'Validate Sequence'
*         . Appropriate Text put for 'Timing' section
*
* 1.2.0 -
*         . Added Mac and Linux support; GUI Layout improvements
*
* 1.1.0 -
*         . Added support for internal pattern storage & 2nd flash;
*           Check for v1.0.1;Added Peripherals tab;Layout and nomenclature
*           improvements
*
* 1.0.1 - Added capability to download firmware
*
* 1.0.0 - Initial release
*
*/

#endif //VERSION_H
