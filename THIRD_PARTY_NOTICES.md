# Third-party notices

SLStudio uses third-party software under its respective licenses. SLStudio's
[LICENSE](LICENSE), including its commercial licensing option, does not replace
or change those licenses.

The components present depend on the platform, enabled backends and dependency
versions. This document is an index and includes the notices for the TI code
shipped in the source tree. The dependency-specific notices accompanying a binary
package provide additional copyright holders, license texts and attribution.
An entry here does not mean that the component is included in every package.

## Components

| Component | Use | License |
| --- | --- | --- |
| [OpenCV 4](https://opencv.org/license/) | Image processing, calibration and camera capture | Apache-2.0 in 4.5.0 and later; earlier releases use BSD-3-Clause. Individual files and bundled dependencies have additional notices. |
| [Eigen 3](https://libeigen.gitlab.io/) | Linear algebra | Primarily MPL-2.0; some files have alternative or additional licenses. See the notices for the installed version. |
| [Dear ImGui](https://github.com/ocornut/imgui) | Desktop user interface and its embedded default font | MIT; see the bundled ImGui license, including its font attribution. |
| [GLFW](https://www.glfw.org/license.html) | Desktop windows and input | Zlib |
| [GLEW](https://glew.sourceforge.net/credits.html) | OpenGL function loading | BSD-style and MIT-style terms; see its complete notices. |
| [Aravis](https://github.com/AravisProject/aravis) | Optional USB3 Vision / GigE Vision camera backend | LGPL-2.1-or-later |
| [libdc1394](https://github.com/indigo-astronomy/libdc1394) | Optional IIDC USB / FireWire camera backend | LGPL-2.1-or-later |
| [GLib](https://docs.gtk.org/glib/) | Aravis dependency | LGPL-2.1-or-later, with additional file-specific notices |
| [libusb](https://libusb.info/) | USB camera transport | LGPL-2.1-or-later |
| [HIDAPI](https://github.com/libusb/hidapi) | Optional LightCrafter 4500 USB transport | BSD-3-Clause, GPL-3.0 or HIDAPI's original license, at the recipient's option. SLStudio's fetched HIDAPI build uses the BSD option. |
| Texas Instruments LightCrafter 3000 API sample code 2.0 | Optional DM365 network control | TI BSD, reproduced below; the original software manifest is retained in the source tree and installed when this backend is enabled. |
| Texas Instruments LightCrafter 4500 API | Optional DLPC350 USB control, pattern upload and flash utilities | TI BSD, reproduced below |
| [ROS 2](https://docs.ros.org/en/jazzy/) and [Fast DDS](https://github.com/eProsima/Fast-DDS) | Optional point-cloud publishing and its middleware | Many core components use Apache-2.0; other ROS packages and transitive dependencies use their own licenses. See the package-specific notices. |
| Linux display, USB and runtime libraries | Optional Wayland/XRandR/DRM output and transitive dependencies | Component-specific MIT, BSD, LGPL and other licenses; see the package-specific notices. |
| [AppImage type-2 runtime](https://github.com/AppImage/type2-runtime) | Linux portable executable launcher | MIT for the runtime itself, plus separately licensed statically linked components including musl, libfuse, squashfuse, zstd and zlib. |
| Microsoft Visual C++ runtime / GCC runtime libraries | Platform-specific runtime dependencies, where bundled | Microsoft redistribution terms / GPL with the GCC Runtime Library Exception, respectively. See the toolchain's terms and bundled notices. |

OpenCV builds can bring additional image/video codecs, numerical libraries and
other dependencies. The ROS distribution and the platform package manager can
also change the transitive dependency set. The table is not an exhaustive
inventory of every file in a portable package.

XIMEA xiAPI and FLIR/Teledyne Spinnaker are optional proprietary SDKs. They are not
included in the CI portable packages. Redistributing a custom build with these
SDKs requires following the manufacturer's applicable license and redistribution
instructions.

## Where package notices are installed

- **Linux AppImage:** `usr/share/licenses/SLStudio/` inside the extracted
  AppImage. `debian/` contains the collected Debian/Ubuntu package copyright
  files and `debian/common-licenses/` contains the shared license texts they
  reference. `build-packages.tsv` records installed build-environment package
  versions; it is not an exact inventory of bundled libraries.
- **Windows ZIP:** `share/licenses/SLStudio/` in the extracted application
  folder. `vcpkg/` contains dependency notices from the pinned vcpkg installation.
- **macOS app:** `SLStudio.app/Contents/Resources/licenses/`.
  `homebrew/` contains collected Homebrew notice files and `packages.json`
  records the installed formula metadata, not an exact bundle inventory.
- **Source installs:** normally `<prefix>/share/licenses/SLStudio/`, or the
  macOS app location above. CMake installs this document, SLStudio's license,
  applicable fetched ImGui/HIDAPI notices and the LC3000 manifest when enabled.
  System dependency notices remain the responsibility of the system packages
  unless the portable packaging scripts collect them.

The standalone Aravis build helper additionally installs its exact source
archive and license files under `aravis/` in the package's license directory.
This does not apply to Aravis supplied through vcpkg or another system package.

## Texas Instruments API notices

### LightCrafter 3000

The original `src/projector/LC3000API/v2.0 DLP LightCrafter API Sample Code Manifest.htm`
identifies the Linux API sample code, version 2.0, as **TI BSD**, and supplies
the license below. This accompanies the original copyright/proprietary headers
retained in the individual files.

Copyright (C) {2012} Texas Instruments Incorporated - http://www.ti.com/

Individual source files also carry:
(c) Copyright, Texas Instruments Incorporated, 2012-2013. All Rights Reserved.

### LightCrafter 4500

Copyright (C) {2015} Texas Instruments Incorporated - http://www.ti.com/

### TI BSD license terms

The following terms apply to the TI API components identified above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the
  distribution.
* Neither the name of Texas Instruments Incorporated nor the names of
  its contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Source availability and redistribution

SLStudio source and build/packaging scripts are available at
<https://github.com/jakobwilm/slstudio>. Use the source revision matching the
binary package, together with its dependency versions and build instructions.
Upstream project links in this document are references, not a written source
offer or a substitute for providing required corresponding source.

When redistributing binaries, retain all applicable notices, license texts and
attribution files. GPL/LGPL components can require corresponding source and
build materials; LGPL components can also require permitting replacement or
relinking and reverse engineering for debugging modifications. MPL-covered
files have their own source-availability requirements. Dynamic linking alone
does not remove these obligations.

The notice collectors and Aravis source archive do not by themselves provide
complete corresponding source for SLStudio and every bundled dependency.
Before publishing or redistributing a package, check its actual contents
(including the AppImage runtime's static dependencies), license notices and
version-matched source-delivery obligations. This document does not assert
that notice collection alone establishes complete license compliance.
