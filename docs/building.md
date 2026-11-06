# Building from Source

## Requirements

SLStudio requires CMake 3.21 or newer, a C++17 compiler, OpenCV 4 or 5, and Eigen 3.3 or newer. The desktop application also uses GLFW, GLEW, OpenGL 3.3 Core, and Dear ImGui; CMake downloads the pinned ImGui version.

Optional camera backends, ROS 2 publishing, and projector integrations require additional dependencies. XIMEA and Spinnaker camera support require their respective manufacturer SDKs. Aravis supports compatible USB3 Vision and GigE Vision cameras. Available integrations are reported during CMake configuration.

## Linux

On Ubuntu 24.04 or newer, install the desktop build dependencies:

```sh
sudo apt install cmake git g++ pkg-config libopencv-dev libeigen3-dev \
  libglfw3-dev libglew-dev libaravis-dev libudev-dev libdrm-dev \
  libwayland-dev wayland-protocols
```

Configure, build, and run tests with the desktop preset:

```sh
cmake --preset desktop
cmake --build --preset desktop -j4
ctest --preset desktop
./build/desktop/SLStudio
```

Optional X11 DRM leasing also requires `libxrandr-dev`, `libx11-xcb-dev`, and `libxcb-randr0-dev`; install them and rerun CMake configuration.

Automatic GNOME display reservation requires GIO (`libglib2.0-dev` on Ubuntu). CMake enables it when available; without it, DRM lease outputs must already be offered by the compositor.

If the desktop's HiDPI scale is not detected, set `SLSTUDIO_UI_SCALE` when starting the application, for example `SLSTUDIO_UI_SCALE=2 ./build/desktop/SLStudio`. Values from 0.5 to 4 override automatic UI scaling; projector output pixels are unchanged.

## Windows

Install Visual Studio 2022 with **Desktop development with C++**, CMake, and Git. From a **Developer PowerShell for VS 2022**, run these commands in the source directory:

```powershell
git clone https://github.com/microsoft/vcpkg.git .vcpkg
$baseline = (Get-Content vcpkg.json -Raw | ConvertFrom-Json).'builtin-baseline'
git -C .vcpkg checkout $baseline
.\.vcpkg\bootstrap-vcpkg.bat -disableMetrics
cmake --preset desktop -G "Visual Studio 17 2022" -A x64 "-DCMAKE_TOOLCHAIN_FILE=$PWD/.vcpkg/scripts/buildsystems/vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows
cmake --build --preset desktop --config Release --parallel 4
ctest --preset desktop -C Release
.\build\desktop\Release\SLStudio.exe
```

The first build installs and compiles the dependencies declared in [vcpkg.json](../vcpkg.json) and can take some time.

## macOS

These commands match the current macOS CI configuration and assume Homebrew OpenCV 5. Run them from the repository root. Install Apple's command-line tools if they are not already available:

```sh
xcode-select --install
```

Install Homebrew if needed, then install the build dependencies:

```sh
brew install cmake gcc meson ninja pkgconf glib libxml2 libusb opencv eigen@3 glfw glew
```

Build the shared Aravis camera backend used by the macOS configuration:

```sh
cmake "-DARAVIS_DEPENDENCY_PREFIXES=$(brew --prefix);$(brew --prefix libxml2)" \
  -P cmake/build-aravis.cmake
```

Configure the desktop preset. CMake detects optional integrations from the installed dependencies:

```sh
cmake --preset desktop -G Ninja \
  -DOpenCV_DIR="$(brew --prefix opencv)/lib/cmake/opencv5" \
  -DEigen3_DIR="$(brew --prefix eigen@3)/share/eigen3/cmake" \
  -DCMAKE_PREFIX_PATH="$(brew --prefix);$(brew --prefix gcc)"
```

Build, test, and launch SLStudio:

```sh
cmake --build --preset desktop --parallel 4
ctest --preset desktop --timeout 180
./build/desktop/SLStudio
```

The OpenCV CMake path above follows the current Homebrew formula used in CI. If Homebrew changes its OpenCV CMake directory, update `OpenCV_DIR` to the installed formula's path. For the CI configuration and packaging workflow, see the [build workflow](../.github/workflows/build.yml).