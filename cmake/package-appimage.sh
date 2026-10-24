#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 3 ]]; then
    echo "Usage: $0 BUILD_DIR OUTPUT_DIR ROS_PREFIX" >&2
    exit 1
fi
build=$(realpath "$1")
mkdir -p "$2"
output=$(realpath "$2")
ros=$(realpath "$3")
source_dir=$(cd "$(dirname "$0")/.." && pwd)
appdir="$build/SLStudio.AppDir"
tools="$build/appimage-tools"
if [[ -e "$appdir" ]]; then
    echo "AppDir already exists: $appdir; use a fresh build directory" >&2
    exit 1
fi
if ! grep -q '^SLSTUDIO_WITH_ROS2_ENABLED:INTERNAL=ON$' "$build/CMakeCache.txt"; then
    echo "AppImage packaging requires a build with ROS 2 output enabled." >&2
    exit 1
fi
test -f "$ros/lib/librmw_fastrtps_cpp.so"
test -d "$ros/share/ament_index/resource_index"
cmake -DOUTPUT_DIR="$tools" -P "$source_dir/cmake/download-appimage-tools.cmake"
DESTDIR="$appdir" cmake --install "$build" --component Runtime --prefix /usr
mkdir -p "$appdir/usr/lib" "$appdir/usr/share/ament_index"
cp -a "$ros/share/ament_index/resource_index" "$appdir/usr/share/ament_index/"
install -m755 "$source_dir/resources/AppRun" "$appdir/AppRun"

# ROS loads RMW and generated type-support libraries with dlopen; ldd alone misses them.
ros_libraries=()
while IFS= read -r -d '' library; do
    ros_libraries+=("$library")
done < <(find "$ros/lib" -maxdepth 1 -name '*.so*' -type f -print0)
export APPIMAGE_EXTRACT_AND_RUN=1
aravis=$(sed -n 's/^SLSTUDIO_ARAVIS_ROOT:PATH=//p' "$build/CMakeCache.txt")
export LD_LIBRARY_PATH="$ros/lib:$aravis/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ldd "$appdir/usr/bin/SLStudio" "$appdir/usr/bin/slstudio-reconstruct" "${ros_libraries[@]}" > "$build/appimage-dependencies.txt"
if grep -q 'not found' "$build/appimage-dependencies.txt"; then
    cat "$build/appimage-dependencies.txt" >&2
    exit 1
fi
libraries=()
# linuxdeploy's general-purpose exclusion list also omits USB/windowing libraries.
# Explicitly bundle those, leaving only libc, the C++ ABI and graphics drivers to the host.
while IFS= read -r library; do
    case "$(basename "$library")" in
        libc.so.*|libm.so.*|libpthread.so.*|libdl.so.*|librt.so.*|libresolv.so.*|libgcc_s.so.*|libstdc++.so.*|ld-linux*|libGL*|libEGL*|libOpenGL*|libdrm*|libgbm*|libvulkan*) continue ;;
    esac
    libraries+=(--library "$library")
done < <({ printf '%s\n' "${ros_libraries[@]}"; awk '$2 == "=>" && $3 ~ /^\// {print $3}' "$build/appimage-dependencies.txt"; } | sort -u)
"$tools/linuxdeploy.AppImage" --appdir "$appdir" "${libraries[@]}" \
    --desktop-file "$source_dir/resources/slstudio.desktop" \
    --icon-file "$source_dir/resources/slstudio.png"

licenses="$appdir/usr/share/licenses/SLStudio"
mkdir -p "$licenses/debian"
dpkg-query -W -f='${Package}\t${Version}\n' > "$licenses/build-packages.tsv"
while IFS= read -r -d '' notice; do
    package=$(basename "$(dirname "$notice")")
    mkdir -p "$licenses/debian/$package"
    cp -L "$notice" "$licenses/debian/$package/"
done < <(find /usr/share/doc -maxdepth 2 -name copyright \( -type f -o -type l \) -print0)
cp -Lr /usr/share/common-licenses "$licenses/debian/"
image="$output/SLStudio-linux-x86_64.AppImage"
ARCH=x86_64 "$tools/appimagetool.AppImage" --runtime-file "$tools/runtime-x86_64" "$appdir" "$image"
(cd "$output" && sha256sum "$(basename "$image")" > "$(basename "$image").sha256")
