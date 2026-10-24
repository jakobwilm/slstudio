cmake_minimum_required(VERSION 3.21)
if(NOT DEFINED OUTPUT_DIR)
    message(FATAL_ERROR "Pass -DOUTPUT_DIR=...")
endif()
file(MAKE_DIRECTORY "${OUTPUT_DIR}")
function(download name url sha256)
    set(path "${OUTPUT_DIR}/${name}")
    if(EXISTS "${path}")
        file(SHA256 "${path}" actual)
        if(NOT actual STREQUAL sha256)
            message(FATAL_ERROR "Cached packaging tool checksum mismatch: ${path}")
        endif()
    else()
        file(DOWNLOAD "${url}" "${path}" EXPECTED_HASH "SHA256=${sha256}"
            TLS_VERIFY ON TIMEOUT 180 INACTIVITY_TIMEOUT 30 SHOW_PROGRESS)
    endif()
    file(CHMOD "${path}" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
endfunction()
download(linuxdeploy.AppImage
    "https://github.com/linuxdeploy/linuxdeploy/releases/download/1-alpha-20251107-1/linuxdeploy-x86_64.AppImage"
    c20cd71e3a4e3b80c3483cef793cda3f4e990aca14014d23c544ca3ce1270b4d)
download(appimagetool.AppImage
    "https://github.com/AppImage/appimagetool/releases/download/1.9.1/appimagetool-x86_64.AppImage"
    ed4ce84f0d9caff66f50bcca6ff6f35aae54ce8135408b3fa33abfc3cb384eb0)
# Pin the runtime as well: appimagetool otherwise downloads an unpinned runtime.
download(runtime-x86_64
    "https://github.com/AppImage/type2-runtime/releases/download/continuous/runtime-x86_64"
    617aa77314d57c8b8e4fae9c3f0b35af7b30d5529cb69e98f64f92510ed98a46)
