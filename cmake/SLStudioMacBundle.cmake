set(iconset "${CMAKE_CURRENT_BINARY_DIR}/slstudio.iconset")
file(MAKE_DIRECTORY "${iconset}")
find_program(SIPS_EXECUTABLE sips REQUIRED)
find_program(ICONUTIL_EXECUTABLE iconutil REQUIRED)
foreach(size IN ITEMS 16 32 128 256 512)
    math(EXPR retina "${size} * 2")
    execute_process(COMMAND "${SIPS_EXECUTABLE}" -z ${size} ${size}
        "${CMAKE_CURRENT_SOURCE_DIR}/resources/slstudio.png" --out "${iconset}/icon_${size}x${size}.png"
        OUTPUT_QUIET COMMAND_ERROR_IS_FATAL ANY)
    execute_process(COMMAND "${SIPS_EXECUTABLE}" -z ${retina} ${retina}
        "${CMAKE_CURRENT_SOURCE_DIR}/resources/slstudio.png" --out "${iconset}/icon_${size}x${size}@2x.png"
        OUTPUT_QUIET COMMAND_ERROR_IS_FATAL ANY)
endforeach()
execute_process(COMMAND "${ICONUTIL_EXECUTABLE}" -c icns "${iconset}"
    -o "${CMAKE_CURRENT_BINARY_DIR}/slstudio.icns" COMMAND_ERROR_IS_FATAL ANY)
target_sources(SLStudio PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/slstudio.icns")
set_source_files_properties("${CMAKE_CURRENT_BINARY_DIR}/slstudio.icns" PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
set_target_properties(SLStudio PROPERTIES
    MACOSX_BUNDLE TRUE
    MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/resources/Info.plist.in"
    MACOSX_BUNDLE_GUI_IDENTIFIER "org.slstudio.SLStudio"
    MACOSX_BUNDLE_BUNDLE_NAME SLStudio
    MACOSX_BUNDLE_ICON_FILE slstudio.icns)
