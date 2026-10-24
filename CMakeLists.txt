cmake_minimum_required(VERSION 3.21)
project(SLStudio LANGUAGES C CXX)

include(CTest)
include(GNUInstallDirs)
include(cmake/SLStudioFeatures.cmake)
option(SLSTUDIO_BUILD_GUI "Build the GLFW/ImGui desktop application" ON)
option(SLSTUDIO_ENABLE_WARNINGS "Enable compiler warnings for SLStudio code" ON)
slstudio_feature(SLSTUDIO_WITH_ROS2 "Build native ROS 2 PointCloud2 output")
slstudio_feature(SLSTUDIO_CAMERA_ARAVIS "Build the Aravis shared-library camera backend")
slstudio_feature(SLSTUDIO_CAMERA_XIMEA "Build the XIMEA camera backend")
slstudio_feature(SLSTUDIO_CAMERA_SPINNAKER "Build the FLIR Spinnaker camera backend")
slstudio_feature(SLSTUDIO_PROJECTOR_LC4500 "Build the LightCrafter 4500 USB backend")
slstudio_feature(SLSTUDIO_WITH_DRM "Build Linux Wayland/XRandR DRM lease output")
option(SLSTUDIO_PACKAGE_RUNTIME "Bundle dependencies when installing the Runtime component" OFF)
set(SLSTUDIO_RUNTIME_DESTINATION "${CMAKE_INSTALL_BINDIR}")
if(WIN32)
    set(SLSTUDIO_RUNTIME_DESTINATION ".")
elseif(APPLE)
    set(SLSTUDIO_RUNTIME_DESTINATION "SLStudio.app/Contents/MacOS")
endif()
set(SLSTUDIO_SANITIZER "none" CACHE STRING "Runtime checks: none, address, or thread")
set_property(CACHE SLSTUDIO_SANITIZER PROPERTY STRINGS none address thread)
if(NOT SLSTUDIO_SANITIZER MATCHES "^(none|address|thread)$")
    message(FATAL_ERROR "SLSTUDIO_SANITIZER must be none, address, or thread")
endif()
if(NOT SLSTUDIO_SANITIZER STREQUAL "none" AND
   (MSVC OR NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang"))
    message(FATAL_ERROR "Sanitizer configurations require GCC or Clang")
endif()

function(slstudio_enable_checks target)
    if(SLSTUDIO_ENABLE_WARNINGS)
        if(MSVC)
            target_compile_options(${target} PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:/W4;/permissive->")
        else()
            target_compile_options(${target} PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic>")
        endif()
    endif()
    if(SLSTUDIO_SANITIZER STREQUAL "address")
        target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
        target_link_options(${target} PUBLIC -fsanitize=address,undefined)
    elseif(SLSTUDIO_SANITIZER STREQUAL "thread")
        target_compile_options(${target} PRIVATE -fsanitize=thread -fno-omit-frame-pointer)
        target_link_options(${target} PUBLIC -fsanitize=thread)
    endif()
endfunction()

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs calib3d)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

add_library(slstudio_core
    src/IntensityCalibrationLut.cpp
    src/GeometricCalibration.cpp
    src/RBFInterpolator.cpp
    src/FrameSequence.cpp
    src/DecoderWorker.cpp
    src/TriangulatorWorker.cpp
    src/CaptureWorker.cpp
    src/GeometricCalibrationData.cpp
    src/triangulator/Triangulator.cpp
    src/PointCloud.cpp
    src/Ros2Output.cpp
    src/codec/CodecFactory.cpp
    src/codec/CodecGrayCode.cpp
    src/codec/CodecPhaseShiftMotionCompensated.cpp
    src/codec/CodecPhaseShift.cpp
    src/codec/CodecPhaseShiftHeterodyne.cpp
    src/codec/CodecPhaseShiftSpatialUnwrap.cpp
    src/codec/CodecPhaseShiftMicro.cpp
    src/codec/CodecPhaseShiftModulated.cpp
    src/codec/spatialUnwrapping.cpp
    src/codec/pstools.cpp
    src/cvtools.cpp
)
add_library(SLStudio::Core ALIAS slstudio_core)
target_compile_features(slstudio_core PUBLIC cxx_std_17)
set_target_properties(slstudio_core PROPERTIES CXX_EXTENSIONS OFF)
set_target_properties(slstudio_core PROPERTIES EXPORT_NAME Core)
target_include_directories(slstudio_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/codec>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/triangulator>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/slstudio>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/slstudio/codec>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/slstudio/triangulator>
)
target_link_libraries(slstudio_core PUBLIC ${OpenCV_LIBS} Eigen3::Eigen Threads::Threads)
set(ros2_available OFF)
if(NOT SLSTUDIO_WITH_ROS2 STREQUAL "OFF")
    find_package(rclcpp QUIET)
    find_package(sensor_msgs QUIET)
    if(TARGET rclcpp::rclcpp AND TARGET sensor_msgs::sensor_msgs__rosidl_typesupport_cpp)
        set(ros2_available ON)
    endif()
endif()
slstudio_resolve_feature(SLSTUDIO_WITH_ROS2 "${ros2_available}" "rclcpp and sensor_msgs")
if(SLSTUDIO_WITH_ROS2_ENABLED)
    target_compile_definitions(slstudio_core PRIVATE SLSTUDIO_WITH_ROS2)
    target_link_libraries(slstudio_core PRIVATE
        rclcpp::rclcpp sensor_msgs::sensor_msgs__rosidl_typesupport_cpp)
endif()

option(SLSTUDIO_BUILD_CLI "Build the headless reconstruction tool" ON)
if(SLSTUDIO_BUILD_CLI)
    add_executable(slstudio-reconstruct src/cli/main.cpp)
    target_link_libraries(slstudio-reconstruct PRIVATE SLStudio::Core)
    install(TARGETS slstudio-reconstruct
        RUNTIME DESTINATION ${SLSTUDIO_RUNTIME_DESTINATION} COMPONENT Runtime)
endif()

install(TARGETS slstudio_core EXPORT SLStudioTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES src/IntensityCalibrationLut.h src/GeometricCalibration.h src/GeometricCalibrationData.h
    src/ReconstructionSettings.h
    src/FrameSequence.h
    src/DecoderWorker.h src/TriangulatorWorker.h
    src/CaptureWorker.h
    src/PointCloud.h src/RBFInterpolator.h src/cvtools.h
    src/Ros2Output.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/slstudio)
install(DIRECTORY src/codec src/triangulator
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/slstudio FILES_MATCHING PATTERN "*.h")
install(FILES src/projector/Projector.h src/projector/ProjectorFactory.h
    src/projector/patternTools.h
    src/projector/ProjectorLightCrafter3000.h src/projector/ProjectorLightCrafter4500.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/slstudio/projector)
install(EXPORT SLStudioTargets NAMESPACE SLStudio::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SLStudio)
include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/SLStudioConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/SLStudioConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SLStudio)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/SLStudioConfig.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SLStudio)

if(BUILD_TESTING)
    add_test(NAME cmake_features COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/cmake_features_tests.cmake")
    add_executable(slstudio_ros2_output_tests tests/ros2_output_tests.cpp)
    target_compile_definitions(slstudio_ros2_output_tests PRIVATE SLSTUDIO_ROS2_OUTPUT_TEST_MAIN)
    target_link_libraries(slstudio_ros2_output_tests PRIVATE SLStudio::Core)
    if(SLSTUDIO_WITH_ROS2_ENABLED)
        target_compile_definitions(slstudio_ros2_output_tests PRIVATE SLSTUDIO_WITH_ROS2)
        target_link_libraries(slstudio_ros2_output_tests PRIVATE
            rclcpp::rclcpp sensor_msgs::sensor_msgs__rosidl_typesupport_cpp)
    endif()
    add_test(NAME ros2_output COMMAND slstudio_ros2_output_tests)
    set_tests_properties(ros2_output PROPERTIES TIMEOUT 30)
    foreach(suite IN ITEMS codec geometric_calibration gray_code intensity_calibration pipeline point_cloud reconstruction)
        add_executable(slstudio_${suite}_tests tests/${suite}_tests.cpp)
        target_link_libraries(slstudio_${suite}_tests PRIVATE SLStudio::Core)
        target_compile_definitions(slstudio_${suite}_tests PRIVATE SLSTUDIO_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
        add_test(NAME ${suite} COMMAND slstudio_${suite}_tests)
    endforeach()
    add_executable(slstudio_persistence_tests tests/persistence_tests.cpp src/gui/ApplicationSettings.cpp)
    target_include_directories(slstudio_persistence_tests PRIVATE src/gui src/projector)
    target_link_libraries(slstudio_persistence_tests PRIVATE SLStudio::Core)
    target_compile_definitions(slstudio_persistence_tests PRIVATE SLSTUDIO_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME persistence COMMAND slstudio_persistence_tests)
    set_tests_properties(persistence PROPERTIES TIMEOUT 20)
    set_tests_properties(pipeline PROPERTIES TIMEOUT 45)
    add_executable(slstudio_projector_tests tests/projector_tests.cpp
        src/projector/patternTools.cpp src/projector/ProjectorFactory.cpp)
    target_include_directories(slstudio_projector_tests PRIVATE src/projector)
    target_link_libraries(slstudio_projector_tests PRIVATE SLStudio::Core)
    add_test(NAME projector COMMAND slstudio_projector_tests)
    add_executable(slstudio_acquisition_tests tests/acquisition_tests.cpp src/gui/ProjectorPresentationController.cpp)
    target_include_directories(slstudio_acquisition_tests PRIVATE src/gui src/projector)
    target_link_libraries(slstudio_acquisition_tests PRIVATE SLStudio::Core)
    add_test(NAME acquisition COMMAND slstudio_acquisition_tests)
    set_tests_properties(acquisition PROPERTIES TIMEOUT 30)
endif()

if(SLSTUDIO_BUILD_GUI)
    find_package(OpenCV 4 REQUIRED COMPONENTS videoio)
    include(FetchContent)
    FetchContent_Declare(imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.91.9b
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(imgui)
    add_library(slstudio_imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
    target_include_directories(slstudio_imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
    target_link_libraries(slstudio_imgui PRIVATE glfw OpenGL::GL ${CMAKE_DL_LIBS})
    target_compile_features(slstudio_imgui PRIVATE cxx_std_17)
    add_executable(SLStudio src/gui/main.cpp src/gui/ApplicationController.cpp src/gui/GuiRuntime.cpp
        src/gui/IntensityCalibrationDialog.cpp src/gui/MessageDialog.cpp
        src/gui/ProjectorPresentationController.cpp src/gui/ApplicationSettings.cpp src/gui/ApplicationWindows.cpp
        src/gui/ApplicationCapture.cpp src/gui/CloudRenderer.cpp src/gui/ApplicationMenu.cpp
        src/gui/CameraProjectorHeader.cpp src/gui/CameraPreviewTexture.cpp
        src/gui/RenderSettingsWindow.cpp src/gui/ExportPointCloudDialog.cpp
        src/gui/SettingsWindow.cpp src/gui/FilePickerDialog.cpp src/gui/FrameSequenceWindow.cpp
        src/gui/LightCrafter3000Window.cpp src/gui/LightCrafter4500Window.cpp
        src/gui/GeometricCalibrationDialog.cpp src/gui/IntensityCurveWindow.cpp
        src/gui/ProjectorPatternUploadDialog.cpp)
    if(BUILD_TESTING)
        target_sources(SLStudio PRIVATE tests/gui_smoke_tests.cpp tests/ros2_output_tests.cpp)
        target_include_directories(SLStudio PRIVATE src/gui tests)
        target_compile_definitions(SLStudio PRIVATE SLSTUDIO_GUI_SMOKE_TESTS)
        if(SLSTUDIO_WITH_ROS2_ENABLED)
            target_compile_definitions(SLStudio PRIVATE SLSTUDIO_WITH_ROS2)
            target_link_libraries(SLStudio PRIVATE
                rclcpp::rclcpp sensor_msgs::sensor_msgs__rosidl_typesupport_cpp)
        endif()
    endif()
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/resources/slstudio.png" SLSTUDIO_ICON_BYTES HEX)
    string(REGEX REPLACE "(..)" "0x\\1," SLSTUDIO_ICON_BYTES "${SLSTUDIO_ICON_BYTES}")
    configure_file(src/gui/AppIcon.h.in generated/AppIcon.h @ONLY)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/resources/slstudio.png")
    target_include_directories(SLStudio PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
    target_link_libraries(SLStudio PRIVATE SLStudio::Core SLStudio::Projector SLStudio::Camera
        slstudio_imgui glfw GLEW::GLEW OpenGL::GL opencv_videoio)
    target_compile_definitions(SLStudio PRIVATE SLSTUDIO_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    if(APPLE)
        include(cmake/SLStudioMacBundle.cmake)
    endif()
    install(TARGETS SLStudio
        BUNDLE DESTINATION . COMPONENT Runtime
        RUNTIME DESTINATION ${SLSTUDIO_RUNTIME_DESTINATION} COMPONENT Runtime)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        install(FILES resources/slstudio.desktop DESTINATION "${CMAKE_INSTALL_DATADIR}/applications" COMPONENT Runtime)
        install(FILES icon.svg DESTINATION "${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps" RENAME slstudio.svg COMPONENT Runtime)
        install(FILES resources/slstudio.png DESTINATION "${CMAKE_INSTALL_DATADIR}/icons/hicolor/256x256/apps" COMPONENT Runtime)
    endif()
endif()

add_library(slstudio_camera src/camera/CameraFactory.cpp)
add_library(SLStudio::Camera ALIAS slstudio_camera)
target_compile_features(slstudio_camera PUBLIC cxx_std_17)
target_include_directories(slstudio_camera PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/camera)
target_link_libraries(slstudio_camera PUBLIC opencv_core)
set(SLSTUDIO_ARAVIS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/build/dependencies/install" CACHE PATH "External shared Aravis prefix")
set(aravis_available OFF)
if(NOT SLSTUDIO_CAMERA_ARAVIS STREQUAL "OFF")
    list(PREPEND CMAKE_PREFIX_PATH "${SLSTUDIO_ARAVIS_ROOT}")
    if(WIN32)
        find_program(PKG_CONFIG_EXECUTABLE NAMES pkgconf pkg-config
            HINTS "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/tools/pkgconf"
                  "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/tools/pkg-config")
    endif()
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(ARAVIS QUIET IMPORTED_TARGET aravis-0.8>=0.8.31)
        if(TARGET PkgConfig::ARAVIS)
            set(aravis_available ON)
        endif()
    endif()
endif()
slstudio_resolve_feature(SLSTUDIO_CAMERA_ARAVIS "${aravis_available}" "Aravis >= 0.8.31 and pkg-config")
if(SLSTUDIO_CAMERA_ARAVIS_ENABLED)
    target_sources(slstudio_camera PRIVATE src/camera/CameraAravis.cpp)
    target_compile_definitions(slstudio_camera PRIVATE SLSTUDIO_CAMERA_ARAVIS)
    target_link_libraries(slstudio_camera PRIVATE PkgConfig::ARAVIS)
endif()
if(BUILD_TESTING)
    add_executable(slstudio_camera_tests tests/camera_tests.cpp)
    target_link_libraries(slstudio_camera_tests PRIVATE SLStudio::Camera)
    add_executable(slstudio_camera_diagnostics tests/camera_diagnostics.cpp)
    target_link_libraries(slstudio_camera_diagnostics PRIVATE SLStudio::Camera)
    add_test(NAME camera COMMAND slstudio_camera_tests)
    add_test(NAME camera_sdk COMMAND slstudio_camera_tests --sdk)
    set_tests_properties(camera_sdk PROPERTIES TIMEOUT 30)
    if(SLSTUDIO_CAMERA_ARAVIS_ENABLED)
        target_link_libraries(slstudio_camera_tests PRIVATE PkgConfig::ARAVIS)
        target_compile_definitions(slstudio_camera_tests PRIVATE SLSTUDIO_TEST_ARAVIS)
        add_test(NAME camera_aravis COMMAND slstudio_camera_tests --aravis)
        set_tests_properties(camera_aravis PROPERTIES TIMEOUT 30)
    endif()
endif()
foreach(driver IN ITEMS XIMEA SPINNAKER)
    set(sdk_available OFF)
    if(NOT SLSTUDIO_CAMERA_${driver} STREQUAL "OFF")
        if(driver STREQUAL "XIMEA")
            set(header xiApi.h)
            set(libraries m3api xiapi64 xiapi)
            set(source CameraXimea)
            set(roots /opt/XIMEA "$ENV{ProgramFiles}/XIMEA/API")
        else()
            set(header Spinnaker.h)
            set(libraries Spinnaker Spinnaker_v140)
            set(source CameraSpinnaker)
            set(roots /opt/spinnaker "$ENV{ProgramFiles}/Teledyne/Spinnaker" "$ENV{ProgramFiles}/FLIR Systems/Spinnaker")
        endif()
        find_path(${driver}_INCLUDE_DIR NAMES ${header} HINTS ${${driver}_ROOT} ${roots} PATH_SUFFIXES include)
        find_library(${driver}_LIBRARY NAMES ${libraries} HINTS ${${driver}_ROOT} ${roots} PATH_SUFFIXES lib lib64 lib64/vs2015)
        if(EXISTS "${${driver}_INCLUDE_DIR}/${header}" AND EXISTS "${${driver}_LIBRARY}")
            set(sdk_available ON)
        endif()
    endif()
    slstudio_resolve_feature(SLSTUDIO_CAMERA_${driver} "${sdk_available}" "the ${driver} SDK headers and library")
    if(SLSTUDIO_CAMERA_${driver}_ENABLED)
        target_sources(slstudio_camera PRIVATE src/camera/${source}.cpp)
        target_include_directories(slstudio_camera SYSTEM PRIVATE ${${driver}_INCLUDE_DIR})
        target_compile_definitions(slstudio_camera PRIVATE SLSTUDIO_CAMERA_${driver})
        target_link_libraries(slstudio_camera PRIVATE ${${driver}_LIBRARY})
    endif()
endforeach()

if(SLSTUDIO_BUILD_GUI)
    include(cmake/SLStudioPackaging.cmake)
endif()
option(SLSTUDIO_BUILD_PROJECTORS "Build projector output backends" ON)
if(SLSTUDIO_BUILD_GUI OR SLSTUDIO_BUILD_PROJECTORS)
    find_package(OpenGL REQUIRED)
    find_package(GLEW REQUIRED)
    find_package(glfw3 3.3 REQUIRED)
    add_library(slstudio_projector src/projector/ProjectorDesktop.cpp
        src/projector/ProjectorFactory.cpp src/projector/patternTools.cpp)
    target_compile_definitions(slstudio_projector PRIVATE SLSTUDIO_PROJECTOR_DESKTOP)
    add_library(SLStudio::Projector ALIAS slstudio_projector)
    target_compile_features(slstudio_projector PUBLIC cxx_std_17)
    target_include_directories(slstudio_projector PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/projector ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(slstudio_projector PUBLIC opencv_core PRIVATE opencv_imgproc glfw GLEW::GLEW OpenGL::GL)

    option(SLSTUDIO_PROJECTOR_LC3000 "Build the LightCrafter 3000 network backend" OFF)
    if(SLSTUDIO_PROJECTOR_LC3000)
        add_library(slstudio_lc3000_protocol STATIC
            src/projector/LC3000API/lcr_cmd.cpp src/projector/LC3000API/lcr_packetizer.cpp)
        target_sources(slstudio_projector PRIVATE src/projector/ProjectorLightCrafter3000.cpp)
        target_compile_definitions(slstudio_projector PRIVATE SLSTUDIO_PROJECTOR_LC3000)
        target_link_libraries(slstudio_projector PRIVATE slstudio_lc3000_protocol opencv_imgcodecs)
        if(WIN32)
            target_link_libraries(slstudio_projector PRIVATE ws2_32)
        endif()
    endif()

    if(NOT SLSTUDIO_PROJECTOR_LC4500 STREQUAL "OFF")
        include(cmake/SLStudioHidapi.cmake)
    endif()
    slstudio_resolve_feature(SLSTUDIO_PROJECTOR_LC4500 "${SLSTUDIO_HIDAPI_AVAILABLE}"
        "native HIDAPI >= 0.14 or its build prerequisites (Linux, Windows or macOS)")
    if(SLSTUDIO_PROJECTOR_LC4500_ENABLED)
        slstudio_require_hidapi()
        add_library(slstudio_lc4500_protocol STATIC
            src/projector/LC4500API/dlpc350_api.cpp
            src/projector/LC4500API/dlpc350_common.cpp
            src/projector/LC4500API/dlpc350_usb.cpp
            src/projector/LC4500API/dlpc350_firmware.cpp
            src/projector/LC4500API/dlpc350_BMPParser.cpp)
        target_link_libraries(slstudio_lc4500_protocol PRIVATE ${SLSTUDIO_HIDAPI_TARGET} Threads::Threads)
        if(BUILD_TESTING)
            add_executable(slstudio_lc4500_diagnostics tests/lc4500_diagnostics.cpp)
            target_include_directories(slstudio_lc4500_diagnostics PRIVATE src/projector)
            target_compile_features(slstudio_lc4500_diagnostics PRIVATE cxx_std_17)
            target_link_libraries(slstudio_lc4500_diagnostics PRIVATE slstudio_projector slstudio_lc4500_protocol
                opencv_core opencv_imgcodecs Threads::Threads)
            add_executable(slstudio_lc4500_tests tests/lc4500_tests.cpp
                src/projector/ProjectorLightCrafter4500.cpp src/projector/lc4500tools.cpp src/projector/ProjectorFactory.cpp
                src/projector/patternTools.cpp)
            target_include_directories(slstudio_lc4500_tests PRIVATE src/projector ${OpenCV_INCLUDE_DIRS})
            target_compile_features(slstudio_lc4500_tests PRIVATE cxx_std_17)
            target_compile_definitions(slstudio_lc4500_tests PRIVATE SLSTUDIO_PROJECTOR_LC4500)
            target_link_libraries(slstudio_lc4500_tests PRIVATE slstudio_lc4500_protocol
                opencv_core opencv_imgcodecs Threads::Threads)
            slstudio_enable_checks(slstudio_lc4500_tests)
            add_test(NAME lc4500 COMMAND slstudio_lc4500_tests)
        endif()
        target_sources(slstudio_projector PRIVATE src/projector/ProjectorLightCrafter4500.cpp src/projector/lc4500tools.cpp)
        target_compile_definitions(slstudio_projector PRIVATE SLSTUDIO_PROJECTOR_LC4500)
        target_link_libraries(slstudio_projector PRIVATE slstudio_lc4500_protocol opencv_imgcodecs)
    endif()

    set(drm_available OFF)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND NOT SLSTUDIO_WITH_DRM STREQUAL "OFF")
        find_package(PkgConfig QUIET)
        if(PkgConfig_FOUND)
            pkg_check_modules(DRM QUIET IMPORTED_TARGET libdrm)
            pkg_check_modules(WAYLAND QUIET IMPORTED_TARGET wayland-client)
            pkg_check_modules(XRANDR QUIET IMPORTED_TARGET xrandr x11-xcb xcb-randr)
            pkg_check_modules(WAYLAND_PROTOCOLS QUIET wayland-protocols)
            if(WAYLAND_PROTOCOLS_FOUND)
                pkg_get_variable(WAYLAND_PROTOCOLS_DIR wayland-protocols pkgdatadir)
            endif()
            find_program(WAYLAND_SCANNER wayland-scanner)
            set(LEASE_XML "${WAYLAND_PROTOCOLS_DIR}/staging/drm-lease/drm-lease-v1.xml")
            if(DRM_FOUND AND WAYLAND_FOUND AND WAYLAND_PROTOCOLS_FOUND AND
               EXISTS "${WAYLAND_SCANNER}" AND EXISTS "${LEASE_XML}")
                set(drm_available ON)
            endif()
        endif()
    endif()
    slstudio_resolve_feature(SLSTUDIO_WITH_DRM "${drm_available}"
        "Linux, libdrm, wayland-client, the DRM lease protocol XML and wayland-scanner")
    if(SLSTUDIO_WITH_DRM_ENABLED)
        add_custom_command(OUTPUT drm-lease-v1-client-protocol.h
            COMMAND ${WAYLAND_SCANNER} client-header ${LEASE_XML} drm-lease-v1-client-protocol.h
            DEPENDS ${LEASE_XML})
        add_custom_command(OUTPUT drm-lease-v1-protocol.c
            COMMAND ${WAYLAND_SCANNER} private-code ${LEASE_XML} drm-lease-v1-protocol.c
            DEPENDS ${LEASE_XML})
        target_sources(slstudio_projector PRIVATE src/projector/ProjectorLinux.cpp
            ${CMAKE_CURRENT_BINARY_DIR}/drm-lease-v1-client-protocol.h
            ${CMAKE_CURRENT_BINARY_DIR}/drm-lease-v1-protocol.c)
        target_include_directories(slstudio_projector PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
        target_compile_definitions(slstudio_projector PUBLIC SLSTUDIO_WITH_DRM)
        target_link_libraries(slstudio_projector PRIVATE PkgConfig::DRM PkgConfig::WAYLAND)
        if(XRANDR_FOUND)
            target_compile_definitions(slstudio_projector PRIVATE SLSTUDIO_WITH_XRANDR_LEASE)
            target_link_libraries(slstudio_projector PRIVATE PkgConfig::XRANDR)
        else()
            message(STATUS "XRandR leasing disabled: install xrandr, x11-xcb and xcb-randr development packages to enable")
        endif()
    endif()
else()
    slstudio_resolve_feature(SLSTUDIO_PROJECTOR_LC4500 OFF "projector backends to be built")
    slstudio_resolve_feature(SLSTUDIO_WITH_DRM OFF "projector backends to be built")
endif()

slstudio_feature_summary()

foreach(target IN ITEMS slstudio_core slstudio_camera slstudio_projector slstudio-reconstruct SLStudio
        slstudio_acquisition_tests slstudio_camera_tests slstudio_codec_tests slstudio_geometric_calibration_tests
        slstudio_gray_code_tests slstudio_intensity_calibration_tests slstudio_persistence_tests
        slstudio_pipeline_tests slstudio_point_cloud_tests slstudio_projector_tests slstudio_reconstruction_tests
        slstudio_ros2_output_tests
        slstudio_camera_diagnostics slstudio_lc4500_diagnostics)
    if(TARGET ${target})
        slstudio_enable_checks(${target})
    endif()
endforeach()