#Baysen Vision Inc. Structured Light project setup file
#Copyright Reserved. 2013-2016

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

project(slstudio)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost 1.47 COMPONENTS filesystem thread timer chrono REQUIRED)
find_package(Glew REQUIRED)
find_package(OpenGL REQUIRED)

if(VTK_QT_VERSION VERSION_GREATER "4")
    find_package(Qt5 COMPONENTS Core Gui Test Widgets OpenGL REQUIRED)
    # Instruct CMake to run moc automatically when needed.
    set(CMAKE_AUTOMOC ON)
else(VTK_QT_VERSION VERSION_GREATER "4")
    #Legacy qt4 support
    find_package(Qt4 REQUIRED)
endif(VTK_QT_VERSION VERSION_GREATER "4")

if(OpenCV_VERSION VERSION_LESS "3")
    add_definitions(-DOPENCV2)
endif(OpenCV_VERSION VERSION_LESS "3")

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_BINARY_DIR} 
                    ${PCL_INCLUDE_DIRS} ${VTK_DIR} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} 
                    ${GLEW_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS} )
include(${VTK_USE_FILE})

include_directories(${PROJECT_SOURCE_DIR}/src/camera)
include_directories(${PROJECT_SOURCE_DIR}/src/projector)
include_directories(${PROJECT_SOURCE_DIR}/src/codec)
include_directories(${PROJECT_SOURCE_DIR}/src/triangulator)
include_directories(${PROJECT_SOURCE_DIR}/src/calibrator)
include_directories(${PROJECT_SOURCE_DIR}/src/tracker)

add_definitions(${PCL_DEFINITIONS})

set(QT_USE_QTOPENGL TRUE)

#generate cmake config header file
configure_file(${PROJECT_SOURCE_DIR}/slconfig.h.in
        ${PROJECT_BINARY_DIR}/slconfig.h)

option(WITH_CAMERAXIMEA "Use Ximea camera" OFF)
option(WITH_CAMERAIIDC "Use IIDC camera" OFF)
option(WITH_CAMERAIDSIMAGING "Use IDS camera" OFF)
option(WITH_CAMERAPOINTGREY "Use Pointgrey camera" OFF)
option(WITH_CAMERAV4L "Use V4L camera" OFF)
option(WITH_CAMERAOPENCV "Use OpenCV camera" OFF)

option(WITH_LC3000API "Use LightCrafter 3000 projector" ON)
option(WITH_LC4500API "Use LightCrafter 4500 projector" ON)
option(WITH_PROJECTOROPENGL "Use OpenGL projector" ON)
option(WITH_PROJECTORQT "Use Qt projector" ON)

#Set source file set
file(GLOB UI_FILES src/SLStudio.ui 
        src/SLPreferenceDialog.ui 
        src/SLCalibrationDialog.ui 
        src/SLVideoDialog.ui 
        src/SLAboutDialog.ui 
        src/SLTrackerDialog.ui)

file(GLOB H_FILES ${PROJECT_BINARY_DIR}/cmakeconfig.h
        src/SLStudio.h
        src/SLVideoWidget.h 
        src/SLPreferenceDialog.h 
        src/SLCalibrationDialog.h 
        src/SLCameraVirtual.h 
        src/SLProjectorVirtual.h 
        src/SLScanWorker.h 
        src/SLDecoderWorker.h 
        src/SLTrackerWorker.h 
        src/SLVideoDialog.h 
        src/SLAboutDialog.h 
        src/SLPoseWidget.h 
        src/SLPointCloudWidget.h 
        src/SLTrackerDialog.h 
        src/SLTriangulatorWorker.h 
        src/SLTraceWidget.h 
        src/camera/Camera.h 
        src/tracker/Tracker.h 
        src/tracker/TrackerICP.h 
        src/tracker/TrackerNDT.h 
        src/tracker/CorrRejectOrgBoundFast.h 
        src/tracker/CorrEstOrgProjFast.h 
        src/tracker/CorrEstKdTreeFast.h 
        src/tracker/TrackerPCL.h 
        src/tracker/PoseFilter.h 
        src/cvtools.h)

file(GLOB CPP_FILES src/main.cpp 
        src/SLStudio.cpp 
        src/SLScanWorker.cpp 
        src/SLDecoderWorker.cpp 
        src/SLTrackerWorker.cpp 
        src/SLVideoWidget.cpp 
        src/SLPreferenceDialog.cpp 
        src/SLCalibrationDialog.cpp 
        src/SLCameraVirtual.cpp 
        src/SLProjectorVirtual.cpp 
        src/SLVideoDialog.cpp 
        src/SLAboutDialog.cpp 
        src/SLPoseWidget.cpp 
        src/SLPointCloudWidget.cpp 
        src/SLTrackerDialog.cpp 
        src/SLTriangulatorWorker.cpp 
        src/SLTraceWidget.cpp 
        src/camera/Camera.cpp 
        src/cvtools.cpp 
        src/tracker/TrackerICP.cpp 
        src/tracker/TrackerNDT.cpp 
        src/tracker/CorrRejectOrgBoundFast.cpp 
        src/tracker/TrackerPCL.cpp 
        src/tracker/PoseFilter.cpp)

file(GLOB RC_FILES src/SLResources.qrc)

set(PROJECTOR_DEVICE_LIBS)
set(CAMERA_DEVICE_LIBS)
set(PROJECTOR_DEVICE_DEFS)
set(CAMERA_DEVICE_DEFS)
#Device Options - camera
add_subdirectory(${PROJECT_SOURCE_DIR}/src/camera)
#Device Options - projector
add_subdirectory(${PROJECT_SOURCE_DIR}/src/projector)

#Algorithms library
add_subdirectory(${PROJECT_SOURCE_DIR}/src)

add_definitions(${CAMERA_DEVICE_DEFS})
add_definitions(${PROJECTOR_DEVICE_DEFS})

if(VTK_QT_VERSION VERSION_GREATER "4")
    qt5_wrap_ui(UISrcs ${UI_FILES})
    # CMAKE_AUTOMOC in ON so the MocHdrs will be automatically wrapped.
    qt5_add_resources(RCC_SRCS ${RC_FILES})
    add_executable(slstudio ${CPP_FILES} ${UISrcs} ${H_FILES} ${RCC_SRCS})
    qt5_use_modules(slstudio Core Gui Widgets Test)
    add_definitions(${COMPILER_DEFINES})
    target_link_libraries(slstudio slalgorithms ${VTK_LIBRARIES} ${PCL_LIBRARIES}
            ${OpenCV_LIBS} ${Boost_LIBRARIES} ${CAMERA_DEVICE_LIBS}
            ${PROJECTOR_DEVICE_LIBS})
else(VTK_QT_VERSION VERSION_GREATER "4")
    #Legacy qt4 support
    include(${QT_USE_FILE})

    qt4_wrap_ui(UISrcs ${UI_FILES})
    qt4_wrap_cpp(MOCSrcs ${H_FILES})
    qt4_add_resources(RCC_SRCS ${RC_FILES})

    add_executable(slstudio ${CPP_FILES} ${UISrcs} ${MOCSrcs} ${RCC_SRCS})
    target_link_libraries(slstudio slalgorithms Qt4::QtTest ${VTK_LIBRARIES} ${PCL_LIBRARIES}
            ${OpenCV_LIBS} ${Boost_LIBRARIES} ${CAMERA_DEVICE_LIBS} 
            ${PROJECTOR_DEVICE_LIBS} )
endif(VTK_QT_VERSION VERSION_GREATER "4")

if(MSVC)
    set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "/wd4819 /wd4996" )
endif(MSVC)


