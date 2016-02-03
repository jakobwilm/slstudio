#Baysen Vision Inc. Structured Light project setup file
#Copyright Reserved. 2013-2016

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(slstudio)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost 1.47 COMPONENTS filesystem thread chrono REQUIRED)
find_package(Glew REQUIRED)
find_package(OpenGL REQUIRED)


include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_BINARY_DIR} )
include_directories(${PCL_INCLUDE_DIRS} ${VTK_DIR} ${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${QT_INCLUDE_DIR})
include_directories(${GLEW_INCLUDE_DIRS})
include_directories( ${OPENGL_INCLUDE_DIRS} )
include(${VTK_USE_FILE})

include_directories(${PROJECT_SOURCE_DIR}/src/camera)
include_directories(${PROJECT_SOURCE_DIR}/src/projector)
include_directories(${PROJECT_SOURCE_DIR}/src/codec)
include_directories(${PROJECT_SOURCE_DIR}/src/triangulator)
include_directories(${PROJECT_SOURCE_DIR}/src/calibrator)
include_directories(${PROJECT_SOURCE_DIR}/src/tracker)

add_definitions(${PCL_DEFINITIONS})

set(QT_USE_QTOPENGL TRUE)
#Set C++ 11 on gcc
SET(CMAKE_CXX_FLAGS "-std=c++0x")

#generate cmake config header file
configure_file(${PROJECT_SOURCE_DIR}/slconfig.h.in
        ${PROJECT_BINARY_DIR}/slconfig.h)

option(WITH_CAMERAXIMEA "Use Ximea Camera" ON)
option(WITH_CAMERAIIDC "Use IIDC Camera" OFF)
option(WITH_CAMERAIDSIMAGING "Use IDS Camera" OFF)
option(WITH_CAMERAPOINTGREY "Use Pointgrey Camera" OFF)
option(WITH_LC3000API "Use LightCrafter 3000 projector" OFF)
option(WITH_LC4500API "Use LightCrafter 4500 projector" ON)


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
        src/projector/Projector.h 
        src/projector/ProjectorOpenGL.h 
        src/projector/OpenGLContext.h 
        src/projector/ProjectorLC3000.h 
        src/projector/ProjectorLC4500.h 
        src/codec/Codec.h 
        src/codec/phaseunwrap.h 
        src/codec/phasecorr.h 
        src/codec/pstools.h 
        src/codec/CodecCalibration.h 
        src/codec/CodecPhaseShift2x3.h 
        src/codec/CodecPhaseShiftDescatter.h 
        src/codec/CodecPhaseShift3.h 
        src/codec/CodecPhaseShift3FastWrap.h 
        src/codec/CodecPhaseShift3Unwrap.h 
        src/codec/CodecPhaseShift4.h 
        src/codec/CodecGrayCode.h 
        src/codec/CodecPhaseShift2p1.h 
        src/codec/CodecFastRatio.h 
        src/codec/CodecPhaseShiftModulated.h 
        src/codec/CodecPhaseShiftMicro.h 
        src/codec/CodecPhaseShiftNStep.h 
        src/triangulator/Triangulator.h 
        src/calibrator/CalibrationData.h 
        src/calibrator/Calibrator.h 
        src/calibrator/CalibratorLocHom.h 
        src/calibrator/CalibratorRBF.h 
        src/calibrator/CThinPlateSpline.h 
        src/calibrator/RBFInterpolator.h 
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
        src/projector/ProjectorOpenGL.cpp 
        src/codec/phaseunwrap.cpp 
        src/codec/phasecorr.cpp 
        src/codec/CodecCalibration.cpp 
        src/codec/CodecPhaseShift2x3.cpp 
        src/codec/CodecPhaseShiftDescatter.cpp 
        src/codec/CodecPhaseShift3.cpp 
        src/codec/CodecPhaseShift3FastWrap.cpp 
        src/codec/CodecPhaseShift3Unwrap.cpp 
        src/codec/CodecPhaseShift4.cpp 
        src/codec/CodecFastRatio.cpp 
        src/codec/CodecPhaseShift2p1.cpp 
        src/codec/CodecPhaseShiftModulated.cpp 
        src/codec/CodecPhaseShiftMicro.cpp 
        src/codec/CodecGrayCode.cpp 
        src/codec/pstools.cpp 
        src/codec/CodecPhaseShiftNStep.cpp 
        src/triangulator/Triangulator.cpp 
        src/calibrator/CalibrationData.cpp 
        src/calibrator/CalibratorLocHom.cpp 
        src/calibrator/CalibratorRBF.cpp 
        src/calibrator/CThinPlateSpline.cpp 
        src/calibrator/RBFInterpolator.cpp 
        src/cvtools.cpp 
        src/tracker/TrackerICP.cpp 
        src/tracker/TrackerNDT.cpp 
        src/tracker/CorrRejectOrgBoundFast.cpp 
        src/tracker/TrackerPCL.cpp 
        src/tracker/PoseFilter.cpp)

file(GLOB RC_FILES src/SLResources.qrc)
# Compile with system dependent OpenGL Context code
if(APPLE)
    list(APPEND CPP_FILES src/projector/OpenGLContext.Mac.mm)
elseif(WIN32)
    list(APPEND CPP_FILES src/projector/OpenGLContext.Win.cpp)
else()
    list(APPEND CPP_FILES src/projector/OpenGLContext.Unix.cpp)
endif()
#Device Options - camera
add_subdirectory(${PROJECT_SOURCE_DIR}/src/camera)
#Device Options - projector
add_subdirectory(${PROJECT_SOURCE_DIR}/src/projector)

if(WITH_CAMERAXIMEA)
    set(DEVICE_LIBS ${DEVICE_LIBS} XimeaCamera)
    #add_definitions(-DWITH_CAMERAXIMEA)
endif(WITH_CAMERAXIMEA)

if(WITH_LC4500API)
    set(DEVICE_LIBS ${DEVICE_LIBS} LC4500Projector)
    #add_definitions(-DWITH_LC4500API)
endif(WITH_LC4500API)

if(WITH_LC3000API)
    set(DEVICE_LIBS ${DEVICE_LIBS} LC3000Projector)
    #add_definitions(-DWITH_LC3000API)
endif(WITH_LC3000API)

#Use QT4
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

qt4_wrap_ui(UISrcs ${UI_FILES})
qt4_wrap_cpp(MOCSrcs ${H_FILES})
qt4_add_resources(RCC_SRCS ${RC_FILES})

add_executable(slstudio ${CPP_FILES} ${UISrcs} ${MOCSrcs} ${RCC_SRCS})


if(VTK_VERSION  VERSION_GREATER "6")
    target_link_libraries(slstudio ${VTK_LIBRARIES} ${PCL_LIBRARIES}
            ${OpenCV_LIBS} ${Boost_LIBRARIES} ${GLEW_LIBRARIES}
            ${OPENGL_LIBRARIES} ${DEVICE_LIBS})
else(VTK_VERSION  VERSION_GREATER "6")
    target_link_libraries(slstudio QVTK ${PCL_LIBRARIES} ${OpenCV_LIBS}
            ${Boost_LIBRARIES} ${GLEW_LIBRARIES} ${OPENGL_LIBRARIES} ${DEVICE_LIBS})
endif(VTK_VERSION  VERSION_GREATER "6")

message(status " :QT Inlcude : ${QT_INCLUDE_DIR}")
message(status " :OpenGL Libs : ${OPENGL_LIBRARIES} ")
message(status " :Device Libs : ${DEVICE_LIBS} ")


