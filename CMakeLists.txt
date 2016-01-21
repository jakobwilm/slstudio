#Baysen Vision Inc. Structured Light project setup file
#Copyright Reserved. 2013-2016

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#Macros to create and main precompiled headers for Visual Studio
#MACRO(ADD_MSVC_PRECOMPILED_HEADER PrecompiledHeader PrecompiledSource SourcesVar)
#  IF(MSVC)
#    GET_FILENAME_COMPONENT(PrecompiledBasename ${PrecompiledHeader} NAME_WE)
#    SET(PrecompiledBinary "$(IntDir)/${PrecompiledBasename}.pch")
#    SET(Sources ${${SourcesVar}})
#
#    SET_SOURCE_FILES_PROPERTIES(${PrecompiledSource}
#                                PROPERTIES COMPILE_FLAGS "/Yc\"${PrecompiledHeader}\" /Fp\"${PrecompiledBinary}\""
#                                           OBJECT_OUTPUTS "${PrecompiledBinary}")
#    SET_SOURCE_FILES_PROPERTIES(${Sources}
#                                PROPERTIES COMPILE_FLAGS "/Yu\"${PrecompiledHeader}\" /FI\"${PrecompiledHeader}\" /Fp\"${PrecompiledBinary}\""
#                                           OBJECT_DEPENDS "${PrecompiledBinary}")  
#    # Add precompiled header to SourcesVar
#    LIST(APPEND ${SourcesVar} ${PrecompiledSource})
#  ENDIF(MSVC)
#ENDMACRO(ADD_MSVC_PRECOMPILED_HEADER)


project(slstudio)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost 1.47 COMPONENTS filesystem thread chrono REQUIRED)
find_package(Glew REQUIRED)
find_package(OpenGL REQUIRED)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_BINARY_DIR} )
include_directories(${PCL_INCLUDE_DIRS} ${VTK_DIR} ${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${QT_INCLUDE_DIR})
include_directories(${GLEW_INCLUDE_DIRS})
include_directories( ${OPENGL_INCLUDE_DIRS} )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/camera)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/projector)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/codec)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/triangulator)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/calibrator)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/tracker)






add_definitions(${PCL_DEFINITIONS})
include(${VTK_USE_FILE})


link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})




# make sure boost is dynamically linked
set(Boost_USE_STATIC_LIBS        OFF)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     OFF)
set(BOOST_ALL_DYN_LINK           ON)   # force dynamic linking for all libraries
set(QT_USE_QTOPENGL TRUE)

#Use cotire to handle precompile headers
#set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake")
#include(cotire)

#Set C++ 11 on gcc
SET(CMAKE_CXX_FLAGS "-std=c++0x")

add_definitions(-DQT_NO_DEBUG)


file(GLOB UI_FILES src/SLStudio.ui 
        src/SLPreferenceDialog.ui 
        src/SLCalibrationDialog.ui 
        src/SLVideoDialog.ui 
        src/SLAboutDialog.ui 
        src/SLTrackerDialog.ui)

file(GLOB H_FILES src/SLStudio.h 
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
        src/cvtools.h
        src/projector/LC4500API/API.h
        src/projector/LC3000API/lcr_cmd.h)
        
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
        src/tracker/PoseFilter.cpp
        src/projector/OpenGLContext.Mac.mm
        src/projector/ProjectorLC4500.cpp 
        src/projector/LC4500API/API.cpp 
        src/projector/LC4500API/usb.cpp
        src/projector/LC4500API/hid.Mac.c
        src/projector/ProjectorLC3000.cpp 
        src/projector/LC3000API/lcr_cmd.cpp 
        src/projector/LC3000API/lcr_packetizer.cpp
        src/projector/LC3000API/tcp_client.cpp)
       
file(GLOB RC_FILES src/SLResources.qrc)


#if(MSVC)
#    ADD_MSVC_PRECOMPILED_HEADER("stdafx.h" "stdafx.cpp" CPP_FILES)
#endif(MSVC)


#Legacy qt4 support
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

qt4_wrap_ui(UISrcs ${UI_FILES})
qt4_wrap_cpp(MOCSrcs ${H_FILES})
qt4_add_resources(RCC_SRCS ${RC_FILES})

add_executable(slstudio ${CPP_FILES} ${UISrcs} ${MOCSrcs} ${RCC_SRCS})    

#if(MSVC)
#    set_target_properties(slstudio PROPERTIES COMPILE_FLAGS "/Zm450")
#    #handle boost exception 
#	set_target_properties(slstudio PROPERTIES COMPILE_FLAGS "/EHsc")
#endif(MSVC)
    link_libraries()

if(VTK_VERSION  VERSION_GREATER "6")
    target_link_libraries(slstudio ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${GLEW_LIBRARIES} ${OPENGL_LIBRARIES})
else(VTK_VERSION  VERSION_GREATER "6")
    target_link_libraries(slstudio QVTK ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${GLEW_LIBRARIES} ${OPENGL_LIBRARIES})
endif(VTK_VERSION  VERSION_GREATER "6")

message(status " :PCL Include Dir: ${PCL_INCLUDE_DIRS}")
message(status " :PCL Library Dir: ${PCL_LIBRARY_DIRS}")
message(status " :PCL Libraries: ${PCL_LIBRARIES}")
message(status " :OpenCV Include Dir: ${OpenCV_INCLUDE_DIRS}")
message(status " :OpenCV Library Dir: ${OpenCV_LIBRARY_DIRS}")
message(status " :OpenCV Libraries: ${OpenCV_LIBS}")
message(status " :Boost Include: ${Boost_INCLUDE_DIR}")
message(status " :Boost Library Dir: ${Boost_LIBRARY_DIRS}")
message(status " :Boost Libraries: ${Boost_LIBRARIES}")
message(status " :VTK version: ${VTK_VERSION}")
message(status " :VTK QT version: ${VTK_QT_VERSION}")
message(status " :CMake source dir: ${CMAKE_CURRENT_SOURCE_DIR}")
message(status " :CMake binary dir: ${CMAKE_CURRENT_BINARY_DIR}")
message(status " :QT Inlcude : ${QT_INCLUDE_DIR}")
message(status " :OpenGL Libs : ${OPENGL_LIBRARIES} ")#${OPENGL_gl_LIBRARY}")


#cotire(BVBase)
