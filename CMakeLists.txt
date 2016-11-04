#Baysen Vision Inc. Structured Light project setup file
#Copyright Reserved. 2013-2016

cmake_minimum_required(VERSION 2.8.6 FATAL_ERROR)
include(GenerateExportHeader)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

project(slstudio)
find_package(OpenCV REQUIRED)
if(OpenCV_VERSION VERSION_LESS "3")
    add_definitions(-DOPENCV2)
endif(OpenCV_VERSION VERSION_LESS "3")

find_package(Glew REQUIRED)
find_package(OpenGL REQUIRED)

find_package(PCL)
find_package(VTK)
find_package(Boost 1.47 COMPONENTS chrono filesystem system timer REQUIRED)

set(USE_QT5 true)
if(VTK_FOUND)
    if(VTK_QT_VERSION VERSION_LESS "5")
        set(USE_QT5 false)
    endif(VTK_QT_VERSION VERSION_LESS "5")
endif(VTK_FOUND)

if(${USE_QT5})
    find_package(Qt5 COMPONENTS Core Gui Test Widgets OpenGL)
    # Instruct CMake to run moc automatically when needed.
    set(CMAKE_AUTOMOC ON)
else()
    #Legacy qt4 support
    find_package(Qt4 REQUIRED QtCore QtGui QtTest QtOpenGL)
    include(${QT_USE_FILE})
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_BINARY_DIR} 
                    ${CMAKE_CURRENT_BINARY_DIR}/src ${CMAKE_CURRENT_BINARY_DIR}/src/camera
                    ${CMAKE_CURRENT_BINARY_DIR}/src/projector
                    ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} 
                    ${GLEW_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})

include_directories(${PROJECT_SOURCE_DIR}/src/camera)
include_directories(${PROJECT_SOURCE_DIR}/src/projector)
include_directories(${PROJECT_SOURCE_DIR}/src/codec)
include_directories(${PROJECT_SOURCE_DIR}/src/triangulator)
include_directories(${PROJECT_SOURCE_DIR}/src/calibrator)
include_directories(${PROJECT_SOURCE_DIR}/src/tracker)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/src/camera)
add_subdirectory(${PROJECT_SOURCE_DIR}/src/projector)

#generate cmake config header file
configure_file(${PROJECT_SOURCE_DIR}/slconfig.h.in
        ${PROJECT_BINARY_DIR}/slconfig.h)

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
        src/cvtools.cpp 
        src/tracker/TrackerICP.cpp 
        src/tracker/TrackerNDT.cpp 
        src/tracker/CorrRejectOrgBoundFast.cpp 
        src/tracker/TrackerPCL.cpp 
        src/tracker/PoseFilter.cpp)

file(GLOB RC_FILES src/SLResources.qrc)

if(VTK_FOUND AND PCL_FOUND)
    include(${VTK_USE_FILE})
    add_definitions(${PCL_DEFINITIONS})
    
    
    include_directories(${PCL_INCLUDE_DIRS} ${VTK_DIR})
    
    if(USE_QT5)
        qt5_wrap_ui(UISrcs ${UI_FILES})
        # CMAKE_AUTOMOC in ON so the MocHdrs will be automatically wrapped.
        qt5_add_resources(RCC_SRCS ${RC_FILES})
        add_executable(slstudio ${CPP_FILES} ${UISrcs} ${H_FILES} ${RCC_SRCS})
        qt5_use_modules(slstudio Core Gui Widgets Test)
        add_definitions(${COMPILER_DEFINES})
        target_link_libraries(slstudio slalgorithm slcamera slprojector
                ${VTK_LIBRARIES} ${PCL_LIBRARIES}
                ${OpenCV_LIBS} ${Boost_LIBRARIES})
    else(USE_QT5)
        #Legacy qt4 support

        qt4_wrap_ui(UISrcs ${UI_FILES})
        qt4_wrap_cpp(MOCSrcs ${H_FILES})
        qt4_add_resources(RCC_SRCS ${RC_FILES})

        add_executable(slstudio ${CPP_FILES} ${UISrcs} ${MOCSrcs} ${RCC_SRCS})
        target_link_libraries(slstudio slalgorithm slcamera slprojector
                Qt4::QtTest ${VTK_LIBRARIES} ${PCL_LIBRARIES}
                ${OpenCV_LIBS} ${Boost_LIBRARIES})
    endif(USE_QT5)

    if(MSVC)
        set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "/wd4819 /wd4996" )
    endif(MSVC)
endif()


