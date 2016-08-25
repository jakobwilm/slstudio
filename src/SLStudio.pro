#-----------------------------------------------------
#
# SLStudio - Platform for Real-Time  Structured Light
#
# (c) 2013--2014 Jakob Wilm, DTU, Kgs.Lyngby, Denmark
#
#-----------------------------------------------------

QT       += core gui opengl testlib
CONFIG   += qt thread sse2
TARGET = SLStudio
TEMPLATE = app

FORMS    += SLStudio.ui \
        SLPreferenceDialog.ui \
        SLCalibrationDialog.ui \
        SLVideoDialog.ui \
        SLAboutDialog.ui \
        SLTrackerDialog.ui

HEADERS  += SLStudio.h \
        SLVideoWidget.h \
        SLPreferenceDialog.h \
        SLCalibrationDialog.h \
        SLCameraVirtual.h \
        SLProjectorVirtual.h \
        SLScanWorker.h \
        SLDecoderWorker.h \
        SLTrackerWorker.h \
        SLVideoDialog.h \
        SLAboutDialog.h \
        SLPoseWidget.h \
        SLPointCloudWidget.h \
        SLTrackerDialog.h \
        SLTriangulatorWorker.h \
        SLTraceWidget.h \
        camera/Camera.h \
        projector/Projector.h \
        projector/ProjectorOpenGL.h \
        projector/OpenGLContext.h \
        projector/ProjectorLC3000.h \
        projector/ProjectorLC4500.h \
        projector/ProjectorQtGL.h \
        codec/Codec.h \
        codec/phaseunwrap.h \
        codec/phasecorr.h \
        codec/pstools.h \
        codec/CodecCalibration.h \
        codec/CodecPhaseShift2x3.h \
        codec/CodecPhaseShiftDescatter.h \
        codec/CodecPhaseShift3.h \
        codec/CodecPhaseShift3FastWrap.h \
        codec/CodecPhaseShift3Unwrap.h \
        codec/CodecPhaseShift4.h \
        codec/CodecGrayCode.h \
        codec/CodecPhaseShift2p1.h \
        codec/CodecFastRatio.h \
        codec/CodecPhaseShiftModulated.h \
        codec/CodecPhaseShiftMicro.h \
        codec/CodecPhaseShiftNStep.h \
        triangulator/Triangulator.h \
        calibrator/CalibrationData.h \
        calibrator/Calibrator.h \
        calibrator/CalibratorLocHom.h \
        calibrator/CalibratorRBF.h \
        calibrator/CThinPlateSpline.h \
        calibrator/RBFInterpolator.h \
        tracker/Tracker.h \
        tracker/TrackerICP.h \
        tracker/TrackerNDT.h \
        tracker/CorrRejectOrgBoundFast.h \
        tracker/CorrEstOrgProjFast.h \
        tracker/CorrEstKdTreeFast.h \
        tracker/TrackerPCL.h \
        tracker/PoseFilter.h \
        cvtools.h \
        camera/CameraOpenCV.h


SOURCES += main.cpp \
        SLStudio.cpp \
        SLScanWorker.cpp \
        SLDecoderWorker.cpp \
        SLTrackerWorker.cpp \
        SLVideoWidget.cpp \
        SLPreferenceDialog.cpp \
        SLCalibrationDialog.cpp \
        SLCameraVirtual.cpp \
        SLProjectorVirtual.cpp \
        SLVideoDialog.cpp \
        SLAboutDialog.cpp \
        SLPoseWidget.cpp \
        SLPointCloudWidget.cpp \
        SLTrackerDialog.cpp \
        SLTriangulatorWorker.cpp \
        SLTraceWidget.cpp \
        camera/Camera.cpp \
        projector/ProjectorOpenGL.cpp \
        projector/ProjectorQtGL.cpp \
        codec/phaseunwrap.cpp \
        codec/phasecorr.cpp \
        codec/CodecCalibration.cpp \
        codec/CodecPhaseShift2x3.cpp \
        codec/CodecPhaseShiftDescatter.cpp \
        codec/CodecPhaseShift3.cpp \
        codec/CodecPhaseShift3FastWrap.cpp \
        codec/CodecPhaseShift3Unwrap.cpp \
        codec/CodecPhaseShift4.cpp \
        codec/CodecFastRatio.cpp \
        codec/CodecPhaseShift2p1.cpp \
        codec/CodecPhaseShiftModulated.cpp \
        codec/CodecPhaseShiftMicro.cpp \
        codec/CodecGrayCode.cpp \
        codec/pstools.cpp \
        codec/CodecPhaseShiftNStep.cpp \
        triangulator/Triangulator.cpp \
        calibrator/CalibrationData.cpp \
        calibrator/CalibratorLocHom.cpp \
        calibrator/CalibratorRBF.cpp \
        calibrator/CThinPlateSpline.cpp \
        calibrator/RBFInterpolator.cpp \
        cvtools.cpp \
        tracker/TrackerICP.cpp \
        tracker/TrackerNDT.cpp \
        tracker/CorrRejectOrgBoundFast.cpp \
        tracker/TrackerPCL.cpp \
        tracker/PoseFilter.cpp \
        camera/CameraOpenCV.cpp

INCLUDEPATH += camera/ projector/ codec/ triangulator/ calibrator/ tracker/

RESOURCES += \
    SLResources.qrc

# Operating System dependant linking and including
# Linux
unix:!macx {
    CONFIG += link_pkgconfig
    # Link VTK and Boost (no pkg-config)
    INCLUDEPATH += /usr/include/vtk-5.10/
    LIBS += -lQVTK -lvtkCommon -lvtkFiltering -lvtkRendering -lvtkIO -lvtkGraphics -lvtkHybrid
    # PCL pkg-config workaround
    LIBS += -lboost_system -lpcl_visualization -lpcl_common -lpcl_io -lpcl_search -lpcl_surface
    # PKG-config libs
    INCLUDEPATH += /usr/include/pcl-1.7 /usr/include/eigen3/
    PKGCONFIG += opencv pcl_registration-1.7 pcl_visualization-1.7 pcl_surface-1.7 pcl_search-1.7 pcl_filters-1.7 pcl_kdtree-1.7 pcl_tracking-1.7 flann eigen3
}
# Windows
win32 {
    # Boost join
    DEFINES += DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED

    # opencv
    INCLUDEPATH += "$$(OPENCV_INCLUDE_DIR)/" #C:\opencv\build\include

    CONFIG(debug,debug|release){
        #debug
        LIBS += -L"$$(OPENCV_DIR)" \ #C:\opencv\build\x64\vc12\lib
                -lopencv_core2411d \
                -lopencv_highgui2411d \
                -lopencv_imgproc2411d \
                -lopencv_calib3d2411d
    } else {
        #release
        LIBS += -L"$$(OPENCV_DIR)" \
                -lopencv_core2411 \
                -lopencv_highgui2411 \
                -lopencv_imgproc2411 \
                -lopencv_calib3d2411
    }

    # pcl
    INCLUDEPATH += "$$(PCL_INCLUDE_DIR)/" #C:\Program Files\PCL\include\pcl-1.7

    CONFIG(debug,debug|release){
        #debug
        LIBS += -L"$$(PCL_DIR)" \ #C:\Program Files\PCL\lib
                -lpcl_visualization_debug \
                -lpcl_io_debug \
                -lpcl_common_debug \
                -lpcl_features_debug \
                -lpcl_filters_debug \
                -lpcl_io_debug \
                -lpcl_io_ply_debug \
                -lpcl_kdtree_debug \
                -lpcl_keypoints_debug \
                -lpcl_octree_debug \
                -lpcl_registration_debug \
                -lpcl_sample_consensus_debug \
                -lpcl_search_debug \
                -lpcl_segmentation_debug \
                -lpcl_surface_debug \
                -lpcl_tracking_debug \
                -lpcl_visualization_debug
    } else {
        # release
        LIBS += -L"$$(PCL_DIR)" \
                -lpcl_visualization_release \
                -lpcl_io_release \
                -lpcl_common_release \
                -lpcl_features_release \
                -lpcl_filters_release \
                -lpcl_io_release \
                -lpcl_io_ply_release \
                -lpcl_kdtree_release \
                -lpcl_keypoints_release \
                -lpcl_octree_release \
                -lpcl_registration_release \
                -lpcl_sample_consensus_release \
                -lpcl_search_release \
                -lpcl_segmentation_release \
                -lpcl_surface_release \
                -lpcl_tracking_release \
                -lpcl_visualization_release
    }

    # pcl dependencies
    INCLUDEPATH += "$$(BOOST_ROOT)/include" \
                   "$$(EIGEN_ROOT)" \
                   "$$(FLANN_ROOT)/include"
    LIBS += -L"$$(BOOST_ROOT)/lib" -lboost_system-vc100-mt-1_50 -lboost_system-vc100-mt-gd-1_50

    # vtk
    INCLUDEPATH += "$$(VTK_INCLUDE_DIR)" #C:\Program Files\VTK\include\vtk-5.10

    CONFIG(debug,debug|release){
    #debug
    LIBS += -L"$$(VTK_DIR)" \ #C:\Program Files\VTK\lib\vtk-5.10
            -lvtkGraphics-gd \
            -lQVTK-gd \
            -lvtkCommon-gd \
            -lvtkFiltering-gd \
            -lvtkRendering-gd \
            -lvtkIO-gd \
            -lvtkpng-gd \
            -lvtksys-gd \
            -lvtktiff-gd \
            -lvtkjpeg-gd \
            -lvtkexpat-gd \
            -lvtkzlib-gd
    } else {
    # release
    LIBS += -L"$$(VTK_DIR)" \
            -lvtkGraphics \
            -lQVTK \
            -lvtkCommon \
            -lvtkFiltering \
            -lvtkRendering \
            -lvtkIO \
            -lvtkpng \
            -lvtksys \
            -lvtktiff \
            -lvtkjpeg \
            -lvtkexpat \
            -lvtkzlib
    }

}
# Mac OS X
macx {
    INCLUDEPATH += /opt/local/include/vtk-5.10/
    LIBS += -L/opt/local/lib/vtk-5.10/ -lQVTK -lvtkCommon -lvtkFiltering -lvtkRendering -lvtkIO -lvtkGraphics
    LIBS += -L/opt/local/lib/ -lboost_system-mt
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv pcl_visualization-1.7 pcl_filters-1.7 pcl_search-1.7 pcl_surface-1.7 pcl_tracking-1.7 pcl_registration-1.7
    DEFINES += BOOST_TT_HAS_OPERATOR_HPP_INCLUDED
}


# Compile with system dependent OpenGL Context code
unix:!macx{
    SOURCES += projector/OpenGLContext.Unix.cpp
    LIBS += -lXxf86vm
    PKGCONFIG += gl glu glew x11 #xrandr
#    SOURCES += projector/OpenGLContext.GLFW.cpp
#    LIBS += -lglfw3 -lXxf86vm -lXi
}
win32{
    SOURCES += projector/OpenGLContext.Win.cpp
    INCLUDEPATH += "$$(GLEW_INCLUDE_DIR)" #C:\Program Files\glew-1.12.0\include
    LIBS += -L"$$(GLEW_DIR)" -lglew32 #C:\Program Files\glew-1.12.0\lib\Release\x64
}
macx{
    CONFIG += objective_c
    OBJECTIVE_SOURCES += projector/OpenGLContext.Mac.mm
    LIBS += -framework Cocoa -framework OpenGL
    PKGCONFIG += glew
#    SOURCES += projector/OpenGLContext.GLFW.cpp
#    LIBS += -L/usr/local/lib/ -lglfw3
}


# Compile with specific camera driver bindings
# libdc1394
unix:!macx:exists(/usr/include/dc1394/dc1394.h) {
    DEFINES += WITH_CAMERAIIDC
    LIBS += -ldc1394
}
macx:exists(/usr/local/include/dc1394/dc1394.h) {
    DEFINES += WITH_CAMERAIIDC
    LIBS += -ldc1394
}
contains(DEFINES, WITH_CAMERAIIDC) {
    HEADERS += camera/CameraIIDC.h
    SOURCES += camera/CameraIIDC.cpp
}
# IDS Imaging libueye
unix:!macx:exists(/usr/include/ueye.h) {
    DEFINES += WITH_CAMERAIDSIMAGING
    LIBS += -lueye_api
}
win32:exists("C:/Program Files/IDS/uEye/Develop/include/uEye.h"){
    DEFINES += WITH_CAMERAIDSIMAGING
    INCLUDEPATH += "C:/Program Files/IDS/uEye/Develop/include/"
    LIBS += -L"C:/Program Files/IDS/uEye/Develop/Lib" -luEye_api_64
}
contains(DEFINES, WITH_CAMERAIDSIMAGING) {
    HEADERS += camera/CameraIDSImaging.h
    SOURCES += camera/CameraIDSImaging.cpp
}
# XIMEA libm3api
unix:!macx:exists(/opt/XIMEA/include/xiApi.h){
    INCLUDEPATH += /opt/XIMEA/include
    DEFINES += WITH_CAMERAXIMEA
    LIBS += -lm3api
}
win32:exists("C:/XIMEA/API/xiApi.h"){
    DEFINES += WITH_CAMERAXIMEA
    INCLUDEPATH += "C:/XIMEA/API"
    LIBS += -L"C:/XIMEA/API/x64" -lm3apiX64
}
macx:exists(/Library/Frameworks/m3api.framework/m3api){
    DEFINES += WITH_CAMERAXIMEA
    INCLUDEPATH += /Library/Frameworks/m3api.framework/Headers/
    LIBS += -framework m3api
}
contains(DEFINES, WITH_CAMERAXIMEA) {
    HEADERS += camera/CameraXIMEA.h
    SOURCES += camera/CameraXIMEA.cpp
}
# Point Grey flycapture
unix:!macx:exists(/usr/include/flycapture/FlyCapture2.h){
    INCLUDEPATH += /usr/include/flycapture
    DEFINES += WITH_CAMERAPOINTGREY
    LIBS += -lflycapture
}
win32:exists("C:/Program Files/Point Grey Research/FlyCapture2/include/FlyCapture2.h"){
    DEFINES += WITH_CAMERAPOINTGREY
    INCLUDEPATH += "C:/Program Files/Point Grey Research/FlyCapture2/include/"
    LIBS += -L"C:/Program Files/Point Grey Research/FlyCapture2/lib64" -lFlyCapture2
}
contains(DEFINES, WITH_CAMERAPOINTGREY) {
    HEADERS += camera/CameraPointGrey.h
    SOURCES += camera/CameraPointGrey.cpp
}


# Compile with direct projector APIs
# LC3000 Api
DEFINES += WITH_LC3000API
HEADERS += projector/LC3000API/lcr_cmd.h
SOURCES += projector/ProjectorLC3000.cpp \
        projector/LC3000API/lcr_cmd.cpp \
        projector/LC3000API/lcr_packetizer.cpp \
        projector/LC3000API/tcp_client.cpp

## LC4500 Api
DEFINES += WITH_LC4500API
HEADERS += projector/LC4500API/API.h
SOURCES += projector/ProjectorLC4500.cpp \
        projector/LC4500API/API.cpp \
        projector/LC4500API/usb.cpp
macx:SOURCES += projector/LC4500API/hid.Mac.c
unix:!macx{
    #SOURCES += projector/LC4500API/hid.Unix.c
    SOURCES += projector/LC4500API/hid.Libusb.c
    CONFIG += link_pkgconfig
    #PKGCONFIG += libudev
    PKGCONFIG += libusb-1.0
}
win32{
    SOURCES += projector/LC4500API/hid.Win.c
    LIBS += -lsetupapi
}

unix:!macx{
    DEFINES += WITH_CAMERAV4L
    HEADERS += camera/CameraV4L.h
    SOURCES += camera/CameraV4L.cpp
}
