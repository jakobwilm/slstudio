QT       += core gui opengl testlib
CONFIG   += qt thread console
TARGET = CameraTest
TEMPLATE = app

FORMS += CameraTest.ui

HEADERS += Camera.h \
        ../SLVideoWidget.h \
        CameraWorker.h \
        CameraTest.h

SOURCES += \
        Camera.cpp \
        ../SLVideoWidget.cpp \
        CameraWorker.cpp \
        CameraTest.cpp \
        mainCameraTest.cpp

win32 {
    # opencv
    INCLUDEPATH += C:/opencv/build/include
    LIBS += -L"C:/opencv/build/x64/vc10/lib" -lopencv_core246

    DEFINES -= UNICODE

    # uEye API
    exists("C:/Program Files/IDS/uEye/Develop/include/uEye.h"){
        DEFINES += WITH_CAMERAIDSIMAGING
        INCLUDEPATH += "C:/Program Files/IDS/uEye/Develop/include/"
        LIBS += -L"C:/Program Files/IDS/uEye/Develop/Lib" -luEye_api_64
    }

    # XIMEA API
    exists("C:/XIMEA/API/xiApi.h"){
        INCLUDEPATH += "C:/XIMEA/API"
        DEFINES += WITH_CAMERAXIMEA
        LIBS += -L"C:/XIMEA/API/x64" -lm3apiX64
    }
}

unix:!macx {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv
    #LIBS += -lopencv_core -lGLU
    DEFINES += WITH_CAMERACV
    DEFINES += WITH_CAMERAV4L2
    exists(/usr/include/dc1394/dc1394.h){
        DEFINES += WITH_CAMERAIIDC
        LIBS += -ldc1394
    }
    exists(/usr/include/ueye.h){
        DEFINES += WITH_CAMERAIDSIMAGING
        LIBS += -lueye_api
    }
    exists(/opt/XIMEA/include/xiApi.h){
        INCLUDEPATH += /opt/XIMEA/include
        DEFINES += WITH_CAMERAXIMEA
        LIBS += -lm3api
    }
}

macx {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv
    exists(/usr/local/include/dc1394/dc1394.h) {
        DEFINES += WITH_CAMERAIIDC
        LIBS += -L/usr/local/lib -ldc1394
    }
    exists(/Library/Frameworks/m3api.framework/m3api){
        DEFINES += WITH_CAMERAXIMEA
        INCLUDEPATH += /Library/Frameworks/m3api.framework/Headers/
        LIBS += -framework m3api
    }
}

# Compile with specific camera driver support
# opencv
contains(DEFINES, WITH_CAMERACV) {
    HEADERS += CameraCV.h
    SOURCES +=CameraCV.cpp
}
# v4l2
contains(DEFINES, WITH_CAMERAV4L2) {
    HEADERS += CameraV4L2.h
    SOURCES +=CameraV4L2.cpp
}
# libdc1394
contains(DEFINES, WITH_CAMERAIIDC) {
    HEADERS += CameraIIDC.h
    SOURCES +=CameraIIDC.cpp
}
# libueye
contains(DEFINES, WITH_CAMERAIDSIMAGING) {
    HEADERS += CameraIDSImaging.h
    SOURCES += CameraIDSImaging.cpp
}
# libm3api
contains(DEFINES, WITH_CAMERAXIMEA) {
    HEADERS += CameraXIMEA.h
    SOURCES += CameraXIMEA.cpp
}
