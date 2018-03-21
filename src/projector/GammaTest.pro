QT += core testlib
CONFIG -= app_bundle
CONFIG += qt qtmain
TARGET = GammaTest
TEMPLATE = app

HEADERS += Projector.h\
        OpenGLContext.h\
        ProjectorOpenGL.h\
        ../camera/Camera.h

SOURCES += mainDetermineGamma.cpp\
        ProjectorOpenGL.cpp\
        ../cvtools.cpp \
        ../codec/pstools.cpp \
        ../codec/CodecPhaseShift2x3.cpp \
        ../camera/Camera.cpp 

# OpenCV
mac {

}

unix:!mac {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv
    #LIBS += -lopencv_core -lGLU
    DEFINES += WITH_CAMERACV
    DEFINES += WITH_CAMERAV4L2
    INCLUDEPATH += ../camera
}

# Compile with specific camera driver support
# opencv
contains(DEFINES, WITH_CAMERACV) {
    HEADERS += ../camera/CameraCV.h
    SOURCES += ../camera/CameraCV.cpp
}
# v4l2
contains(DEFINES, WITH_CAMERAV4L2) {
    HEADERS += ../camera/CameraV4L2.h
    SOURCES += ../camera/CameraV4L2.cpp
}

win32 {

}

# OpenGL
mac {
    OBJECTIVE_SOURCES += OpenGLContext.Mac.mm
    LIBS += -framework OpenGL -framework Cocoa
}

unix:!mac {
    SOURCES += OpenGLContext.Unix.cpp
    CONFIG += link_pkgconfig
    PKGCONFIG += gl glew x11 xrandr
    LIBS += -lXxf86vm
}

win32 {
    SOURCES += OpenGLContext.Win.cpp
    LIBS += -lOpenGL32 -lgdi32 -lUser32
}

# LC3000 Api
HEADERS += LC3000API/lcr_cmd.h \
        LC3000API/lcr_packetizer.h
SOURCES += LC3000API/tcp_client.cpp \
    LC3000API/lcr_packetizer.cpp \
    LC3000API/lcr_cmd.cpp

# LC4500 Api
#HEADERS += LC4500API/API.h
#SOURCES += LC4500API/API.cpp \
#        LC4500API/usb.cpp
#macx:SOURCES += LC4500API/hid.Mac.c
unix:!macx{
    #SOURCES += LC4500API/hid.Unix.c
    CONFIG += link_pkgconfig
    PKGCONFIG += libudev
}
win32{
    SOURCES += LC4500API/hid.Win.c
    LIBS += -lsetupapi
}
