QT += core testlib
CONFIG -= app_bundle
CONFIG += qt qtmain
TARGET = ProjectorTest
TEMPLATE = app

HEADERS += Projector.h\
        OpenGLContext.h\
        ProjectorOpenGL.h

SOURCES += mainProjectorTest.cpp\
        ProjectorOpenGL.cpp\
        ../cvtools.cpp

# OpenCV
mac {

}

unix:!mac {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv4
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
    #LIBS += -lXxf86vm
}

win32 {
    SOURCES += OpenGLContext.Win.cpp
    LIBS += -lOpenGL32 -lgdi32 -lUser32
}

# Compile with direct projector APIs
# LC3000 Api
DEFINES += WITH_LC3000API
HEADERS += LC3000API/lcr_cmd.h
SOURCES += ProjectorLC3000.cpp \
        LC3000API/lcr_cmd.cpp \
        LC3000API/lcr_packetizer.cpp \
        LC3000API/tcp_client.cpp

## LC4500 Api
DEFINES += WITH_LC4500API
HEADERS += LC4500API/dlpc350_api.h \
        LC4500API/dlpc350_usb.h \
        LC4500API/dlpc350_common.h
SOURCES += ProjectorLC4500.cpp \
        LC4500API/dlpc350_api.cpp \
        LC4500API/dlpc350_usb.cpp \
        LC4500API/dlpc350_common.cpp
macx:SOURCES += LC4500API/hid.Mac.c
unix:!macx{
    #SOURCES += LC4500API/hid.Unix.c
    SOURCES += LC4500API/hid.Libusb.c
    CONFIG += link_pkgconfig
    #PKGCONFIG += libudev
    PKGCONFIG += libusb-1.0
}
win32{
    SOURCES += LC4500API/hid.Win.c
    LIBS += -lsetupapi
}
