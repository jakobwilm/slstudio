TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += lc4500startup.cpp

# LC4500 Api
INCLUDEPATH += ../../src/projector/LC4500API/
SOURCES += ../../src/projector/LC4500API/API.cpp \
           ../../src/projector/LC4500API/usb.cpp
macx:SOURCES += ../../src/projector/LC4500API/hid.Mac.c
unix:!macx {
    SOURCES += ../../src/projector/LC4500API/hid.Libusb.c
    INCLUDEPATH += /usr/include/libusb-1.0/
    LIBS += -lusb-1.0 -lpthread
#    SOURCES += ../../src/projector/LC4500API/hid.Unix.c
#    LIBS += -ludev
}
win32 {
    SOURCES += ../../src/projector/LC4500API/hid.Win.c
    LIBS += -lsetupapi
}
