TEMPLATE = app
CONFIG -= app_bundle
CONFIG -= qt

HEADERS += Codec.h \
        CodecGrayCode.h \
        CodecPhaseShift.h \
        ../projector/Contexth \
        ../projector/Projector.h

SOURCES += mainCodecTest.cpp\
        CodecGrayCode.cpp \
        CodecPhaseShift.cpp \
        ../projector/Context.Unix.cpp \
        ../projector/Projector.cpp

INCLUDEPATH += ../projector/

# pkg-config libs
CONFIG += link_pkgconfig
PKGCONFIG += opencv  gl glu x11 xrandr
