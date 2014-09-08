TEMPLATE = app
CONFIG -= app_bundle
CONFIG -= qt

HEADERS += Projector.h\
    ProjectorGrayCode.h \
    Calibrator.h

SOURCES += mainCalibratorTest.cpp\
    ProjectorGrayCode.cpp \
    Calibrator.cpp

OBJECTIVE_SOURCES += Projector.mm

mac {
    LIBS += -framework OpenGL -framework Cocoa
}
