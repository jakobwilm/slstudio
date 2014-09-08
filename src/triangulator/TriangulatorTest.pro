TEMPLATE = app
CONFIG -= app_bundle
CONFIG -= qt

HEADERS += Reconstructor.h \
    ReconstructorGrayCode.h

SOURCES += mainReconstructorTest.cpp\
           Reconstructor.cpp \
        ReconstructorGrayCode.cpp

LIBS    += -lopencv_ocl
