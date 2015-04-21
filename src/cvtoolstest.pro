TEMPLATE = app
CONFIG -= app_bundle
CONFIG -= qt

HEADERS += cvtools.h phasecorr.h

SOURCES += cvtools.cpp phasecorr.cpp mainCvtoolstest.cpp

# pkg-config libs
CONFIG += link_pkgconfig
PKGCONFIG += opencv
