TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += lc3000startup.cpp

# LC3000 Api
INCLUDEPATH += ../../src/projector/LC3000API/
HEADERS += ../../src/projector/LC3000API/lcr_cmd.h
SOURCES += ../../src/projector/ProjectorLC3000.cpp \
        ../../src/projector/LC3000API/lcr_cmd.cpp \
        ../../src/projector/LC3000API/lcr_packetizer.cpp \
        ../../src/projector/LC3000API/tcp_client.cpp
