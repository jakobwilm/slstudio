TEMPLATE = app
QT += core
CONFIG -= app_bundle
CONFIG += qt

TARGET = TrackerTest

HEADERS += Tracker.h \
           TrackerICP.h \
           TrackerNDT.h \
           CorrEstOrgProjFast.h \
           CorrRejectOrgBoundFast.h \
           CorrEstKdTreeFast.h \
           TrackerPCL.h

SOURCES += mainTrackerTest.cpp\
           TrackerICP.cpp \
           TrackerNDT.cpp \
           CorrRejectOrgBoundFast.cpp \
           TrackerPCL.cpp

# Mac OS X
macx {
    LIBS += -L/opt/local/lib/ -lboost_system-mt
    CONFIG -= no-pkg-config
    CONFIG += link_pkgconfig
    PKGCONFIG += pcl_visualization-1.7 pcl_filters-1.7 pcl_search-1.7 pcl_registration-1.7 pcl_tracking-1.7
    DEFINES += BOOST_TT_HAS_OPERATOR_HPP_INCLUDED
    INCLUDEPATH += /usr/local/include/pcl-1.7/
}

# Linux
unix:!macx {
    # Link VTK and Boost (no pkg-config)
    INCLUDEPATH += /usr/include/vtk-5.8/
    LIBS += -lQVTK -lvtkCommon -lvtkFiltering -lvtkRendering -lvtkIO
    # PCL pkg-config workaround
    LIBS += -lboost_system -lpcl_visualization -lpcl_common -lpcl_io -lpcl_search -lpcl_surface
    # PKG-config libs
    CONFIG += link_pkgconfig
    INCLUDEPATH += /usr/local/include/pcl-1.7/ /usr/include/eigen3/
    PKGCONFIG += gl glu x11 xrandr opencv pcl_registration-1.7 pcl_visualization-1.7 pcl_surface-1.7 pcl_search-1.7 pcl_filters-1.7 pcl_kdtree-1.7  pcl_tracking-1.7 flann eigen3
}
