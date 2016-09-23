include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR}
        ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/src/projector)
set(DEVICE_LIBS)
set(DEVICE_DEFS)

if(WITH_LC4500API)
    list(APPEND LC4500_SRC ProjectorLC4500.cpp LC4500API/API.cpp LC4500API/usb.cpp)
    if(APPLE)
        list(APPEND LC4500_SRC LC4500API/hid.Mac.c)
    elseif(WIN32)
        list(APPEND LC4500_SRC LC4500API/hid.Win.c)
    else()
        list(APPEND LC4500_SRC LC4500API/hid.Libusb.c)
    endif()
    add_library(LC4500Projector STATIC ${LC4500_SRC} LC4500API/API.h)
    set(DEVICE_DEFS ${DEVICE_DEFS} -DWITH_LC4500API)

    if(WIN32)
        set(PROJ4500_LIBS LC4500Projector setupapi)
    else(WIN32)
        set(PROJ4500_LIBS LC4500Projector)
    endif(WIN32)
endif(WITH_LC4500API)

if(WITH_LC3000API)
    list(APPEND LC3000_SRC ProjectorLC3000.cpp LC3000API/lcr_cmd.cpp
            LC3000API/lcr_packetizer.cpp LC3000API/tcp_client.cpp)
    add_library(LC3000Projector STATIC ${LC3000_SRC} LC3000API/lcr_cmd.h)
    set(PROJ3000_LIBS LC3000Projector)
    set(DEVICE_DEFS ${DEVICE_DEFS} -DWITH_LC3000API)
endif(WITH_LC3000API)

set(DEVICE_LIBS ${DEVICE_LIBS} ${PROJ4500_LIBS} ${PROJ3000_LIBS})

if(WITH_PROJECTOROPENGL)
    # Compile with system dependent OpenGL Context code
    if(APPLE)
        set(OPENGL_CONTEXT_CPP OpenGLContext.Mac.mm)
    elseif(WIN32)
        set(OPENGL_CONTEXT_CPP OpenGLContext.Win.cpp)
        add_definitions(-DWIN32 -D_UNICODE -DUNICODE)
    else()
        set(OPENGL_CONTEXT_CPP OpenGLContext.Unix.cpp)
    endif()

    add_library(OpenGlProjector STATIC ProjectorOpenGL.cpp ${OPENGL_CONTEXT_CPP} ProjectorOpenGL.h OpenGLContext.h)
    set(DEVICE_LIBS ${DEVICE_LIBS} OpenGlProjector ${OPENGL_LIBRARIES} ${GLEW_LIBRARY})
    set(DEVICE_DEFS ${DEVICE_DEFS} -DWITH_PROJECTOROPENGL)
endif(WITH_PROJECTOROPENGL)

if(WITH_PROJECTORQT)
    qt4_wrap_cpp(MOCSrcs ProjectorQtGL.h)
    add_library(QtProjector STATIC ProjectorQtGL.cpp ProjectorQtGL.h OpenGLContext.h ${MOCSrcs})
    set(DEVICE_LIBS ${DEVICE_LIBS} QtProjector)
    set(DEVICE_DEFS ${DEVICE_DEFS} -DWITH_PROJECTORQT)
endif(WITH_PROJECTORQT)

set(PROJECTOR_DEVICE_LIBS ${DEVICE_LIBS} PARENT_SCOPE)
set(PROJECTOR_DEVICE_DEFS ${DEVICE_DEFS} PARENT_SCOPE)