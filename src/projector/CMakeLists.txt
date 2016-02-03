include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR}
        ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/src/projector)

if(WITH_LC4500API)
    list(APPEND LC4500_SRC ProjectorLC4500.cpp LC4500API/API.cpp LC4500API/usb.cpp)
    if(APPLE)
        list(APPEND LC4500_SRC LC4500API/hid.Mac.c)
    elseif(WIN32)
        list(APPEND LC4500_SRC LC4500API/hid.Win.c)
    else()
        list(APPEND LC4500_SRC LC4500API/hid.Libusb.c)
    endif()
    #message(status " :LC4500 SRC : ${LC4500_SRC}")
    add_library(LC4500Projector STATIC ${LC4500_SRC} LC4500API/API.h)
    target_link_libraries(LC4500Projector)
endif(WITH_LC4500API)

if(WITH_LC3000API)
    list(APPEND LC3000_SRC ProjectorLC3000.cpp LC3000API/lcr_cmd.cpp
            LC3000API/lcr_packetizer.cpp LC3000API/tcp_client.cpp)
    add_library(LC3000Projector STATIC ${LC3000_SRC} LC3000API/lcr_cmd.h)
    target_link_libraries(LC3000Projector)
endif(WITH_LC3000API)