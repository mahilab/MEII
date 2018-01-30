cmake_minimum_required(VERSION 3.7)


set(QUANSER_ROOT "C:/Program Files/Quanser" CACHE FILEPATH "Absolute path to Quanser installation."          )

# enable C/C++
enable_language(C)
enable_language(CXX)

# enable C++11
set(CMAKE_CXX_STANDARD 11)

# create project
project(MEII)

# set binary output locations
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# add definitons
add_definitions(-D_CRT_SECURE_NO_WARNINGS -DNOMINMAX -D_WINSOCK_DEPRECATED_NO_WARNINGS)
add_definitions(-DMEL_INCLUDE_EIGEN)

# add configurations
set(CMAKE_CONFIGURATION_TYPES release)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /W4")

#===============================================================================
# INCLUDES
#===============================================================================

# MEL Lib and my stuff
set(INCLUDE_DIRS include C:/Users/Ted/GitHub/MEL/include C:/dev/eigen)

# include directories
include_directories(${INCLUDE_DIRS})


file(GLOB SRC_EMG_RT_CONTROL
    "${CMAKE_SOURCE_DIR}/src/EmgRTControl/EmgRTControl.cpp"
    "${CMAKE_SOURCE_DIR}/src/EmgRTControl/main.cpp"
    "${CMAKE_SOURCE_DIR}/include/EmgRTControl/EmgRTControl.hpp"
)


#===============================================================================
# LINK DIRECTORIES
#===============================================================================

set(LINK_LIBS "")
link_directories(C:/Users/Ted/GitHub/MEL/lib/windows/release)
link_directories("${QUANSER_ROOT}/QUARC/lib/win64")
list(APPEND LINK_LIBS
    MEL
    ws2_32
    winmm
    Pdh
    Psapi
    hil
    quanser_communications
    quanser_runtime
    quanser_common
    bufferoverflowU
    legacy_stdio_definitions)

#===============================================================================
# CREATE EXAMPLES EXECUTABLES
#===============================================================================



add_executable(EmgRTControl ${SRC_EMG_RT_CONTROL})
target_link_libraries(EmgRTControl ${LINK_LIBS})

add_executable(melnet_unity "${CMAKE_SOURCE_DIR}/src/EmgRTControl/ex_melnet_unity.cpp")
target_link_libraries(melnet_unity ${LINK_LIBS})