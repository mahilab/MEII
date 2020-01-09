cmake_minimum_required(VERSION 3.13.0)

#===============================================================================
# USER OPTIONS
#===============================================================================
option(EXAMPLES    "Turn ON to build example executable(s)." OFF)
option(TESTS       "Turn ON to build test executable(s)"     OFF)

# get directory for the eigen library
set(EIGEN_ROOT "C:/dev/eigen" CACHE FILEPATH "Absolute path to Eigen installation.")

# add ./cmake to CMake module path so our .cmake files can be found
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

# create project
project(MEII VERSION 0.1.0 LANGUAGES CXX)

find_package(MEL REQUIRED)

# set binary output locations
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# add definitons
add_definitions(-D_CRT_SECURE_NO_WARNINGS -DNOMINMAX -D_WINSOCK_DEPRECATED_NO_WARNINGS)

# set compiler flags
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O3 -ldl") # all warnings
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /W3") # warning level 4
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /MP") # multicore build
endif()

#===============================================================================
# INCLUDES
#===============================================================================

# MEL Lib and Eigen Lib
set(INCLUDE_DIRS include ${EIGEN_ROOT})
include_directories(${INCLUDE_DIRS})

#===============================================================================
# FIND PROJECT SOURCE FILES
#===============================================================================

include(MEIISources)

#===============================================================================
# CREATE MEII LIBRARY AND LINK EXTERNAL LIBS
#===============================================================================

add_library(MEII STATIC ${MEII_SOURCE_FILES} ${MEII_HEADER_FILES})

target_link_libraries(MEII MEL::quanser)

#===============================================================================
# EXAMPLES
#===============================================================================

if(EXAMPLES)
    message("Building examples")
    add_subdirectory(examples)
endif()

#===============================================================================
# TESTS
#===============================================================================

if(TESTS)
    add_executable(testing tests/testing.cpp)
    target_link_libraries(testing MEII)
endif()
