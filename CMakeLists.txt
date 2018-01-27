cmake_minimum_required(VERSION 3.7)

# create project
project(MEII)

# enable C/C++
enable_language(C)
enable_language(CXX)

# enable C++11
set(CMAKE_CXX_STANDARD 11)

# set binary output locations
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# add definitons
add_definitions(-D_CRT_SECURE_NO_WARNINGS -DNOMINMAX -D_WINSOCK_DEPRECATED_NO_WARNINGS)

#===============================================================================
# INCLUDES
#===============================================================================

# MEL Lib and my stuff
set(INCLUDE_DIRS include C:/Users/Ted/GitHub/MEL/include C:/dev/eigen)

# include directories
include_directories(${INCLUDE_DIRS})


file(GLOB SRC_EMG_RT_CONTROL
    "${CMAKE_SOURCE_DIR}/src/EmgRTControl/*.cpp"
    "${CMAKE_SOURCE_DIR}/include/EmgRTControl/*.hpp"
)

file(GLOB SRC_EMG_TESTING
	"${CMAKE_SOURCE_DIR}/src/EMGTesting/*.cpp"
    "${CMAKE_SOURCE_DIR}/include/EMGTesting/*.hpp"
)

#===============================================================================
# LINK DIRECTORIES
#===============================================================================

link_directories(C:/Users/Ted/GitHub/MEL/lib/windows)

#===============================================================================
# CREATE EXAMPLES EXECUTABLES
#===============================================================================

add_executable(EmgRTControl ${SRC_EMG_RT_CONTROL})
target_link_libraries(EmgRTControl MEL)

add_executable(EMGTesting ${SRC_EMG_TESTING})
target_link_libraries(EMGTesting MEL)


