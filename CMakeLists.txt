cmake_minimum_required(VERSION 3.7)

#===============================================================================
# USER OPTIONS
#===============================================================================
option(OSIM        "Turn ON to build OpenSim related executable(s)."                           OFF)
option(PHRI_LEARN  "Turn ON to build Phyical Human-Robot Interaction Learning executable(s)."  OFF)
option(EMG_RT_CTRL "Turn ON to build EMG Real-Time Control executable(s)."                     OFF)
option(MYO         "Turn ON to build Thalmic Labs Myo Armband."                                OFF)
option(EXAMPLES    "Turn ON to build example executable(s)."                                   OFF)
option(TESTS       "Turn ON to build test executable(s)"                                       OFF)

set(QUANSER_ROOT "C:/Program Files/Quanser"     CACHE FILEPATH "Absolute path to Quanser installation.")
set(MEL_ROOT     "C:/Git/MEL"                   CACHE FILEPATH "Absolute path to MEL.")
set(EIGEN_ROOT   "C:/dev/eigen"                 CACHE FILEPATH "Absolute path to Eigen installation.")
set(MYO_ROOT     "C:/dev/myo-sdk-win-0.9.0"     CACHE FILEPATH "Absolute path to Myo SDK.")

# enable C/C++
enable_language(C)
enable_language(CXX)

# enable C++11
set(CMAKE_CXX_STANDARD 11)

# create project
project(MEII)

# set binary output locations
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# add definitons
add_definitions(-D_CRT_SECURE_NO_WARNINGS -DNOMINMAX -D_WINSOCK_DEPRECATED_NO_WARNINGS)

# set compiler flags
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /W4") # WARNING LEVEL 4
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /MP") # MULTICORE BUILDS

#===============================================================================
# INCLUDES
#===============================================================================

# MEL Lib and Eigen Lib
set(INCLUDE_DIRS include ${MEL_ROOT}/include ${EIGEN_ROOT})
if (MYO)
    list(APPEND INCLUDE_DIRS ${MYO_ROOT}/include)
endif()

# include directories
include_directories(${INCLUDE_DIRS})

#===============================================================================
# FIND PROJECT SOURCE FILES
#===============================================================================

file(GLOB SRC_MAHI_EXO_II
    "${CMAKE_SOURCE_DIR}/src/MEII/MahiExoII/*.cpp"
    "${CMAKE_SOURCE_DIR}/include/MEII/MahiExoII/*.hpp"
)

file(GLOB SRC_CONTROL
    "${CMAKE_SOURCE_DIR}/src/MEII/Control/*.cpp"
    "${CMAKE_SOURCE_DIR}/include/MEII/Control/*.hpp"
)

file(GLOB SRC_EMG
    "${CMAKE_SOURCE_DIR}/src/MEII/EMG/*.cpp"
    "${CMAKE_SOURCE_DIR}/include/MEII/EMG/*.hpp"
)

file(GLOB SRC_SIGNAL_PROCESSING
    "${CMAKE_SOURCE_DIR}/src/MEII/SignalProcessing/*.cpp"
    "${CMAKE_SOURCE_DIR}/include/MEII/SignalProcessing/*.hpp"
)

file(GLOB SRC_CLASSIFICATION
    "${CMAKE_SOURCE_DIR}/src/MEII/Classification/*.cpp"
    "${CMAKE_SOURCE_DIR}/include/MEII/Classification/*.hpp"
)

file(GLOB SRC_UNITY
    "${CMAKE_SOURCE_DIR}/src/MEII/Unity/*.cpp"
    "${CMAKE_SOURCE_DIR}/include/MEII/Unity/*.hpp"
)

file(GLOB SRC_UTILITY
    "${CMAKE_SOURCE_DIR}/src/MEII/Utility/*.cpp"
    "${CMAKE_SOURCE_DIR}/include/MEII/Utility/*.hpp"
)

file(GLOB SRC_EMG_REAL_TIME_CONTROL
    "${CMAKE_SOURCE_DIR}/src/MEII/EmgRealTimeControl/*.cpp"
    "${CMAKE_SOURCE_DIR}/include/MEII/EmgRealTimeControl/*.hpp"
)

file(GLOB SRC_PHRI_LEARNING
    "${CMAKE_SOURCE_DIR}/src/MEII/PhriLearning/*.cpp"
    "${CMAKE_SOURCE_DIR}/include/MEII/PhriLearning/*.hpp"
)

file(GLOB SRC_OSIM
    "${CMAKE_SOURCE_DIR}/src/MEII/OpenSim/*.cpp"
    "${CMAKE_SOURCE_DIR}/include/MEII/OpenSim/*.hpp"
)

# MEII expansion library source files
set(MEII_SOURCE_FILES
    ${SRC_MAHI_EXO_II}
    ${SRC_CONTROL}
    ${SRC_EMG}
    ${SRC_SIGNAL_PROCESSING}
    ${SRC_CLASSIFICATION}
    ${SRC_UNITY}
    ${SRC_UTILITY}
)

if (EMG_RT_CTRL)
    list(APPEND MEII_SOURCE_FILES ${SRC_EMG_REAL_TIME_CONTROL})
endif()

if (PHRI_LEARN)
    list(APPEND MEII_SOURCE_FILES ${SRC_PHRI_LEARNING})
endif()

if (OSIM)
    list(APPEND MEII_SOURCE_FILES ${SRC_OSIM})
endif()

# create filters for Visual Studio
source_group("MahiExoII" FILES ${SRC_MAHI_EXO_II})
source_group("Control" FILES ${SRC_CONTROL})
source_group("EMG" FILES ${SRC_EMG})
source_group("SignalProcessing" FILES ${SRC_SIGNAL_PROCESSING})
source_group("Classification" FILES ${SRC_CLASSIFICATION})
source_group("Unity" FILES ${SRC_UNITY})
source_group("Utility" FILES ${SRC_UTILITY})
source_group("EmgRealTimeControl" FILES ${SRC_EMG_REAL_TIME_CONTROL})
source_group("PhriLearning" FILES ${SRC_PHRI_LEARNING})
source_group("OpenSim" FILES ${SRC_OSIM})




#===============================================================================
# CREATE MEII LIBRARY
#===============================================================================

add_library(MEII STATIC ${MEII_SOURCE_FILES})


#===============================================================================
# LINK EXTERNAL LIBS
#===============================================================================

set(LINK_LIBS "")
link_directories("${MEL_ROOT}/lib/windows/release")
if (CMAKE_SIZEOF_VOID_P EQUAL 8) # 64-bit
    link_directories("${QUANSER_ROOT}/QUARC/lib/win64")
else() # 32-bit
    link_directories("${QUANSER_ROOT}/QUARC/lib/windows")
endif()

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

if (MYO)
    link_directories("${MYO_ROOT}/lib")
    if (CMAKE_SIZEOF_VOID_P EQUAL 8) # 64-bit
        list(APPEND LINK_LIBS myo64)
    else() # 32-bit
        list(APPEND LINK_LIBS myo32)
    endif()
endif()

target_link_libraries(MEII ${LINK_LIBS})

#===============================================================================
# CREATE EXECUTABLES
#===============================================================================



if (OSIM)   
    add_executable(osim_data_collection src/osim_data_collection.cpp)
    target_link_libraries(osim_data_collection MEII)

    add_executable(osim_emg_data_collection src/osim_emg_data_collection.cpp)
    target_link_libraries(osim_emg_data_collection MEII)
endif()

if(EMG_RT_CTRL)
    #add_executable(emg_real_time_control src/emg_real_time_control.cpp)
    #target_link_libraries(emg_real_time_control MEII)

endif()

if(PHRI_LEARN)
    add_executable(phri_learning src/phri_learning.cpp)
    target_link_libraries(phri_learning MEII)
endif()

if (MYO)
    add_executable(myo_armband_ml src/myo_armband_ml.cpp)
    target_link_libraries(myo_armband_ml MEII)
endif()

if(EXAMPLES)
    add_subdirectory(examples)
endif()

if(TESTS)
    add_executable(testing tests/testing.cpp)
    target_link_libraries(testing MEII)
endif()
