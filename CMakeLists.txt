# MIT License
#
# Copyright (c) 2020 Mechatronics and Haptic Interfaces Lab - Rice University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# Author(s): Nathan Dunkelberger (nbd2@rice.edu)

cmake_minimum_required(VERSION 3.13.0)

# Options
if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    option(MEII_EXAMPLES "Turn ON to build example executable(s)" ON)
else()
    option(MEII_EXAMPLES "Turn ON to build example executable(s)" OFF)
endif()

# create project
project(mahiexoii VERSION 0.1.0 LANGUAGES CXX)

# defines conventional GNU isntallation directories
include(GNUInstallDirs)

# set compiler flags
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O3") # all warnings
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /W3") # warning level 4
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /MP") # multicore build
endif()

# Use fetch content to get libraries that meii is dependent on
include(FetchContent)

# MAHI DAQ
FetchContent_Declare(mahi-daq GIT_REPOSITORY https://github.com/mahilab/mahi-daq.git)
FetchContent_MakeAvailable(mahi-daq)

# MAHI ROBO
FetchContent_Declare(mahi-robo GIT_REPOSITORY https://github.com/mahilab/mahi-robo.git)
FetchContent_MakeAvailable(mahi-robo)

# MAHI COM
FetchContent_Declare(mahi-com GIT_REPOSITORY https://github.com/mahilab/mahi-com.git)
FetchContent_MakeAvailable(mahi-com)

# add definitons
add_definitions(-D_CRT_SECURE_NO_WARNINGS -DNOMINMAX -D_WINSOCK_DEPRECATED_NO_WARNINGS)

set(SRC_MEII 
    src/MEII/Control/DisturbanceObserver.cpp
    # src/MEII/Control/DynamicMotionPrimitive.cpp
    # src/MEII/Control/MinimumJerk.cpp
    # src/MEII/Control/Trajectory.cpp
    # src/MEII/Control/Waypoint.cpp
    src/MEII/MahiExoII/Joint.cpp
    src/MEII/MahiExoII/JointHardware.cpp
    src/MEII/MahiExoII/JointVirtual.cpp
    src/MEII/MahiExoII/MahiExoII.cpp
    # src/MEII/MahiExoII/MahiExoIIHardware.cpp
    src/MEII/MahiExoII/MahiExoIIVirtual.cpp)

file(GLOB_RECURSE INC_MEII "include/*.hpp")

add_library(meii src/MEII/Control/DisturbanceObserver.cpp)
add_library(meii::meii ALIAS meii)
set_target_properties(meii PROPERTIES DEBUG_POSTFIX -d)
target_compile_features(meii PUBLIC cxx_std_11)
set_target_properties(meii PROPERTIES OUTPUT_NAME meii)

target_compile_features(meii PUBLIC cxx_std_11)
install(TARGETS meii EXPORT meii-targets LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

target_sources(meii PRIVATE ${SRC_MEII} ${INC_MEII})

target_include_directories(meii
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(meii mahi::daq mahi::robo mahi::com)

if(MEII_EXAMPLES)
    message("Building MEII examples")
    add_subdirectory(examples)
endif()