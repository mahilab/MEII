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
include(GNUInstallDirs)

# create project
project(MEII VERSION 0.1.0 LANGUAGES CXX)

# Enable IDE folders and set them for predefined CMake projects
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMake")

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

#===============================================================================
# FIND PROJECT SOURCE FILES
#===============================================================================

include(MEIISources)

#===============================================================================
# CREATE MEII LIBRARY AND LINK EXTERNAL LIBS
#===============================================================================

add_library(MEII STATIC "")
target_compile_definitions(MEII PUBLIC)
add_library(MEII::MEII ALIAS MEII)
target_sources(MEII PRIVATE ${MEII_SOURCE_FILES} ${MEII_HEADER_FILES})

# add <project>/include as public include directory
target_include_directories(MEII
    PUBLIC
        $<INSTALL_INTERFACE:include>
        ${EIGEN_ROOT}
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(MEII MEL::quanser)

#===============================================================================
# Install
#===============================================================================

# install the library
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/MEII)
install(TARGETS MEII
	EXPORT MEII-targets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

set_target_properties(MEII PROPERTIES EXPORT_NAME MEII)
set_target_properties(MEII PROPERTIES FOLDER "MEII")

# install headers
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# set where we want to install our config
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/MEII)

# export the targets to a script
install(EXPORT MEII-targets
	FILE
		MEIITargets.cmake
	# NAMESPACE
	# 	MEII::
	DESTINATION
		${INSTALL_CONFIGDIR}
)

# include helper functions for creating config files that can be included by other projects to find and use a package
include(CMakePackageConfigHelpers)

# generate a package configuration file and a package version file
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/MEIIConfig.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/MEIIConfig.cmake
	INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)
write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/MEIIConfigVersion.cmake
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY AnyNewerVersion
)

# install the config and configversion modules
install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/MEIIConfig.cmake
	${CMAKE_CURRENT_BINARY_DIR}/MEIIConfigVersion.cmake
	DESTINATION ${INSTALL_CONFIGDIR}
)

# export from the build tree
export(EXPORT MEII-targets
	   FILE ${CMAKE_CURRENT_BINARY_DIR}/MEIITargets.cmake)

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
