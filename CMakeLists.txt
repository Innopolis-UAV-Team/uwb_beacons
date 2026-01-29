# Copyright (C) 2025 Anastasiia Stepanova <asiiapine@gmail.com>
# Distributed under the terms of the GPL v3 license, available in the file LICENSE.

cmake_minimum_required(VERSION 3.15.3)

# Pathes
set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})
set(SRC_DIR ${ROOT_DIR}/Src)
set(CMAKE_DIR ${ROOT_DIR})

# Option 1. Choose the platform
option(USE_ANCHOR "Build for anchor" OFF)
option(USE_ROUTER "Build for rover" OFF)

string(ASCII 27 Esc)
set(Blue        "${Esc}[34m")
set(Yellow      "${Esc}[33m")
set(ColourReset "${Esc}[m")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Debug mode is enabled")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g -DDEBUG")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g -DDEBUG")
endif()

if(NOT DEFINED ID)
    set(ID "1")  # default value
endif()

if (TYPE STREQUAL "anchor")
    set(TARGET_NAME "anchor")
    set(USE_ANCHOR ON)
    message(STATUS "${Blue}ANCHOR type set${ColourReset}")
else()
    set(TARGET_NAME "router")
    message(STATUS "${Yellow}ROUTER type set${ColourReset}")
endif()

add_definitions(-DCALIBRATE=${CALIBRATE} -DPLATFORM=${PLATFORM} -DID=${ID})

if(NOT DEFINED BAUDRATE)
    set(BAUDRATE "230400")  # default value
endif()
add_definitions(-DBAUDRATE=${BAUDRATE})

message(STATUS "Setup for ${TYPE} with BAUDRATE=${BAUDRATE}")

# Set build dir based on hardware version and protocol
set(BUILD_ROOT_DIR ${ROOT_DIR}/build)
set(BUILD_TARGET_DIR ${BUILD_ROOT_DIR}/${TARGET_NAME})
set(BUILD_SRC_DIR ${BUILD_TARGET_DIR}/src)
set(BUILD_OBJ_DIR ${BUILD_TARGET_DIR}/obj)

list(APPEND APPLICATION_HEADERS ${ROOT_DIR}/Src)


# Project
project(${TARGET_NAME} CXX C ASM)
set(EXECUTABLE ${PROJECT_NAME})

# Include the application
include(${ROOT_DIR}/Src/CMakeLists.txt)

# Hardware version is not supported at the momemnt, let's hardcode it for a while
add_definitions(-DHW_VERSION_MAJOR=2)
add_definitions(-DHW_VERSION_MINOR=1)

message("PERIPHERAL_SOURCES ${PERIPHERAL_SOURCES}")
message("PERIPHERAL_HEADERS ${PERIPHERAL_HEADERS}")
