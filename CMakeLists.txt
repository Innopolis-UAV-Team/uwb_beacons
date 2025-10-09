cmake_minimum_required(VERSION 3.15.3)

# Pathes
set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})
set(SRC_DIR ${ROOT_DIR}/Src)
set(CMAKE_DIR ${ROOT_DIR})

# Option 1. Choose the platform
option(USE_ANCHOR "Build for anchor" OFF)
option(USE_ROUTER "Build for rover" OFF)

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
add_definitions(-DID=${ID})

if (USE_ANCHOR)
    set(TARGET_NAME "anchor")
    add_definitions(-DANCHOR)
else()
    set(TARGET_NAME "router")
    add_definitions(-DROUTER)
endif()

if(NOT DEFINED BAUDRATE)
    set(BAUDRATE "230400")  # default value
endif()
add_definitions(-DBAUDRATE=${BAUDRATE})

message(STATUS "Building for ROUTER with BAUDRATE=${BAUDRATE}")

set(stm32cubeMxProjectPath ${ROOT_DIR}/stm32-ioc)
FILE(GLOB ldFile            ${stm32cubeMxProjectPath}/*_FLASH.ld)
FILE(GLOB coreSources       ${stm32cubeMxProjectPath}/Core/Src/*)
FILE(GLOB driversSources    ${stm32cubeMxProjectPath}/Drivers/*/*.c
                            ${stm32cubeMxProjectPath}/Drivers/*/*/*.c
                            ${stm32cubeMxProjectPath}/Drivers/*/*/*/*.c)
FILE(GLOB startupFile       ${stm32cubeMxProjectPath}/*.s
                            ${stm32cubeMxProjectPath}/Core/Startup/*.s
)
FILE(GLOB middlewaresSources ${stm32cubeMxProjectPath}/Drivers/Middlewares/*/*/*.c)

include(${ROOT_DIR}/toolchain-stm32f103xB.cmake)

# Set build dir based on hardware version and protocol
set(BUILD_ROOT_DIR ${ROOT_DIR}/build)
set(BUILD_TARGET_DIR ${BUILD_ROOT_DIR}/${TARGET_NAME})
set(BUILD_SRC_DIR ${BUILD_TARGET_DIR}/src)
set(BUILD_OBJ_DIR ${BUILD_TARGET_DIR}/obj)

list(APPEND APPLICATION_HEADERS ${ROOT_DIR}/Src)

list(APPEND CUBE_MX_HEADERS
                ${stm32cubeMxProjectPath}/Core/Inc
                ${stm32cubeMxProjectPath}/Drivers/API
                ${stm32cubeMxProjectPath}/Drivers/CMSIS/Include
                ${stm32cubeMxProjectPath}/Drivers/Middlewares/decawave/compiler
                ${stm32cubeMxProjectPath}/Drivers/Middlewares/decawave/decadriver
                ${stm32cubeMxProjectPath}/Drivers/Middlewares/decawave/platform
                ${stm32cubeMxProjectPath}/Drivers/Middlewares/SEGGER_RTT/Config
                ${stm32cubeMxProjectPath}/Drivers/Middlewares/SEGGER_RTT/Syscalls
                ${stm32cubeMxProjectPath}/Drivers/Middlewares/SEGGER_RTT/RTT
                ${stm32cubeMxProjectPath}/Drivers/CMSIS/Device/ST/STM32F1xx/Include
                ${stm32cubeMxProjectPath}/Drivers/STM32F1xx_HAL_Driver/Inc
                ${stm32cubeMxProjectPath}/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy)

# Include the application
include(${ROOT_DIR}/Src/CMakeLists.txt)

# Hardware version is not supported at the momemnt, let's hardcode it for a while
add_definitions(-DHW_VERSION_MAJOR=2)
add_definitions(-DHW_VERSION_MINOR=1)

# Project
project(${TARGET_NAME} CXX C ASM)

# Set linker flags after project definition
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${ldFile} -Wl,-Map=${PROJECT_NAME}.map,--cref")

set(EXECUTABLE ${PROJECT_NAME})
message("PERIPHERAL_SOURCES ${PERIPHERAL_SOURCES}")
message("PERIPHERAL_HEADERS ${PERIPHERAL_HEADERS}")

add_executable(${EXECUTABLE}
    ${coreSources}
    ${driversSources}
    ${startupFile}
    ${middlewaresSources}
    ${PERIPHERAL_SOURCES}
    ${APPLICATION_SOURCES}
)

target_include_directories(${EXECUTABLE} PUBLIC
    ${CUBE_MX_HEADERS}
    ${PERIPHERAL_HEADERS}
    ${APPLICATION_HEADERS}
)

# Post-build
add_custom_command(TARGET ${EXECUTABLE}
    POST_BUILD
    COMMAND ${CMAKE_SIZE} ${EXECUTABLE}
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${EXECUTABLE} ${BUILD_OBJ_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${EXECUTABLE} ${BUILD_OBJ_DIR}/${PROJECT_NAME}.bin
)
