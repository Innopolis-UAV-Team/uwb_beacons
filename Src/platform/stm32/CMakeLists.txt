# Copyright (C) 2025 Anastasiia Stepanova <asiiapine@gmail.com>
# Distributed under the terms of the GPL v3 license, available in the file LICENSE.

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

include(${ROOT_DIR}/cmake/toolchain-stm32f103xB.cmake)

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

add_executable(${EXECUTABLE}
    ${CMAKE_CURRENT_LIST_DIR}/led.cpp
    ${CMAKE_CURRENT_LIST_DIR}/usart_driver.cpp
    ${coreSources}
    ${driversSources}
    ${startupFile}
    ${middlewaresSources}
    ${PERIPHERAL_SOURCES}
    ${APPLICATION_SOURCES}
)

target_include_directories(${EXECUTABLE} PRIVATE
    ${CUBE_MX_HEADERS}
    ${PERIPHERAL_HEADERS}
    ${APPLICATION_HEADERS}
)

# Set linker flags after project definition
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${ldFile} -Wl,-Map=${PROJECT_NAME}.map,--cref")


# Post-build
add_custom_command(TARGET ${EXECUTABLE}
    POST_BUILD
    COMMAND ${CMAKE_SIZE} ${EXECUTABLE}
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${EXECUTABLE} ${BUILD_OBJ_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${EXECUTABLE} ${BUILD_OBJ_DIR}/${PROJECT_NAME}.bin
)
