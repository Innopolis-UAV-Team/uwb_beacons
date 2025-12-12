# Copyright (C) 2023 Dmitry Ponomarev <ponomarevda96@gmail.com>
# Distributed under the terms of the GPL v3 license, available in the file LICENSE.

set(APPLICATION_DIR ${CMAKE_CURRENT_LIST_DIR})
cmake_path(GET CMAKE_CURRENT_LIST_DIR PARENT_PATH APPLICATIONS_DIR)
cmake_path(GET APPLICATIONS_DIR PARENT_PATH SRC_DIR)

include(${APPLICATION_DIR}/drivers/dw1000/CMakeLists.txt)
include(${APPLICATION_DIR}/drivers/uart_logger/CMakeLists.txt)

list(APPEND APPLICATION_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/application.cpp
    ${PERIPHERAL_SOURCES}
)

if (USE_ANCHOR)
    add_definitions(-DANCHOR)
else()
    add_definitions(-DROUTER)
endif()

include(${APPLICATION_DIR}/platform/${PLATFORM}/CMakeLists.txt)
