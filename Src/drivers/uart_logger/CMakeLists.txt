# Copyright (C) 2025 Anastasiia Stepanova <asiiapine@gmail.com>
# Distributed under the terms of the GPL v3 license, available in the file LICENSE.

# Include guard
if(SRC_DRIVERS_LOGGER_DIR)
    return()
endif()
set(SRC_DRIVERS_LOGGER_DIR ${CMAKE_CURRENT_LIST_DIR})

list(APPEND APPLICATION_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/logger.cpp
)
include(${ROOT_DIR}/Src/peripheral/usart/CMakeLists.txt)

message("PERIPHERAL ${PERIPHERAL_SOURCES}")
