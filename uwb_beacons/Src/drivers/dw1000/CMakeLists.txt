# Copyright (C) 2025 Anastasiia Stepanova <asiiapine@gmail.com>
# Distributed under the terms of the GPL v3 license, available in the file LICENSE.

# Include guard
if(SRC_DRIVERS_DW1000_DIR)
    return()
endif()
set(SRC_DRIVERS_DW1000_DIR ${CMAKE_CURRENT_LIST_DIR})

include(${ROOT_DIR}/Src/peripheral/spi/CMakeLists.txt)
include(${ROOT_DIR}/Src/drivers/uart_logger/CMakeLists.txt)

if(USE_ANCHOR)
    message("ANCHOR")
    set(DW1000_SRCS
        ${CMAKE_CURRENT_LIST_DIR}/anchor.cpp
    )
else()
    set(DW1000_SRCS
        ${CMAKE_CURRENT_LIST_DIR}/router.cpp
    )
endif()

list(APPEND APPLICATION_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/dw1000.cpp
    ${DW1000_SRCS}
)
