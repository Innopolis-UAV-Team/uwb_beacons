# Copyright (C) 2025 Anastasiia Stepanova <asiiapine@gmail.com>
# Distributed under the terms of the GPL v3 license, available in the file LICENSE.

# Include guard
if(PERIPHERAL_USART_CMAKE)
    return()
endif()
set(PERIPHERAL_USART_CMAKE ${CMAKE_CURRENT_LIST_DIR})
list(APPEND PERIPHERAL_SOURCES ${CMAKE_CURRENT_LIST_DIR}/usart.c)
list(APPEND PERIPHERAL_HEADERS ${CMAKE_CURRENT_LIST_DIR})
