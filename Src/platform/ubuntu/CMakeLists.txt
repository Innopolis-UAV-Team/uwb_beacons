include(${ROOT_DIR}/cmake/toolchain-ubuntu.cmake)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")

list(APPEND PERIPHERAL_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/led.cpp
    ${CMAKE_CURRENT_LIST_DIR}/usart_driver.c
)

add_executable(${EXECUTABLE}
    ${CMAKE_CURRENT_LIST_DIR}/main.c
    ${PERIPHERAL_SOURCES}
    ${APPLICATION_SOURCES}
)

target_include_directories(${EXECUTABLE} PRIVATE
    ${CUBE_MX_HEADERS}
    ${PERIPHERAL_HEADERS}
    ${APPLICATION_HEADERS}
    ${CMAKE_CURRENT_LIST_DIR}
)
