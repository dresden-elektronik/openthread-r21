cmake_minimum_required(VERSION 3.10.2)
project(ot-samr21 VERSION 0.0.1)

set(OT_PLATFORM_LIB "openthread-r21")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

set(CMAKE_TOOLCHAIN_FILE src/arm-none-eabi.cmake)

add_subdirectory(openthread)

target_compile_definitions(ot-config INTERFACE
    OPENTHREAD_CONFIG_FILE="openthread-core-r21-config.h"
    OPENTHREAD_PROJECT_CORE_CONFIG_FILE="openthread-core-r21-config.h"
    OPENTHREAD_CORE_CONFIG_PLATFORM_CHECK_FILE="openthread-core-r21-config-check.h"
)

target_include_directories(ot-config INTERFACE
    ${PROJECT_SOURCE_DIR}/src
)




add_subdirectory(src)
add_subdirectory(third_party)