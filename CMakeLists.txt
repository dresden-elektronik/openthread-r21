
cmake_minimum_required(VERSION 3.10.2)
project(ot-samr21 VERSION 0.0.1)

set(OT_PLATFORM_LIB "openthread-r21")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)


add_subdirectory(openthread)

target_compile_definitions(ot-config INTERFACE
    OPENTHREAD_CONFIG_FILE="${PROJECT_SOURCE_DIR}/src/openthread-core-r21-config.h"
    OPENTHREAD_PROJECT_CORE_CONFIG_FILE="${PROJECT_SOURCE_DIR}/src/openthread-core-r21-config.h"
    OPENTHREAD_CORE_CONFIG_PLATFORM_CHECK_FILE="${PROJECT_SOURCE_DIR}/src/openthread-core-r21-config-check.h"
)

target_include_directories(ot-config INTERFACE
    ${PROJECT_SOURCE_DIR}/src/OT-HAL/
    ${PROJECT_SOURCE_DIR}/src/OT-HAL/include/

    ${PROJECT_SOURCE_DIR}/src/HAL/include/
    ${PROJECT_SOURCE_DIR}/src/HAL/radio/include/
)

add_subdirectory(third_party)
add_subdirectory(src)