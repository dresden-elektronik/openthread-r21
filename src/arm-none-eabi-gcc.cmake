set(CMAKE_SYSTEM_NAME              Generic)
set(CMAKE_SYSTEM_PROCESSOR         ARM)

set(CMAKE_C_COMPILER               "${PROJECT_SOURCE_DIR}/gcc-arm-none-eabi/bin/arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER             "${PROJECT_SOURCE_DIR}/gcc-arm-none-eabi/bin/arm-none-eabi-g++")
set(CMAKE_ASM_COMPILER             "${PROJECT_SOURCE_DIR}/gcc-arm-none-eabi/bin/arm-none-eabi-as")
set(CMAKE_RANLIB                   "${PROJECT_SOURCE_DIR}/gcc-arm-none-eabi/bin/arm-none-eabi-ranlib")

set(COMMON_C_FLAGS                 "-mcpu=cortex-m0plus -mfloat-abi=soft -mthumb")

set(CMAKE_C_FLAGS_INIT             "${COMMON_C_FLAGS} -std=gnu99")
set(CMAKE_CXX_FLAGS_INIT           "${COMMON_C_FLAGS} -fno-exceptions -fno-rtti")
set(CMAKE_ASM_FLAGS_INIT           "${COMMON_C_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_INIT    "${COMMON_C_FLAGS} -specs=nano.specs -specs=nosys.specs -Wl,--entry=Reset_Handler")
