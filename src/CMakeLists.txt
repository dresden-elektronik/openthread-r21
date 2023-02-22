add_library(openthread-r21
    HAL/samr21Clock.c
    HAL/samr21NopDelay.c
    HAL/samr21Nvm.c
    HAL/samr21Rtc.c
    HAL/samr21Timer.c
    HAL/samr21Usb.c
    HAL/samr21Uart.c
    HAL/radio/samr21RadioAddrMatch.c
    HAL/radio/samr21RadioAes.c
    HAL/radio/samr21RadioCtrl.c
    HAL/radio/samr21RadioIrqHandler.c
    HAL/radio/samr21RadioEdHandler.c
    HAL/radio/samr21RadioRxHandler.c
    HAL/radio/samr21RadioTxHandler.c
    HAL/radio/samr21RadioVars.c
    HAL/radio/samr21TrxRegCopy.c
    HAL/radio/samr21Trx.c
    HAL/radio/samr21RadioFeCtrl.c
    OT-Utils/link_metrics.cpp
    OT-Utils/otUtilWrapper_macFrame.cpp
    OT-Utils/soft_source_match_table.c
    OT-HAL/otPlatAlarm.c
    OT-HAL/otPlatEntropy.c
    OT-HAL/otPlatRadio.c
    OT-HAL/otPlatSystem.c
    OT-HAL/otPlatUartUsb.c
    OT-HAL/otPlatLog.c
)

target_include_directories(openthread-r21
    PRIVATE
        ${OT_PUBLIC_INCLUDES}
        HAL
        HAL/include
        HAL/radio
        HAL/radio/include
        OT-HAL
        OT-HAL/include
        OT-Utils
        OT-Utils/include        
        ./
)

set_target_properties(openthread-r21
    PROPERTIES
        C_STANDARD 99
        CXX_STANDARD 11
)

if(OT_CFLAGS MATCHES "-pedantic-errors")
    string(REPLACE "-pedantic-errors" "" OT_CFLAGS "${OT_CFLAGS}")
endif()

target_link_libraries(openthread-r21
    PUBLIC
        samr21-CMSIS
        samr21-tusb
        -Wl,--gc-sections
        -Wl,-Map=$<TARGET_PROPERTY:NAME>.map
    PRIVATE
        ot-config
)

target_compile_definitions(openthread-r21
    PUBLIC
        ${OT_PLATFORM_DEFINES}
        -D__SAMR21E18A__ 
)

target_compile_options(openthread-r21
    PRIVATE
        -Wno-implicit-function-declaration
        -Wno-expansion-to-defined
        ${OT_CFLAGS}
)

