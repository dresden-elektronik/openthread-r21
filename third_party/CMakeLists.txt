
add_library(samr21-CMSIS
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/r21/source/system_samr21.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/r21/source/gcc/startup_samr21.c
)

add_library(samr21-tusb
    ${PROJECT_SOURCE_DIR}/third_party/tinyusb/src/tusb.c 
    ${PROJECT_SOURCE_DIR}/third_party/tinyusb/src/device/usbd.c
    ${PROJECT_SOURCE_DIR}/third_party/tinyusb/src/device/usbd_control.c 
    ${PROJECT_SOURCE_DIR}/third_party/tinyusb/src/common/tusb_fifo.c 
    ${PROJECT_SOURCE_DIR}/third_party/tinyusb/src/class/cdc/cdc_device.c 
    ${PROJECT_SOURCE_DIR}/third_party/tinyusb_config/usb_descriptors.c 
    ${PROJECT_SOURCE_DIR}/third_party/tinyusb_modified/src/portable/microchip/samr/dcd_samr.c
)

target_include_directories(samr21-tusb
    PUBLIC
        ${PROJECT_SOURCE_DIR}/third_party/tinyusb/src
        ${PROJECT_SOURCE_DIR}/third_party/tinyusb_config
)

target_include_directories(samr21-CMSIS
    PUBLIC
        ${PROJECT_SOURCE_DIR}/third_party/CMSIS/r21/source
        ${PROJECT_SOURCE_DIR}/third_party/CMSIS/r21/include
        ${PROJECT_SOURCE_DIR}/third_party/CMSIS/core
        ${PROJECT_SOURCE_DIR}/third_party/ATMEL/LINKER
        ${PROJECT_SOURCE_DIR}/third_party/ATMEL/AT86RF233
)

if(OT_CFLAGS MATCHES "-pedantic-errors")
    string(REPLACE "-pedantic-errors" "" OT_CFLAGS "${OT_CFLAGS}")
endif()


set(SAMR21_PLATFORM_DEFINES "__SAMR21G18A__")

if(TARGET_DEVICE STREQUAL  "CONBEE2")
    list(APPEND SAMR21_PLATFORM_DEFINES "TARGET_DEVICE=CONBEE2")
elseif(TARGET_DEVICE STREQUAL "RASPBEE2")
    list(APPEND SAMR21_PLATFORM_DEFINES "TARGET_DEVICE=RASPBEE2")
endif()

if(DDEL_GCF_BUILD)
    message("Building with GCF Offset")
    target_link_libraries(samr21-CMSIS
        PUBLIC
            -T${PROJECT_SOURCE_DIR}/third_party/ATMEL/LINKER/samr21e18a_flash_gcf_offset.ld
        PRIVATE
            ot-config
    )
    list(APPEND SAMR21_PLATFORM_DEFINES "GCF_BUILD=1")
else()
    target_link_libraries(samr21-CMSIS
        PUBLIC
            -T${PROJECT_SOURCE_DIR}/third_party/ATMEL/LINKER/samr21e18a_flash.ld
        PRIVATE
            ot-config
     )
endif()

target_compile_definitions(samr21-CMSIS
    PUBLIC
        ${OT_PLATFORM_DEFINES}
        ${SAMR21_PLATFORM_DEFINES}
)

target_link_libraries(samr21-tusb
    PRIVATE
        samr21-CMSIS
        ot-config
        openthread-r21
)

message("OT DEFINES ${OT_PLATFORM_DEFINES}")
message("TARGET_DEVICE ${TARGET_DEVICE}")
message("PLATFORM DEFINES ${SAMR21_PLATFORM_DEFINES}")

target_compile_definitions(samr21-tusb
    PUBLIC
        ${OT_PLATFORM_DEFINES}
        ${SAMR21_PLATFORM_DEFINES}
)
