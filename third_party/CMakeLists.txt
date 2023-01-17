
add_library(samr21-CMSIS
    CMSIS/r21/source/system_samr21.c
    CMSIS/r21/source/gcc/startup_samr21.c
)

add_library(samr21-tusb
    tinyusb/src/tusb.c 
    tinyusb/src/device/usbd.c
    tinyusb/src/device/usbd_control.c 
    tinyusb/src/common/tusb_fifo.c 
    tinyusb/src/class/cdc/cdc_device.c 
    tinyusb_conf/usb_descriptors.c 
    tinyusb_modified/src/portable/microchip/samr/dcd_samr.c
)

target_include_directories(samr21-tusb
    PUBLIC
        tinyusb/src
        tinyusb_conf
)

target_include_directories(samr21-CMSIS
    PUBLIC
        CMSIS/r21/source
        CMSIS/r21/include
        CMSIS/core
        ATMEL/LINKER
        ATMEL/AT86RF233
)

if(OT_CFLAGS MATCHES "-pedantic-errors")
    string(REPLACE "-pedantic-errors" "" OT_CFLAGS "${OT_CFLAGS}")
endif()

target_link_libraries(samr21-CMSIS
    PUBLIC
        -T${PROJECT_SOURCE_DIR}/third_party/ATMEL/LINKER/samr21e18a_flash.ld
        samr21-tusb
    PRIVATE
        ot-config
)

target_link_libraries(samr21-tusb
    PUBLIC
        samr21-CMSIS
    PRIVATE
        ot-config
)

target_compile_definitions(samr21-CMSIS
    PUBLIC
        ${OT_PLATFORM_DEFINES}
        -D__SAMR21G18A__
)

target_compile_definitions(samr21-tusb
    PUBLIC
        ${OT_PLATFORM_DEFINES}
        -D__SAMR21G18A__
)