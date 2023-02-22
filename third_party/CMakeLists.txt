
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

if(DDEL_GCF_BUILD)
    target_link_libraries(samr21-CMSIS
        PUBLIC
            -T${PROJECT_SOURCE_DIR}/third_party/ATMEL/LINKER/samr21e18a_flash_gcf_offset.ld
        PRIVATE
            ot-config
    )


    target_compile_definitions(samr21-CMSIS
        PUBLIC
            ${OT_PLATFORM_DEFINES}
            -D__SAMR21G18A__
            -D_GCF_RELEASE_
    )
else()
    target_link_libraries(samr21-CMSIS
        PUBLIC
            -T${PROJECT_SOURCE_DIR}/third_party/ATMEL/LINKER/samr21e18a_flash.ld
        PRIVATE
            ot-config
    )

    target_compile_definitions(samr21-CMSIS
        PUBLIC
            ${OT_PLATFORM_DEFINES}
            -D__SAMR21G18A__
    )
endif()



target_link_libraries(samr21-tusb
    PRIVATE
        samr21-CMSIS
        ot-config
        openthread-r21
)



target_compile_definitions(samr21-tusb
    PUBLIC
        ${OT_PLATFORM_DEFINES}
        -D__SAMR21G18A__
)
