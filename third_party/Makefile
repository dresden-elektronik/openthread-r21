
CC=../gcc-arm-none-eabi/bin/arm-none-eabi-gcc
CXX=../gcc-arm-none-eabi/bin/arm-none-eabi-g++
ASM=../gcc-arm-none-eabi/bin/arm-none-eabi-as
GDB=../gcc-arm-none-eabi/bin/arm-none-eabi-gdb

MACH=cortex-m0plus

OPT=-O0

OUT_DIR=../out/tinyUSB

CMSIS_DEVICE_DIR=./CMSIS/r21
CMSIS_CORE_DIR=./CMSIS/core

TINYUSB_SRC_DIR=./tinyusb/src
TINYUSB_MOD_SRC_DIR=./tinyusb_modified/src
TINYUSB_CONF_DIR=./tinyusb_config

INCDIRS= -I. -I$(CMSIS_DEVICE_DIR)/include -I$(CMSIS_DEVICE_DIR)/source -I$(CMSIS_CORE_DIR) -I$(TINYUSB_SRC_DIR) -I$(TINYUSB_CONF_DIR)
DEFINES= -D__SAMR21E18A__

CFLAGS= -c -g -mcpu=$(MACH) --specs=nosys.specs -mfloat-abi=soft -mthumb -std=gnu99 -Wall $(OPT) $(INCDIRS) $(DEFINES) 

MKDIR=mkdir

tinyUSB:\
	$(OUT_DIR) \
	$(OUT_DIR)/tusb.o \
	$(OUT_DIR)/usbd.o \
	$(OUT_DIR)/usbd_control.o \
	$(OUT_DIR)/tusb_fifo.o \
	$(OUT_DIR)/cdc_device.o \
	$(OUT_DIR)/usb_descriptors.o \
	$(OUT_DIR)/dcd_samr.o 

	
$(OUT_DIR):
	$(MKDIR) -p $@

$(OUT_DIR)/tusb.o:\
	$(TINYUSB_SRC_DIR)/tusb.c 
	
	$(CC) $(CFLAGS) -o $@ $^

$(OUT_DIR)/usbd.o:\
	$(TINYUSB_SRC_DIR)/device/usbd.c 

	$(CC) $(CFLAGS) -o $@ $^

$(OUT_DIR)/usbd_control.o:\
	$(TINYUSB_SRC_DIR)/device/usbd_control.c 

	$(CC) $(CFLAGS) -o $@ $^

$(OUT_DIR)/tusb_fifo.o:\
	$(TINYUSB_SRC_DIR)/common/tusb_fifo.c 

	$(CC) $(CFLAGS) -o $@ $^

$(OUT_DIR)/cdc_device.o:\
	$(TINYUSB_SRC_DIR)/class/cdc/cdc_device.c 

	$(CC) $(CFLAGS) -o $@ $^

$(OUT_DIR)/usb_descriptors.o:\
	$(TINYUSB_CONF_DIR)/usb_descriptors.c 

	$(CC) $(CFLAGS) -o $@ $^

$(OUT_DIR)/dcd_samr.o:\
	$(TINYUSB_MOD_SRC_DIR)/portable/microchip/samr/dcd_samr.c

	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -rf $(OUT_DIR)

