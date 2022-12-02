
CC=../gcc-arm-none-eabi/bin/arm-none-eabi-gcc
CXX=../gcc-arm-none-eabi/bin/arm-none-eabi-g++
ASM=../gcc-arm-none-eabi/bin/arm-none-eabi-as
GDB=../gcc-arm-none-eabi/bin/arm-none-eabi-gdb
MKDIR=mkdir

MACH=cortex-m0plus

OPT=-O0

OUT_DIR=../out/test

CMSIS_DEVICE_DIR=../third_party/CMSIS/r21
CMSIS_CORE_DIR=../third_party/CMSIS/core

TINYUSB_OBJ_DIR=../out/tinyUSB
TINYUSB_INC_DIR=../third_party/tinyusb/src

INCDIRS=\
	-I. \
	-I$(CMSIS_DEVICE_DIR)/include \
	-I$(CMSIS_DEVICE_DIR)/source \
	-I$(CMSIS_CORE_DIR) \
	-I$(TINYUSB_INC_DIR)
	
DEFINES= -D__SAMR21E18A__

CFLAGS= \
	-c \
	-g \
	-mcpu=$(MACH) \
	--specs=nosys.specs \
	-mfloat-abi=soft \
	-mthumb \
	-std=gnu99 \
	-Wall \
	$(OPT) \
	$(INCDIRS) \
	$(DEFINES) 

LINKERSCRIPT=./ASF/samr21e18a_flash.ld


tinyUSB:\
	$(TINYUSB_OBJ_DIR) \
	$(TINYUSB_OBJ_DIR)/tusb.o \
	$(TINYUSB_OBJ_DIR)/usbd.o \
	$(TINYUSB_OBJ_DIR)/usbd_control.o \
	$(TINYUSB_OBJ_DIR)/tusb_fifo.o \
	$(TINYUSB_OBJ_DIR)/cdc_device.o \
	$(TINYUSB_OBJ_DIR)/usb_descriptors.o \
	$(TINYUSB_OBJ_DIR)/dcd_samr.o
	
	cd ../third_party && make tinyUSB
	
	 	
$(OUT_DIR):
	$(MKDIR) -p $@



clean:
	rm -rf $(OUT_DIR)

