
CC=../gcc-arm-none-eabi/bin/arm-none-eabi-gcc
CXX=../gcc-arm-none-eabi/bin/arm-none-eabi-g++
ASM=../gcc-arm-none-eabi/bin/arm-none-eabi-as
GDB=../gcc-arm-none-eabi/bin/arm-none-eabi-gdb
MKDIR=mkdir

MACH=cortex-m0plus

OPT=-O0

HAL_OUT_DIR=../out/samr21HAL
OT_PLAT_HAL_OUT_DIR=../out/otPlatHAL

CMSIS_DEVICE_DIR=../third_party/CMSIS/r21
CMSIS_CORE_DIR=../third_party/CMSIS/core

OT_INC_DIR=../openthread/include
OT_PLATFORM_INC_DIR=../openthread/examples/platforms

TINYUSB_INC_DIR=../third_party/tinyusb/src
TINYUSB_CONF_DIR=../third_party/tinyusb_config

AT86RF233_INC_DIR=../third_party/ATMEL/AT86RF233

HAL_DIR=./HAL
HAL_INC_DIR=./HAL/include

OT_PLAT_HAL_DIR=./OT-HAL
OT_PLAT_HAL_INC_DIR=./OT-HAL/include

INCDIRS=\
	-I. \
	-I$(CMSIS_DEVICE_DIR)/include \
	-I$(CMSIS_DEVICE_DIR)/source \
	-I$(CMSIS_CORE_DIR) \
	-I$(AT86RF233_INC_DIR) \
	-I$(HAL_INC_DIR) \
	-I$(OT_INC_DIR) \
	-I$(TINYUSB_INC_DIR) \
	-I$(TINYUSB_CONF_DIR) \
	-I$(OT_PLATFORM_INC_DIR) \
	-I$(OT_PLAT_HAL_INC_DIR)

	
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

all:\
	samr21HAL \
	otPlatHAL

samr21HAL:\
	$(HAL_OUT_DIR) \
	$(HAL_OUT_DIR)/samr21PowerManager.o \
	$(HAL_OUT_DIR)/samr21Radio.o \
	$(HAL_OUT_DIR)/samr21RadioFSM.o \
	$(HAL_OUT_DIR)/samr21Timer.o \
	$(HAL_OUT_DIR)/samr21NopDelay.o \
	$(HAL_OUT_DIR)/samr21Rtc.o \
	$(HAL_OUT_DIR)/samr21Trx.o \
	$(HAL_OUT_DIR)/samr21Nvm.o \
	$(HAL_OUT_DIR)/samr21Usb.o \
	$(HAL_OUT_DIR)/samr21Aes.o \
	$(HAL_OUT_DIR)/samr21Clock.o 

otPlatHAL:\
	$(OT_PLAT_HAL_OUT_DIR) \
	$(OT_PLAT_HAL_OUT_DIR)/otPlatAlarm.o \
	$(OT_PLAT_HAL_OUT_DIR)/otPlatEntropy.o \
	$(OT_PLAT_HAL_OUT_DIR)/otPlatUartUsb.o \
	$(OT_PLAT_HAL_OUT_DIR)/otPlatRadio.o

	
$(HAL_OUT_DIR):
	$(MKDIR) -p $@

$(HAL_OUT_DIR)/samr21PowerManager.o:\
	$(HAL_DIR)/samr21PowerManager.c 
	
	$(CC) $(CFLAGS) -o $@ $^

$(HAL_OUT_DIR)/samr21Radio.o:\
	$(HAL_DIR)/samr21Radio.c 

	$(CC) $(CFLAGS) -o $@ $^

$(HAL_OUT_DIR)/samr21RadioFSM.o:\
	$(HAL_DIR)/samr21RadioFSM.c 

	$(CC) $(CFLAGS) -o $@ $^

$(HAL_OUT_DIR)/samr21Timer.o:\
	$(HAL_DIR)/samr21Timer.c 

	$(CC) $(CFLAGS) -o $@ $^
	
$(HAL_OUT_DIR)/samr21NopDelay.o:\
	$(HAL_DIR)/samr21NopDelay.c 

	$(CC) $(CFLAGS) -o $@ $^

$(HAL_OUT_DIR)/samr21Rtc.o:\
	$(HAL_DIR)/samr21Rtc.c 

	$(CC) $(CFLAGS) -o $@ $^

$(HAL_OUT_DIR)/samr21Trx.o:\
	$(HAL_DIR)/samr21Trx.c 

	$(CC) $(CFLAGS) -o $@ $^

$(HAL_OUT_DIR)/samr21Nvm.o:\
	$(HAL_DIR)/samr21Nvm.c 

	$(CC) $(CFLAGS) -o $@ $^

$(HAL_OUT_DIR)/samr21Usb.o:\
	$(HAL_DIR)/samr21Usb.c 

	$(CC) $(CFLAGS) -o $@ $^
	
$(HAL_OUT_DIR)/samr21Clock.o:\
	$(HAL_DIR)/samr21Clock.c 

	$(CC) $(CFLAGS) -o $@ $^

$(HAL_OUT_DIR)/samr21Aes.o:\
	$(HAL_DIR)/samr21Aes.c 

	$(CC) $(CFLAGS) -o $@ $^


$(OT_PLAT_HAL_OUT_DIR):
	$(MKDIR) -p $@

$(OT_PLAT_HAL_OUT_DIR)/otPlatAlarm.o:\
	$(OT_PLAT_HAL_DIR)/otPlatAlarm.c 
	
	$(CC) $(CFLAGS) -o $@ $^

$(OT_PLAT_HAL_OUT_DIR)/otPlatEntropy.o:\
	$(OT_PLAT_HAL_DIR)/otPlatEntropy.c 
	
	$(CC) $(CFLAGS) -o $@ $^

$(OT_PLAT_HAL_OUT_DIR)/otPlatSystem.o:\
	$(OT_PLAT_HAL_DIR)/otPlatSystem.c 
	
	$(CC) $(CFLAGS) -o $@ $^

$(OT_PLAT_HAL_OUT_DIR)/otPlatUartUsb.o:\
	$(OT_PLAT_HAL_DIR)/otPlatUartUsb.c 
	
	$(CC) $(CFLAGS) -o $@ $^

$(OT_PLAT_HAL_OUT_DIR)/otPlatRadio.o:\
	$(OT_PLAT_HAL_DIR)/otPlatRadio.c 
	
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -rf $(HAL_OUT_DIR)
	rm -rf $(OT_PLAT_HAL_OUT_DIR)

