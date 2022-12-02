
CC=../gcc-arm-none-eabi/bin/arm-none-eabi-gcc
CXX=../gcc-arm-none-eabi/bin/arm-none-eabi-g++
ASM=../gcc-arm-none-eabi/bin/arm-none-eabi-as
GDB=../gcc-arm-none-eabi/bin/arm-none-eabi-gdb
MKDIR=mkdir

MACH=cortex-m0plus

OPT=-O0

OUT_DIR=../out/samr21HAL

CMSIS_DEVICE_DIR=../third_party/CMSIS/r21
CMSIS_CORE_DIR=../third_party/CMSIS/core

AT86RF233_INC_DIR=../third_party/ATMEL/AT86RF233

HAL_DIR=./HAL
HAL_INC_DIR=./HAL/include

INCDIRS=\
	-I. \
	-I$(CMSIS_DEVICE_DIR)/include \
	-I$(CMSIS_DEVICE_DIR)/source \
	-I$(CMSIS_CORE_DIR) \
	-I$(AT86RF233_INC_DIR) \
	-I$(HAL_INC_DIR) 
	
DEFINES= -D__SAMR21E18A__ -D_DEBUG

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



samr21HAL:\
	$(OUT_DIR) \
	$(OUT_DIR)/samr21PowerManager.o \
	$(OUT_DIR)/samr21Radio.o \
	$(OUT_DIR)/samr21RadioFSM.o \
	$(OUT_DIR)/samr21Timer.o \
	$(OUT_DIR)/samr21NopDelay.o \
	$(OUT_DIR)/samr21Rtc.o \
	$(OUT_DIR)/samr21Trx.o \
	$(OUT_DIR)/samr21Clock.o 

	
$(OUT_DIR):
	$(MKDIR) -p $@


$(OUT_DIR)/samr21PowerManager.o:\
	$(HAL_DIR)/samr21PowerManager.c 
	
	$(CC) $(CFLAGS) -o $@ $^

$(OUT_DIR)/samr21Radio.o:\
	$(HAL_DIR)/samr21Radio.c 

	$(CC) $(CFLAGS) -o $@ $^

$(OUT_DIR)/samr21RadioFSM.o:\
	$(HAL_DIR)/samr21RadioFSM.c 

	$(CC) $(CFLAGS) -o $@ $^

$(OUT_DIR)/samr21Timer.o:\
	$(HAL_DIR)/samr21Timer.c 

	$(CC) $(CFLAGS) -o $@ $^
	
$(OUT_DIR)/samr21NopDelay.o:\
	$(HAL_DIR)/samr21NopDelay.c 

	$(CC) $(CFLAGS) -o $@ $^

$(OUT_DIR)/samr21Rtc.o:\
	$(HAL_DIR)/samr21Rtc.c 

	$(CC) $(CFLAGS) -o $@ $^

$(OUT_DIR)/samr21Trx.o:\
	$(HAL_DIR)/samr21Trx.c 

	$(CC) $(CFLAGS) -o $@ $^
	
$(OUT_DIR)/samr21Clock.o:\
	$(HAL_DIR)/samr21Clock.c 

	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -rf $(OUT_DIR)

