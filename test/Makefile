
CC=../gcc-arm-none-eabi/bin/arm-none-eabi-gcc
CXX=../gcc-arm-none-eabi/bin/arm-none-eabi-g++
ASM=../gcc-arm-none-eabi/bin/arm-none-eabi-as
GDB=../gcc-arm-none-eabi/bin/arm-none-eabi-gdb

OCD=openocd
MKDIR=mkdir

MACH=cortex-m0plus

OPT=-O0

OBJ_OUT_DIR=../out/test
IMAGE_OUT_DIR=../out/images

CMSIS_DEVICE_DIR=../third_party/CMSIS/r21
CMSIS_CORE_DIR=../third_party/CMSIS/core

TINYUSB_INC_DIR=../third_party/tinyusb/src
TINYUSB_CONF_DIR=../third_party/tinyusb_config

HAL_INC_DIR=../src/HAL/include

AT86RF233_INC_DIR=../third_party/ATMEL/AT86RF233

TINYUSB_OBJS=$(wildcard ../out/tinyUSB/*.o)
CMSIS_OBJS=$(wildcard ../out/CMSIS_Startup/*.o)
HAL_OBJS=$(wildcard ../out/samr21HAL/*.o)



INCDIRS=\
	-I. \
	-I$(CMSIS_DEVICE_DIR)/include \
	-I$(CMSIS_DEVICE_DIR)/source \
	-I$(CMSIS_CORE_DIR) \
	-I$(TINYUSB_INC_DIR) \
	-I$(TINYUSB_CONF_DIR) \
	-I$(AT86RF233_INC_DIR) \
	-I$(HAL_INC_DIR)
	
DEFINES= -D__SAMR21E18A__ -D__TESTBUILD__ -D_DEBUG 
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

LINKERSCRIPT=../third_party/ATMEL/LINKER/samr21e18a_flash.ld
LINKERSCRIPT_GCF=../third_party/ATMEL/LINKER/samr21e18a_flash_gcf_offset.ld

LDFLAGS= \
 	-mcpu=$(MACH) \
	-mfloat-abi=soft \
	-specs=nano.specs \
	-specs=nosys.specs \
	-mthumb \
	-Wall \
	-Wl,--entry=Reset_Handler
	
testGCF:\
	samr21HAL \
	third_party \
	$(OBJ_OUT_DIR) \
	$(IMAGE_OUT_DIR) \
	$(OBJ_OUT_DIR)/testGCF.o \
	$(IMAGE_OUT_DIR)/testGCF.elf

testTX:\
	samr21HAL \
	third_party \
	$(OBJ_OUT_DIR) \
	$(IMAGE_OUT_DIR) \
	$(OBJ_OUT_DIR)/testTX.o \
	$(IMAGE_OUT_DIR)/testTX.elf

testRX:\
	samr21HAL \
	third_party \
	$(OBJ_OUT_DIR) \
	$(IMAGE_OUT_DIR) \
	$(OBJ_OUT_DIR)/testRX.o \
	$(IMAGE_OUT_DIR)/testRX.elf

testED:\
	samr21HAL \
	third_party \
	$(OBJ_OUT_DIR) \
	$(IMAGE_OUT_DIR) \
	$(OBJ_OUT_DIR)/testED.o \
	$(IMAGE_OUT_DIR)/testED.elf

testMisc:\
	samr21HAL \
	third_party \
	$(OBJ_OUT_DIR) \
	$(IMAGE_OUT_DIR) \
	$(OBJ_OUT_DIR)/testMisc.o \
	$(IMAGE_OUT_DIR)/testMisc.elf



$(OBJ_OUT_DIR):
	$(MKDIR) -p $@

$(IMAGE_OUT_DIR):
	$(MKDIR) -p $@
	
$(OBJ_OUT_DIR)/testRX.o:\
	./mainRX.c 	
	$(CC) $(CFLAGS) -o $@ $^

$(OBJ_OUT_DIR)/testTX.o:\
	./mainTX.c 	
	$(CC) $(CFLAGS) -o $@ $^

$(OBJ_OUT_DIR)/testED.o:\
	./mainED.c 	
	$(CC) $(CFLAGS) -o $@ $^

$(OBJ_OUT_DIR)/testMisc.o:\
	./mainMisc.c 	
	$(CC) $(CFLAGS) -o $@ $^

$(OBJ_OUT_DIR)/testGCF.o:\
	./mainGCF.c 	
	$(CC) $(CFLAGS) -o $@ $^


$(IMAGE_OUT_DIR)/testRX.elf:\
	$(OBJ_OUT_DIR)/testRX.o 

	$(CC) $(LDFLAGS) -T$(LINKERSCRIPT) -o $@ $^ $(TINYUSB_OBJS) $(CMSIS_OBJS)	$(HAL_OBJS) 

$(IMAGE_OUT_DIR)/testTX.elf:\
	$(OBJ_OUT_DIR)/testTX.o 

	$(CC) $(LDFLAGS) -T$(LINKERSCRIPT) -o $@ $^ $(TINYUSB_OBJS) $(CMSIS_OBJS)	$(HAL_OBJS) 

$(IMAGE_OUT_DIR)/testED.elf:\
	$(OBJ_OUT_DIR)/testED.o 

	$(CC) $(LDFLAGS) -T$(LINKERSCRIPT) -o $@ $^ $(TINYUSB_OBJS) $(CMSIS_OBJS)	$(HAL_OBJS) 

$(IMAGE_OUT_DIR)/testMisc.elf:\
	$(OBJ_OUT_DIR)/testMisc.o 

	$(CC) $(LDFLAGS) -T$(LINKERSCRIPT) -o $@ $^ $(TINYUSB_OBJS) $(CMSIS_OBJS)	$(HAL_OBJS) 

$(IMAGE_OUT_DIR)/testGCF.elf:\
	$(OBJ_OUT_DIR)/testGCF.o 

	$(CC) $(LDFLAGS) -T$(LINKERSCRIPT_GCF) -o $@ $^ $(TINYUSB_OBJS) $(CMSIS_OBJS)	$(HAL_OBJS) 


flashTX:testTX
	$(OCD) -f ./OpenOCD_config/ATMEL-ICE-OpenOCD-samr21e18a.cfg & echo $$! > ./OpenOCD_config/tempOcdInstance.pid
	$(GDB) --init-eval-command='target extended-remote localhost:3333' $(IMAGE_OUT_DIR)/testTX.elf --eval-command='load'
	if [ -a ./OpenOCD_config/tempOcdInstance.pid ]; then \
    	kill -TERM $$(cat ./OpenOCD_config/tempOcdInstance.pid) || true; \
	fi;
	rm ./OpenOCD_config/tempOcdInstance.pid

flashRX:testRX
	$(OCD) -f ./OpenOCD_config/ATMEL-ICE-OpenOCD-samr21e18a.cfg & echo $$! > ./OpenOCD_config/tempOcdInstance.pid
	$(GDB) --init-eval-command='target extended-remote localhost:3333' $(IMAGE_OUT_DIR)/testRX.elf --eval-command='load'
	if [ -a ./OpenOCD_config/tempOcdInstance.pid ]; then \
    	kill -TERM $$(cat ./OpenOCD_config/tempOcdInstance.pid) || true; \
	fi;
	rm ./OpenOCD_config/tempOcdInstance.pid

flashED:testED
	$(OCD) -f ./OpenOCD_config/ATMEL-ICE-OpenOCD-samr21e18a.cfg & echo $$! > ./OpenOCD_config/tempOcdInstance.pid
	$(GDB) --init-eval-command='target extended-remote localhost:3333' $(IMAGE_OUT_DIR)/testED.elf --eval-command='load'
	if [ -a ./OpenOCD_config/tempOcdInstance.pid ]; then \
    	kill -TERM $$(cat ./OpenOCD_config/tempOcdInstance.pid) || true; \
	fi;
	rm ./OpenOCD_config/tempOcdInstance.pid

flashMisc:testMisc
	$(OCD) -f ./OpenOCD_config/ATMEL-ICE-OpenOCD-samr21e18a.cfg & echo $$! > ./OpenOCD_config/tempOcdInstance.pid
	$(GDB) --init-eval-command='target extended-remote localhost:3333' $(IMAGE_OUT_DIR)/testMisc.elf --eval-command='load'
	if [ -a ./OpenOCD_config/tempOcdInstance.pid ]; then \
    	kill -TERM $$(cat ./OpenOCD_config/tempOcdInstance.pid) || true; \
	fi;
	rm ./OpenOCD_config/tempOcdInstance.pid


third_party:
	cd ../third_party && make

samr21HAL:
	cd ../src && make DEFINES="$(DEFINES)"
	 	
$(OUT_DIR):
	$(MKDIR) -p $@


cleanAll:
	cd ../third_party && make clean
	cd ../src && make clean
	rm -rf $(IMAGE_OUT_DIR)
	rm -rf $(OBJ_OUT_DIR)

clean:
	rm -rf $(IMAGE_OUT_DIR)
	rm -rf $(OBJ_OUT_DIR)

killOCD:
	if [ -a ./OpenOCD_config/tempOcdInstance.pid ]; then \
    	kill -TERM $$(cat ./OpenOCD_config/tempOcdInstance.pid) || true; \
	fi;
	rm ./OpenOCD_config/tempOcdInstance.pid