
CC=./gcc-arm-none-eabi/bin/arm-none-eabi-gcc
CXX=./gcc-arm-none-eabi/bin/arm-none-eabi-g++
ASM=./gcc-arm-none-eabi/bin/arm-none-eabi-as
GDB=./gcc-arm-none-eabi/bin/arm-none-eabi-gdb

MACH=cortex-m0plus

OPT=-O0

HAL_DIR=./HAL

OUT_DIR=./out

LINKERSCRIPT=./ASF/samr21e18a_flash.ld

CMSIS_DEVICE_DIR=./CMSIS/r21

TINYUSB_SRC_DIR=./tinyusb/src
TINYUSB_CONF_DIR=./tinyusb/config

INCDIRS= -I. -I./CMSIS/r21/include -I./CMSIS/r21/source -I./CMSIS/core -I./AT86RF233 -I$(HAL_DIR) -I$(TINYUSB_SRC_DIR) -I$(TINYUSB_CONF_DIR)
DEFINES= -D__SAMR21E18A__ -D__CORECLK__=1000000 -D__PERIPHERALCLK__ -D_DEBUG

CFLAGS= -c -g -mcpu=$(MACH) -mfloat-abi=soft -mthumb -std=gnu99 -Wall $(OPT) $(INCDIRS) $(DEFINES)
LDFLAGS= -mcpu=$(MACH) -mfloat-abi=soft -specs=nano.specs -specs=nosys.specs -mthumb -Wall -Wl,--entry=Reset_Handler -T$(LINKERSCRIPT)

allTX:\
	mainTX.o \
	tusb.o \
	usbd.o \
	usbd_control.o \
	tusb_fifo.o \
	cdc_device.o \
	dcd_samd.o \
	usb_descriptors.o \
	startup_samr21.o \
	system_samr21.o \
	samr21PowerManager.o \
	samr21Radio.o \
	samr21RadioFSM.o \
	samr21Timer.o \
	samr21NopDelay.o \
	samr21Rtc.o \
	samr21Trx.o \
	samr21Clock.o \
	finalTX.elf

allRX:\
	mainRX.o \
	tusb.o \
	usbd.o \
	usbd_control.o \
	tusb_fifo.o \
	cdc_device.o \
	dcd_samd.o \
	usb_descriptors.o \
	startup_samr21.o \
	system_samr21.o \
	samr21PowerManager.o \
	samr21Radio.o \
	samr21RadioFSM.o \
	samr21Timer.o \
	samr21NopDelay.o \
	samr21Rtc.o \
	samr21Trx.o \
	samr21Clock.o \
	finalRX.elf

##TinyUSB

tusb.o:$(TINYUSB_SRC_DIR)/tusb.c
	$(CC) $(CFLAGS) -o $@ $^

usbd.o:$(TINYUSB_SRC_DIR)/device/usbd.c
	$(CC) $(CFLAGS) -o $@ $^

usbd_control.o:$(TINYUSB_SRC_DIR)/device/usbd_control.c
	$(CC) $(CFLAGS) -o $@ $^

tusb_fifo.o:$(TINYUSB_SRC_DIR)/common/tusb_fifo.c
	$(CC) $(CFLAGS) -o $@ $^

cdc_device.o:$(TINYUSB_SRC_DIR)/class/cdc/cdc_device.c
	$(CC) $(CFLAGS) -o $@ $^

dcd_samd.o:$(TINYUSB_SRC_DIR)/portable/microchip/samd/dcd_samd.c
	$(CC) $(CFLAGS) -o $@ $^

usb_descriptors.o:$(TINYUSB_CONF_DIR)/usb_descriptors.c
	$(CC) $(CFLAGS) -o $@ $^




##APPLICATION
mainTX.o:mainTX.c
	$(CC) $(CFLAGS) -o $@ $^

mainRX.o:mainRX.c
	$(CC) $(CFLAGS) -o $@ $^

samr21Trx.o:$(HAL_DIR)/samr21Trx.c
	$(CC) $(CFLAGS) -o $@ $^

samr21Clock.o:$(HAL_DIR)/samr21Clock.c
	$(CC) $(CFLAGS) -o $@ $^

samr21PowerManager.o:$(HAL_DIR)/samr21PowerManager.c
	$(CC) $(CFLAGS) -o $@ $^

samr21Timer.o:$(HAL_DIR)/samr21Timer.c
	$(CC) $(CFLAGS) -o $@ $^

samr21NopDelay.o:$(HAL_DIR)/samr21NopDelay.c
	$(CC) $(CFLAGS) -o $@ $^

samr21Rtc.o:$(HAL_DIR)/samr21Rtc.c
	$(CC) $(CFLAGS) -o $@ $^

samr21Radio.o:$(HAL_DIR)/samr21Radio.c
	$(CC) $(CFLAGS) -o $@ $^

samr21RadioFSM.o:$(HAL_DIR)/samr21RadioFSM.c
	$(CC) $(CFLAGS) -o $@ $^

startup_samr21.o:$(CMSIS_DEVICE_DIR)/source/gcc/startup_samr21.c
	$(CC) $(CFLAGS) -o $@ $^

system_samr21.o:$(CMSIS_DEVICE_DIR)/source/system_samr21.c 
	$(CC) $(CFLAGS) -o $@ $^
	
finalRX.elf:\
	tusb.o \
	mainRX.o \
	usbd.o \
	usbd_control.o \
	tusb_fifo.o \
	cdc_device.o \
	dcd_samd.o \
	usb_descriptors.o \
	startup_samr21.o \
	system_samr21.o \
	samr21PowerManager.o \
	samr21Radio.o \
	samr21RadioFSM.o \
	samr21Timer.o \
	samr21Rtc.o \
	samr21NopDelay.o \
	samr21Trx.o \
	samr21Clock.o 

	$(CC) $(LDFLAGS) -o $@ $^

finalTX.elf:\
	tusb.o \
	mainTX.o \
	usbd.o \
	usbd_control.o \
	tusb_fifo.o \
	cdc_device.o \
	dcd_samd.o \
	usb_descriptors.o \
	startup_samr21.o \
	system_samr21.o \
	samr21PowerManager.o \
	samr21Radio.o \
	samr21RadioFSM.o \
	samr21Timer.o \
	samr21Rtc.o \
	samr21NopDelay.o \
	samr21Trx.o \
	samr21Clock.o 

	$(CC) $(LDFLAGS) -o $@ $^

flashRX: finalRX.elf
	$(GDB) --init-eval-command='target extended-remote localhost:3333' ./finalRX.elf --eval-command='load'

flashTX: finalTX.elf
	$(GDB) --init-eval-command='target extended-remote localhost:3333' ./finalTX.elf --eval-command='load'

clean:
	rm -rf *.o *.elf

