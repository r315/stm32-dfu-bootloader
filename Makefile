TARGET=bootloader-dfu-fw
CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
SZ = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy
GIT_VERSION := $(shell git describe --abbrev=8 --dirty --always --tags)

# Config bits
BOOTLOADER_SIZE = 4
FLASH_SIZE = 64
FLASH_BASE_ADDR = 0x08000000
FLASH_BOOTLDR_PAYLOAD_SIZE_KB = $(shell echo $$(($(FLASH_SIZE) - $(BOOTLOADER_SIZE))))

# Default config
HSE ?=HSE8MHZ

CONFIG ?= \
-DENABLE_SAFEWRITE \
-DENABLE_CHECKSUM \
-DENABLE_DFU_UPLOAD \
-D_ENABLE_WATCHDOG \
-D_ENABLE_PROTECTIONS \
-D_ENABLE_GPIO_DFU_BOOT \
-D_GPIO_DFU_BOOT_PORT=GPUIB \
-D_GPIO_DFU_BOOT_PIN=15 \
-D$(HSE)

CFLAGS =-Os -g -mcpu=cortex-m3 -mthumb -DSTM32F1 \
	-std=c11 -Wall -Werror -Wextra -Wpedantic \
	-ffunction-sections -fdata-sections -fno-builtin \
	-DVERSION=\"$(GIT_VERSION)\" $(CONFIG)

LDFLAGS =-mcpu=cortex-m3 -mthumb \
 	-ffunction-sections -fdata-sections \
	-Wl,-Map=$(TARGET).map,-gc-sections,-cref \
	-nostartfiles -nostdlib -lnosys \

all: $(TARGET).elf

# DFU bootloader firmware -Wl,-Ttext=$(FLASH_BASE_ADDR)
%.elf: init.o main.o usb.o
	@echo "[LD]" $@
	@$(CC) $^ -o $@ -Tstm32f103.ld $(LDFLAGS)
	$(SZ) -A $@
	$(SZ) -B $@

%.bin: %.elf
	@echo "[OBJCOPY]" $@
	$(OBJCOPY) -O binary $^ $@

%.o: %.c | flash_config.h
	@echo "[CC]" $<
	@$(CC) -c $< -o $@ $(CFLAGS)

flash_config.h:
	@echo "#define FLASH_BASE_ADDR $(FLASH_BASE_ADDR)" > flash_config.h
	@echo "#define FLASH_SIZE_KB $(FLASH_SIZE)" >> flash_config.h
	@echo "#define FLASH_BOOTLDR_PAYLOAD_SIZE_KB $(FLASH_BOOTLDR_PAYLOAD_SIZE_KB)" >> flash_config.h
	@echo "#define FLASH_BOOTLDR_SIZE_KB $(BOOTLOADER_SIZE)" >> flash_config.h

clean:
	-rm -f *.elf *.o *.bin *.map flash_config.h
