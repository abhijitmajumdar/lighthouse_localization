# STM32F103C8T6 framework
# To compile code for STM32F103 (used on 'bluepill', a commonly available board)
# and upload it over a USB to serial adapter (DFU feature in STM32), while using
# the STM32 standard peripheral library

# Usage:
# - Put all source and header files into "src" and "include" directories respectively
# - When using the "Bluepill STM32 board", connect A9(TX) and A10(RX) from the board
#		to RX and TX on a "USB to Serial converter" respectively
# - Connect power(+5/+3.3 and Ground) to the STM board to power it up
# - Run 'make' to compile
# - Set BOOT0 pinto "1" on the STM board and press the reset button
# - Run 'make upload' to upload to STM board over TTY
# - When using serial, monitor it using 'screen <device> <baud>'
#		ex: 'screen /dev/ttyS1 115200'
# - Exit from serial monitor using "CTRL+a followed by "k" and "y"

# Note:
# - The default serial port is selected to be "/dev/ttyS1", you may need to change
#		this in case your "USB to Serial converter" is connected to a different port. For
#		example if its connected to "/dev/ttyUSB0" use as: 'USB_DEVICE=/dev/ttyUSB0 make upload'
# - Make sure there exists a common ground between STM board and the USB serial converter
# - When defining Interrupt Service Routines (ISR) make sure to enclose the method
# 	inside 'extern "C" {}' when using CPP

# Prerequisites:
# - Install compiler
# sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi
# - Install firmware loader
# sudo apt-get install stm32flash
# - STM32 Standard Peripheral Library
# search for "STSW-STM32054" and download into this folder
# - Install serial monitor
# sudo apt-get install screen

# More configurations:
# - define project name as 'PROJECT=my_project make', defaults to "project"
# - define build directory as 'BUILD_DIR=bd make', defaults to "build"
# - define build directory as 'SOURCE_DIR=my_sources make', defaults to "src"
# - define build directory as 'INCLUDE_DIR=my_includes make', defaults to "include"
# - define library to use as 'STMLIB=<STM32STD/STM32CUBE> make', defaults to "STM32STD"
# - switch to using gcc instead of g++ with 'COMPILER=CC make', defaults to CPP
# 	read this to make a decision to do so http://warp.povusers.org/grrr/cplusplus_vs_c.html

PROJECT ?= project
CUBE_LIBS ?= STM32Cube_FW_F1_V1.6.0/
STD_PERIPH_LIBS ?= STM32F10x_StdPeriph_Lib_V3.5.0/
STMLIB ?= STM32STD
USB_DEVICE ?= /dev/ttyS1
BUILD_DIR ?= build
SOURCE_DIR ?= src
INCLUDE_DIR ?= include
COMPILER ?= CPP
SERIAL_BAUD ?= 115200

ifneq ("$(STMLIB)","STM32STD")
ifneq ("$(STMLIB)","STM32CUBE")
$(error Invalid STMLIB definition, use STMLIB=<STM32STD/STM32CUBE>)
else
$(info --Using STM32 Cube library)
endif
else
$(info --Using STM32 Standard Peripheral library)
endif

# list of source files
SOURCES_C = $(wildcard $(SOURCE_DIR)/*.c)
SOURCES_CPP = $(wildcard $(SOURCE_DIR)/*.cpp)
# Listing all library, system, startup and linker files from STM Library
ifeq ("$(STMLIB)","STM32STD")
	SYSTEM_FILE = $(STD_PERIPH_LIBS)/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c
	STARTUP_FILE = $(STD_PERIPH_LIBS)/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/TrueSTUDIO/startup_stm32f10x_md.s
	LINKER_FILE = $(STD_PERIPH_LIBS)/Project/STM32F10x_StdPeriph_Template/TrueSTUDIO/STM3210B-EVAL/stm32_flash.ld
	LIB_FILES = $(wildcard $(STD_PERIPH_LIBS)/Libraries/STM32F10x_StdPeriph_Driver/src/*.c)
endif

ifeq ("$(STMLIB)","STM32CUBE")
	SYSTEM_FILE = $(CUBE_LIBS)/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c
	STARTUP_FILE = $(CUBE_LIBS)/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s
	LINKER_FILE = $(CUBE_LIBS)/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/linker/STM32F103XB_FLASH.ld
	LIB_FILES = $(wildcard $(CUBE_LIBS)/Drivers/STM32F1xx_HAL_Driver/Src/*.c)
endif

# Objects
OBJECTS_C = $(SOURCES_C:%.c=$(BUILD_DIR)/%.o)
OBJECTS_CPP = $(SOURCES_CPP:%.cpp=$(BUILD_DIR)/%.o)
SYSTEM := $(SYSTEM_FILE:%.c=$(BUILD_DIR)/%.o)
STARTUP := $(STARTUP_FILE:%.s=$(BUILD_DIR)/%.o)
LIBS := $(LIB_FILES:%.c=$(BUILD_DIR)/%.o)
LINKER := $(LINKER_FILE:%=$(BUILD_DIR)/%)

# ARM Compilers/Linkers etc. (assumed to be in path)
# https://answers.launchpad.net/gcc-arm-embedded/+question/253559 -> use compiler
# as linker
CC=arm-none-eabi-gcc
CPP=arm-none-eabi-g++
CP=arm-none-eabi-objcopy
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
OD=arm-none-eabi-objdump
SE=arm-none-eabi-size
SF=stm32flash
ifeq ("$(COMPILER)","CC")
	COMPILER=$(CC)
else
	COMPILER=$(CPP)
endif
LD=$(COMPILER)

# Compiler flags
CFLAGS  = -g -O2
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m3
CFLAGS += -mfloat-abi=soft
CFLAGS += -Wl,--gc-sections
CFLAGS += --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lm -lrdimon -Wl,--end-group
# CFLAGS += --specs=nosys.specs
# CFLAGS += -specs=nano.specs
CFLAGS += -DSTM32F103xB -DSTM32F10X_MD
CFLAGS += -I$(INCLUDE_DIR)
ifeq ("$(STMLIB)","STM32STD")
	CFLAGS += -I$(STD_PERIPH_LIBS)/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/
	CFLAGS += -I$(STD_PERIPH_LIBS)/Libraries/CMSIS/CM3/CoreSupport
	CFLAGS += -I$(STD_PERIPH_LIBS)/Libraries/STM32F10x_StdPeriph_Driver/inc
	CFLAGS += -DUSE_STDPERIPH_DRIVER
endif
ifeq ("$(STMLIB)","STM32CUBE")
	CFLAGS += -I$(CUBE_LIBS)/Drivers/CMSIS/Include
	CFLAGS += -I$(CUBE_LIBS)/Drivers/CMSIS/Device/ST/STM32F1xx/Include
	CFLAGS += -DSTM32CUBE
endif

.PHONY: all detail clean upload run
all: $(PROJECT).elf

$(PROJECT).elf: $(OBJECTS_C) $(OBJECTS_CPP) $(LIBS) $(SYSTEM) $(STARTUP) $(LINKER)
	@echo "--Linking"
	@$(LD) $(CFLAGS) -T$(LINKER) $(OBJECTS_C) $(OBJECTS_CPP) $(SYSTEM) $(STARTUP) $(LIBS) -o $(BUILD_DIR)/$@
	@echo "--Build complete"

$(PROJECT).bin: $(PROJECT).elf
	@echo "--Generating BIN file"
	@$(CP) -O binary $(BUILD_DIR)/$< $(BUILD_DIR)/$@

$(PROJECT).hex: $(PROJECT).elf
	@echo "--Generating HEX file"
	@$(CP) -O ihex $(BUILD_DIR)/$< $(BUILD_DIR)/$@

$(OBJECTS_C): $(BUILD_DIR)/%.o:%.c
	@echo "--Compiling $(notdir $<)"
	@mkdir -p $(@D)
	@$(COMPILER) $(CFLAGS) -c -o $@ $<

$(OBJECTS_CPP): $(BUILD_DIR)/%.o:%.cpp
	@echo "--Compiling $(notdir $<)"
	@mkdir -p $(@D)
	@$(COMPILER) $(CFLAGS) -c -o $@ $<

$(LIBS): $(BUILD_DIR)/%.o:%.c
	@echo "--Compiling $(notdir $<)"
	@mkdir -p $(@D)
	@$(COMPILER) $(CFLAGS) -c -o $@ $<

$(SYSTEM): $(BUILD_DIR)/%.o:%.c
	@echo "--Compiling system script"
	@mkdir -p $(@D)
	@$(COMPILER) $(CFLAGS) -c -o $@ $<

$(STARTUP): $(BUILD_DIR)/%.o:%.s
	@echo "--Compiling startup file"
	@mkdir -p $(@D)
	@$(COMPILER) $(CFLAGS) -c -o $@ $<

# The linker script had a bunch of 0's, this fixes it
$(LINKER): $(BUILD_DIR)/%:%
	@echo "--Fixing linker script"
	@mkdir -p $(@D)
	@sed 's/^0//' $< > $@

detail: $(PROJECT).elf
	@${SE} $(BUILD_DIR)/$(PROJECT).elf

clean:
	@echo "--Cleaning up"
	@rm -rf build

upload: $(PROJECT).bin
	@echo "--Uploading to device"
	@${SF} -w $(BUILD_DIR)/$(PROJECT).bin -v -g 0x00 ${USB_DEVICE}

run:
	@echo "--Sending run command"
	@${SF} -g 0x00 ${USB_DEVICE}

serial:
	screen $(USB_DEVICE) $(SERIAL_BAUD)
