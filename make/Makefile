##############################################################################
# Target device-Id must be first make parameter
# Command-line examples:
#		make
#
# [Default] 1) make SAMD11D14AM
#			2) make SAMD51J19A
#
# Options:
#       dfu  --Generate DFU image for writing bootloader to user-program address
#		DEBUG=1 --Generate bootloader with debug symbols enabled

ifndef TARGET
MAKECMDGOALS?=SAMD11D14AM
TARGET?=$(word 1, $(MAKECMDGOALS))
endif

RELEASE_SUFFIX:=#
DEBUG_SUFFIX:=_d

## BIN may be defined from make commandline i.e. make BIN=MyDfuBootloader
ifndef BIN
ifdef DEBUG
	BIN = DfuBootloader$(TARGET)$(DEBUG_SUFFIX)
else
	BIN = DfuBootloader$(TARGET)$(RELEASE_SUFFIX)
endif
endif

ifdef DEBUG
CONFIG=debug
else
CONFIG=release
endif

BIN_DIR:=bin
BUILD_ROOT:=build
BUILD_DIR:=$(BUILD_ROOT)\$(CONFIG)\$(TARGET)

##############################################################################
.PHONY: all clean size dfu

CC= arm-none-eabi-gcc
STRIP= arm-none-eabi-strip
OBJCOPY= arm-none-eabi-objcopy
SIZE= arm-none-eabi-size
ELF2DFU=../dx1elf2dfu/dx1elf2dfu

CFLAGS += -W -Wall --std=gnu11 -Os
CFLAGS += -Wno-address-of-packed-member
CFLAGS += -fno-diagnostics-show-caret
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -mthumb
CFLAGS += -MD -MP -MT $(BUILD_DIR)/$(*F).o -MF $(BUILD_DIR)/$(@F).d

LDFLAGS += -mthumb
LDFLAGS += -Wl,--gc-sections -Wl,--print-memory-usage
LDFLAGS += -Wl,--script=../linker/samd11d14.ld

ifdef DEBUG
CFLAGS += -g -Og
LDFLAGS += -g -Og
endif

INCLUDES += \
  -I../include \
  -I..

SOURCE_DIR=..
SOURCES += \
  ../bootloader.c \
  ../usb_descriptors.c 
ASOURCES += \
  ../startup.s

VERSION_FILE=$(SOURCE_DIR)/Version.generated.h
VERSION_TAG=$(shell git -C $(SOURCE_DIR) --no-pager describe --tags --always --dirty)

DEFINES += \
  -DDONT_USE_CMSIS_INIT \
  -DSTARTUP_FROM_RESET

CFLAGS += $(INCLUDES) $(DEFINES)
ASFLAGS += $(CFLAGS)

# The names of all object files that must be generated
OBJECTS  := $(SOURCES:$(SOURCE_DIR)/%.c=$(BUILD_DIR)/%.o)
AOBJECTS := $(ASOURCES:$(SOURCE_DIR)/%.s=$(BUILD_DIR)/%.o)

.PHONY: default
default: ${TARGET}

# SAMD11
.PHONY: SAMD11 SAMD11C14A SAMD11D14AM SAMD11D14AS SAMD11D14AU
SAMD11D14AM: DEFINES += -D__SAMD11D14AM__
SAMD11C14A : DEFINES += -D__SAMD11C14A__
SAMD11D14AS: DEFINES += -D__SAMD11D14AS__
SAMD11D14AU: DEFINES += -D__SAMD11D14AU__
SAMD11C14A SAMD11D14AM SAMD11D14AS SAMD11D14AU: SAMD11
SAMD11: DEFINES += -D__SAMD11__ -DF_CPU=48000000
SAMD11: CFLAGS += -mcpu=cortex-m0plus
SAMD11: LDFLAGS += -mcpu=cortex-m0plus
SAMD11: INCLUDES += -I../include/SAMD11

# SAMD51
.PHONY: SAMD51 SAMD51G18A SAMD51G19A SAMD51J18A SAMD51J19A SAMD51J19B SAMD51J20A SAMD51J20C SAMD51N19A SAMD51N20A SAMD51P19A SAMD51P20A
SAMD51G18A: DEFINES += -D__SAMD51G18A__
SAMD51G19A: DEFINES += -D__SAMD51G19A__
SAMD51J18A: DEFINES += -D__SAMD51J18A__
SAMD51J19A: DEFINES += -D__SAMD51J19A__
SAMD51J19B: DEFINES += -D__SAMD51J19B__
SAMD51J20A: DEFINES += -D__SAMD51J20A__
SAMD51J20C: DEFINES += -D__SAMD51J20C__
SAMD51N19A: DEFINES += -D__SAMD51N19A__
SAMD51N20A: DEFINES += -D__SAMD51N20A__
SAMD51P19A: DEFINES += -D__SAMD51P19A__
SAMD51P20A: DEFINES += -D__SAMD51P20A__
SAMD51G18A SAMD51G19A SAMD51J18A SAMD51J19A SAMD51J19B SAMD51J20A SAMD51J20C SAMD51N19A SAMD51N20A SAMD51P19A SAMD51P20A: SAMD51
## @note Clocked at 48MHz in bootloader mode
SAMD51: DEFINES += -D__SAMD51__ -DF_CPU=48000000 -DUSE_STRING_DESCRIPTORS
SAMD51: CFLAGS += -mcpu=cortex-m4
SAMD51: LDFLAGS += -mcpu=cortex-m4
SAMD51: INCLUDES += -I../include/SAMD51

info:
	@echo DEFINES:
	@echo $(DEFINES)
	@echo INCLUDES:
	@echo $(INCLUDES)
	@echo VERSION_TAG:
	@echo $(VERSION_TAG)

SAMD51 SAMD11: info all	

all: $(BUILD_DIR) $(VERSION_FILE) $(BIN_DIR)/$(BIN).hex $(BIN_DIR)/$(BIN).bin size

 $(BUILD_DIR)/%.o : $(SOURCE_DIR)/%.c
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %/$(subst .o,.c,$(notdir $@)), $(SOURCES)) -c -o $@

$(VERSION_FILE): $(SOURCE_DIR)/.git/index
	@echo "$(VERSION_TAG)" > $@

$(AOBJECTS) : $(BUILD_DIR)/%.o : $(SOURCE_DIR)/%.s
	@echo CC $@
	@$(CC) -x assembler-with-cpp $(ASFLAGS) -c -o $@ $<

$(BIN_DIR)/$(BIN).elf: $(OBJECTS) $(AOBJECTS) $(BIN_DIR)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJECTS) $(AOBJECTS) $(LIBS) -o $@

$(BIN_DIR)/$(BIN).hex: $(BIN_DIR)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex $^ $@

$(BIN_DIR)/$(BIN).bin: $(BIN_DIR)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary $^ $@

$(BUILD_DIR) $(BIN_DIR):
	mkdir $@

size: $(BIN_DIR)/$(BIN).elf
	@echo size:
	@$(SIZE) -t $^
	
dfu: $(BIN_DIR)/$(BIN).dfu

$(BIN_DIR)/$(BIN).dfu: $(BIN_DIR)/$(BIN).elf
	@echo size:
	@$(ELF2DFU) $^ $@

clean:
	@echo clean
	@-rmdir /S /Q $(BUILD_ROOT)


-include $(wildcard $(BUILD_DIR)/*.d)

