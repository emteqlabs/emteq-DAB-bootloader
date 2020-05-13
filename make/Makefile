##############################################################################
BUILD = build
BIN = D11boot

##############################################################################
.PHONY: all directory clean size

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

CFLAGS += -W -Wall --std=gnu99 -Os
CFLAGS += -Wno-address-of-packed-member
CFLAGS += -fno-diagnostics-show-caret
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -mcpu=cortex-m0plus -mthumb
CFLAGS += -MD -MP -MT $(BUILD)/$(*F).o -MF $(BUILD)/$(@F).d

LDFLAGS += -mcpu=cortex-m0plus -mthumb
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--script=../linker/samd11d14.ld

# Command-line examples:
#		make
#
# [Default] 1) make SAMD11D14AM
#			2) make SAMD51J19A

INCLUDES += \
  -I../include \
  -I..

SRCS += \
  ../bootloader.c \
  ../usb_descriptors.c

DEFINES += \
  -DDONT_USE_CMSIS_INIT \
  -DSTARTUP_FROM_RESET

CFLAGS += $(INCLUDES) $(DEFINES)
ASFLAGS += $(CFLAGS)

OBJS = $(addprefix $(BUILD)/, $(notdir %/$(subst .c,.o, $(SRCS))))
OBJS += $(BUILD)/startup.o

.PHONY: SAMD51 SAMD11 SAMD51J19A SAMD11D14AM

# Targets
# @note Default target is the first target i.e. SAMD11D14AM

# SAMD11
SAMD11D14AM: DEFINES += -D__SAMD11D14AM__
SAMD11C14A : DEFINES += -D__SAMD11C14A__
SAMD11D14AS: DEFINES += -D__SAMD11D14AS__
SAMD11D14AU: DEFINES += -D__SAMD11D14AU__
SAMD11C14A SAMD11D14AM SAMD11D14AS SAMD11D14AU: SAMD11
SAMD11: DEFINES += -D__SAMD11__
SAMD11: INCLUDES += -I../include/SAMD11

# SAMD51
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
SAMD51: DEFINES += -D__SAMD51__
SAMD51: INCLUDES += -I../include/SAMD51

info:
	@echo DEFINES:
	@echo $(DEFINES)
	@echo INCLUDES:
	@echo $(INCLUDES)

SAMD51 SAMD11: info all	

all: $(BUILD) $(BUILD)/$(BIN).hex $(BUILD)/$(BIN).bin size

$(BUILD)/$(BIN).elf: $(OBJS) $(BUILD)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

$(BUILD)/$(BIN).hex: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex $^ $@

$(BUILD)/$(BIN).bin: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary $^ $@

%.o:
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %/$(subst .o,.c,$(notdir $@)), $(SRCS)) -c -o $@

$(BUILD)/startup.o: ../startup.s $(BUILD)
	@echo CC $@
	@$(CC) -x assembler-with-cpp $(ASFLAGS) -c -o $@ $<

$(BUILD):
	mkdir $@

size: $(BUILD)/$(BIN).elf
	@echo size:
	@$(SIZE) -t $^

clean:
	@echo clean
	@-rmdir /S /Q $(BUILD)

-include $(wildcard $(BUILD)/*.d)

