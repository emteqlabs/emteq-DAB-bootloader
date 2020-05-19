ifeq ( $(OS), Windows_NT )	
	EXE_SUFFIX := .exe
endif

DX1ELF2DFU_C = dx1elf2dfu.c

CC?=gcc

CFLAGS+= -g -O1

CFLAGS = /Zi
LDFLAGS = /debug:full

all: dx1elf2dfu$(EXE_SUFFIX)

dx1elf2dfu$(EXE_SUFFIX): Makefile $(DX1ELF2DFU_C)
	$(CC) $(DX1ELF2DFU_C) -o $@ $(CFLAGS)
	
#strip $@

clean:
	rm -f dx1elf2dfu$(EXE_SUFFIX)
