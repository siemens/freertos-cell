.PHONY: clean all

src = $(shell pwd)

CROSS_COMPILE ?= arm-linux-gnueabihf-

CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld

CFLAGS += -mcpu=cortex-a7 -mtune=cortex-a7 -mfpu=vfpv4-d16 -mfloat-abi=hard -O2
CFLAGS += -DCONFIG_MACH_SUN7I=1
CFLAGS += -Wall -MD
CFLAGS += -I $(src) -I $(src)/freertos/Source/include -I $(src)/freertos-runtime -I $(src)/freertos/Source/portable/GCC/ARM_A7jailhouse

LDFLAGS += -T lscript.lds

EXE_STEM = freertos-demo

FREERTOS_OBJS = freertos/Source/queue.o \
	freertos/Source/list.o \
	freertos/Source/croutine.o \
	freertos/Source/event_groups.o \
	freertos/Source/portable/MemMang/heap_1.o \
	freertos/Source/portable/GCC/ARM_A7jailhouse/port.o \
	freertos/Source/portable/GCC/ARM_A7jailhouse/gic-v2.o \
	freertos/Source/portable/GCC/ARM_A7jailhouse/portASM.o \
	freertos/Source/timers.o \
	freertos/Source/tasks.o

FREERTOS_RUNTIME_OBJS = freertos-runtime/string.o \
	freertos-runtime/serial_printf.o

OBJS = $(FREERTOS_RUNTIME_OBJS) $(FREERTOS_OBJS) freertos-demo.o boot_stub.o

all: $(EXE_STEM).bin

DEPS := $(OBJS:.o=.d)

$(EXE_STEM).elf: $(OBJS) $(LIBS)
	$(LD) $(LDFLAGS) -o $@ $^

%.bin: %.elf
	$(CROSS_COMPILE)objcopy -O binary $< $@

clean:
	rm -f $(OBJS) $(EXE_STEM).elf $(EXE_STEM).bin

distclean: clean
	rm -f $(OBJS) $(EXE_STEM).elf $(EXE_STEM).bin $(DEPS)

-include $(DEPS)
