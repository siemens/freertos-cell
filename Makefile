.PHONY: clean all

src = $(CURDIR)

CROSS_COMPILE ?= arm-linux-gnueabihf-

CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
AR = $(CROSS_COMPILE)ar
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS += -mcpu=cortex-a7 -mtune=cortex-a7 -mfpu=vfpv4-d16 -mfloat-abi=hard -O2
CFLAGS += -DCONFIG_MACH_SUN7I=1
CFLAGS += -Wall -MMD -pipe
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
	freertos-runtime/serial.o \
	freertos-runtime/printf-stdarg.o \
	freertos-runtime/lib1funcs.o

RUNTIME_OBJS = $(FREERTOS_RUNTIME_OBJS) $(FREERTOS_OBJS)
OBJS = main.o boot_stub.o

RUNTIME_AR = libfreertos.a

all: $(EXE_STEM).bin

DEPS := $(OBJS:.o=.d) $(RUNTIME_OBJS:.o=.d)

$(EXE_STEM).elf: $(OBJS) $(RUNTIME_AR)
	$(LD) $(LDFLAGS) -o $@ $^

$(RUNTIME_AR): $(RUNTIME_OBJS)
	$(AR) -srcv $@ $^

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -f $(OBJS) $(EXE_STEM).elf $(EXE_STEM).bin $(RUNTIME_OBJS) $(RUNTIME_AR)

distclean: clean
	rm -f $(DEPS)

-include $(DEPS)
