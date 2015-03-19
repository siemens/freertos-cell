.PHONY: clean all

src = $(CURDIR)

CROSS_COMPILE ?= arm-linux-gnueabihf-

CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
AR = $(CROSS_COMPILE)ar
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS += -mcpu=cortex-a15 -mtune=cortex-a15 -mfpu=vfpv4-d16 -mfloat-abi=hard -O2
CFLAGS += -DCONFIG_MACH_SUN7I=1
CFLAGS += -Wall -MMD -pipe
CFLAGS += -I $(src) -I $(src)/freertos/Source/include -I $(src)/freertos-runtime -I $(src)/freertos/Source/portable/GCC/ARM_A7jailhouse

LDFLAGS += -T lscript.lds

EXE_STEM = freertos-demo

FREERTOS_OBJS = freertos/Source/queue.o \
	freertos/Source/list.o \
	freertos/Source/croutine.o \
	freertos/Source/event_groups.o \
	freertos/Source/portable/MemMang/heap_4.o \
	freertos/Source/portable/GCC/ARM_A7jailhouse/port.o \
	freertos/Source/portable/GCC/ARM_A7jailhouse/gic-v2.o \
	freertos/Source/portable/GCC/ARM_A7jailhouse/portASM.o \
	freertos/Source/timers.o \
	freertos/Source/tasks.o

FREERTOS_RUNTIME_OBJS = freertos-runtime/string.o \
	freertos-runtime/serial.o \
	freertos-runtime/printf-stdarg.o \
	freertos-runtime/lib1funcs.o

#
# LWIP tcpip stack
#
LWIP_DIR = $(src)/lwip/src
# Core code
LWIP_C_SRCS += \
	$(LWIP_DIR)/core/timers.c \
	$(LWIP_DIR)/core/def.c \
	$(LWIP_DIR)/core/dhcp.c \
	$(LWIP_DIR)/core/dns.c \
	$(LWIP_DIR)/core/init.c \
	$(LWIP_DIR)/core/mem.c \
	$(LWIP_DIR)/core/memp.c \
	$(LWIP_DIR)/core/netif.c \
	$(LWIP_DIR)/core/pbuf.c \
	$(LWIP_DIR)/core/raw.c \
	$(LWIP_DIR)/core/stats.c \
	$(LWIP_DIR)/core/sys.c \
	$(LWIP_DIR)/core/tcp.c \
	$(LWIP_DIR)/core/tcp_in.c \
	$(LWIP_DIR)/core/tcp_out.c \
	$(LWIP_DIR)/core/udp.c
# IPv4 code
CFLAGS += -I$(LWIP_DIR)/include/ipv4
LWIP_C_SRCS += \
	$(LWIP_DIR)/core/ipv4/autoip.c \
	$(LWIP_DIR)/core/ipv4/icmp.c \
	$(LWIP_DIR)/core/ipv4/igmp.c \
	$(LWIP_DIR)/core/ipv4/inet.c \
	$(LWIP_DIR)/core/ipv4/inet_chksum.c \
	$(LWIP_DIR)/core/ipv4/ip_addr.c \
	$(LWIP_DIR)/core/ipv4/ip.c \
	$(LWIP_DIR)/core/ipv4/ip_frag.c \
	$(LWIP_DIR)/netif/etharp.c
# API code
LWIP_C_SRCS += \
	$(LWIP_DIR)/api/api_lib.c \
	$(LWIP_DIR)/api/api_msg.c \
	$(LWIP_DIR)/api/err.c \
	$(LWIP_DIR)/api/netbuf.c \
	$(LWIP_DIR)/api/netdb.c \
	$(LWIP_DIR)/api/netifapi.c \
	$(LWIP_DIR)/api/sockets.c \
	$(LWIP_DIR)/api/tcpip.c
# PPP
LWIP_C_SRCS += \
	lwip/src/netif/ppp/auth.c \
	lwip/src/netif/ppp/chap.c \
	lwip/src/netif/ppp/chpms.c \
	lwip/src/netif/ppp/fsm.c \
	lwip/src/netif/ppp/ipcp.c \
	lwip/src/netif/ppp/lcp.c \
	lwip/src/netif/ppp/magic.c \
	lwip/src/netif/ppp/md5.c \
	lwip/src/netif/ppp/pap.c \
	lwip/src/netif/ppp/ppp.c \
	lwip/src/netif/ppp/ppp_oe.c \
	lwip/src/netif/ppp/randm.c \
	lwip/src/netif/ppp/vj.c
# PPP support
LWIP_C_SRCS += \
	rand.c \
	sio_ppp.c

CFLAGS += -I$(LWIP_DIR)/include -I$(LWIP_DIR)/netif
# Platform code 
LWIP_PLATFORM_DIR = $(src)/lwip-freertos
CFLAGS += -I$(LWIP_PLATFORM_DIR)/include -I$(LWIP_PLATFORM_DIR)/include/lwip
LWIP_C_SRCS += \
	$(LWIP_PLATFORM_DIR)/api/sys_arch.c
		
LWIP_OBJS = $(LWIP_C_SRCS:.c=.o)

ALL_FREERTOS_OBJS = $(FREERTOS_RUNTIME_OBJS) $(FREERTOS_OBJS)
OBJS = main.o boot_stub.o

FREERTOS_AR = libfreertos.a
LWIP_AR = liblwip.a

all: $(EXE_STEM).bin

DEPS := $(OBJS:.o=.d) $(ALL_FREERTOS_OBJS:.o=.d)

$(EXE_STEM).elf: $(OBJS) $(LWIP_AR) $(FREERTOS_AR)
	$(LD) $(LDFLAGS) -o $@ $^

$(LWIP_AR): $(LWIP_OBJS)
	$(AR) -srcv $@ $^

$(FREERTOS_AR): $(ALL_FREERTOS_OBJS)
	$(AR) -srcv $@ $^

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -f $(OBJS) $(EXE_STEM).elf $(EXE_STEM).bin $(ALL_FREERTOS_OBJS) $(FREERTOS_AR) $(LWIP_OBJS) $(LWIP_AR)

distclean: clean
	rm -f $(DEPS)

-include $(DEPS)
