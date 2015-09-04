#include <linux/types.h>
#include <jailhouse/cell-config.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MBYTE(m) ((m)<<20)

#define GUEST_MEM_START 0x7a800000
#define GUEST_MEM_SIZE MBYTE(16)

struct {
	struct jailhouse_cell_desc cell;
	__u64 cpus[1];
	struct jailhouse_memory mem_regions[3];
  struct jailhouse_irqchip irqchips[1];
} __attribute__((packed)) config = {
	.cell = {
		.name = "FreeRTOS",
		.flags = JAILHOUSE_CELL_PASSIVE_COMMREG,

		.cpu_set_size = sizeof(config.cpus),
		.num_memory_regions = ARRAY_SIZE(config.mem_regions),
		.num_irqchips = 1,
		.pio_bitmap_size = 0,
		.num_pci_devices = 0,
	},

	.cpus = {
		0x2,
	},

	.mem_regions = {
		/* RAM */ {
      .phys_start = GUEST_MEM_START,
			.virt_start = 0,
			.size = GUEST_MEM_SIZE,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_EXECUTE | JAILHOUSE_MEM_LOADABLE,
		},
		/* UART 4-7 */ {
			.phys_start = 0x01c29000,
			.virt_start = 0x01c29000,
			.size = 0x1000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE | JAILHOUSE_MEM_IO,
		},
		/* CCU, Ints, GPIO, Timer */ {
			.phys_start = 0x01c20000,
			.virt_start = 0x01c20000,
			.size = 0x1000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE | JAILHOUSE_MEM_IO,
		},
	},
  .irqchips = {
    /* GIC */ {
      .address = 0x01c80000,
      /* Interrupt of UART 7 belongs to the client */
      .pin_bitmap = 1ULL<<(52-32),
    },
  },
};
