#include <jailhouse/types.h>
#include <jailhouse/cell-config.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MBYTE(m) ((m)<<20)

#define GUEST_MEM_START 0x7a800000 /* XXX */
#define GUEST_MEM_SIZE MBYTE(16)

struct {
	struct jailhouse_cell_desc cell;
	__u64 cpus[1];
	struct jailhouse_memory mem_regions[4];
	struct jailhouse_irqchip irqchips[1];
} __attribute__((packed)) config = {
	.cell = {
		.signature = JAILHOUSE_CELL_DESC_SIGNATURE,
		.revision = JAILHOUSE_CONFIG_REVISION,
		.name = "FreeRTOS",
		.flags = JAILHOUSE_CELL_PASSIVE_COMMREG,

		.cpu_set_size = sizeof(config.cpus),
		.num_memory_regions = ARRAY_SIZE(config.mem_regions),
		.num_irqchips = ARRAY_SIZE(config.irqchips),
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
			.size = 0x400,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE | 
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32 | JAILHOUSE_MEM_ROOTSHARED,
		},
#define PIO_P7_DAT_REG (0x01c20800 + 7*0x24 + 0x10)
		/* PIO port 7: blinking LED */ {
			.phys_start = PIO_P7_DAT_REG,
			.virt_start = PIO_P7_DAT_REG,
			.size = 0x4,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32,
		},
	},
	.irqchips = {
		/* GIC */ {
			.address = 0x01c81000, /* XXX perhaps 0x01c81000 */
			.pin_base = 32,
			/* Interrupt of UART 7 belongs to the client */
			.pin_bitmap = {
				1<<(52-32), 0, 0, 0,
			},
		},
	},
};
