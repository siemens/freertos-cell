#include <jailhouse/types.h>
#include <jailhouse/cell-config.h>

#define __ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MBYTE(m) ((m)<<20)

#define GUEST_MEM_START 0x7a800000
#define GUEST_MEM_SIZE MBYTE(16)

struct {
	struct jailhouse_cell_desc cell;
	__u64 cpus[1];
	struct jailhouse_memory mem_regions[8];
	struct jailhouse_irqchip irqchips[1];
	struct jailhouse_pci_device pci_devices[1];
} __attribute__((packed)) config = {
	.cell = {
		.signature = JAILHOUSE_CELL_DESC_SIGNATURE,
		.revision = JAILHOUSE_CONFIG_REVISION,
		.architecture = JAILHOUSE_ARM,
		.name = "FreeRTOS",
		.flags = JAILHOUSE_CELL_PASSIVE_COMMREG,

		.cpu_set_size = sizeof(config.cpus),
		.num_memory_regions = __ARRAY_SIZE(config.mem_regions),
		.num_irqchips = __ARRAY_SIZE(config.irqchips),
		.num_pci_devices = __ARRAY_SIZE(config.pci_devices),

		.vpci_irq_base = 123,
	},

	.cpus = {
		0x2,
	},

	.mem_regions = {
		/* IVSHMEM shared memory regions (demo) */
		{
			.phys_start = 0x7bf00000,
			.virt_start = 0x7bf00000,
			.size = 0x1000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_ROOTSHARED,
		},
		{
			.phys_start = 0x7bf00000 + 0x1000,
			.virt_start = 0x7bf00000 + 0x1000,
			.size = 0x1000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE | JAILHOUSE_MEM_ROOTSHARED,
		},
		{ 0 },
		{ 0 },
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
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32,
		},
		/* CCU (Hack) */ {
			.phys_start = 0x01c2006c,
			.virt_start = 0x01c2006c,
			.size = 0x4,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE | 
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32 | JAILHOUSE_MEM_ROOTSHARED,
		},
#define PIO_P7_REG (0x01c20800 + 7*0x24 + 0x0c)
		/* PIO port 7: blinking LED */ {
			.phys_start = PIO_P7_REG,
			.virt_start = PIO_P7_REG,
			.size = 0x8,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32,
		},
	},
	.irqchips = {
		/* GIC */ {
			.address = 0x01c81000,
			.pin_base = 32,
			/* Interrupt of UART 7 belongs to the client */
			.pin_bitmap = {
				1<<(52-32), 0, 0, 1 << (123+32-128),
			},
		},
	},
	.pci_devices = {
		{
			.type = JAILHOUSE_PCI_TYPE_IVSHMEM,
			.bdf = 0 << 3,
			.bar_mask = JAILHOUSE_IVSHMEM_BAR_MASK_INTX,
			.shmem_regions_start = 0,
			.shmem_dev_id = 1,
			.shmem_peers = 2,
			.shmem_protocol = JAILHOUSE_SHMEM_PROTO_UNDEFINED,
		},
	},
};
