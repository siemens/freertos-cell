/*
 * Jailhouse, a Linux-based partitioning hypervisor
 *
 * Test configuration for Banana Pi board (A20 dual-core Cortex-A7, 1G RAM)
 *
 * Copyright (c) Siemens AG, 2014
 *
 * Authors:
 *  Jan Kiszka <jan.kiszka@siemens.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

#include <jailhouse/types.h>
#include <jailhouse/cell-config.h>

#define __ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

struct {
	struct jailhouse_system header;
	__u64 cpus[1];
	struct jailhouse_memory mem_regions[33];
	struct jailhouse_irqchip irqchips[1];
	struct jailhouse_pci_device pci_devices[1];
} __attribute__((packed)) config = {
	.header = {
		.signature = JAILHOUSE_SYSTEM_SIGNATURE,
		.revision = JAILHOUSE_CONFIG_REVISION,
		.flags = JAILHOUSE_SYS_VIRTUAL_DEBUG_CONSOLE,
		.hypervisor_memory = {
			.phys_start = 0x7c000000,
			.size = 0x4000000,
		},
		.debug_console = {
			.address = 0x01c28000,
			.size = 0x1000,
			/* .clock_reg = 0x01c2006c, */
			/* .gate_nr = 16 */
			/* .divider = 0x0d, */
			.type = JAILHOUSE_CON_TYPE_8250,
			.flags = JAILHOUSE_CON_ACCESS_MMIO |
				 JAILHOUSE_CON_REGDIST_4,
		},
		.platform_info = {
			.pci_mmconfig_base = 0x2000000,
			.pci_mmconfig_end_bus = 0,
			.pci_is_virtual = 1,
			.arm = {
				.gic_version = 2,
				.gicd_base = 0x01c81000,
				.gicc_base = 0x01c82000,
				.gich_base = 0x01c84000,
				.gicv_base = 0x01c86000,
				.maintenance_irq = 25,
			},
		},
		.root_cell = {
			.name = "Banana-Pi",

			.cpu_set_size = sizeof(config.cpus),
			.num_memory_regions = __ARRAY_SIZE(config.mem_regions),
			.num_irqchips = __ARRAY_SIZE(config.irqchips),
			.num_pci_devices = __ARRAY_SIZE(config.pci_devices),

			.vpci_irq_base = 108,
		},
	},

	.cpus = {
		0x3,
	},

	.mem_regions = {
		/* IVSHMEM shared memory regions (demo) */
		{
			.phys_start = 0x7bf00000,
			.virt_start = 0x7bf00000,
			.size = 0x1000,
			.flags = JAILHOUSE_MEM_READ,
		},
		{
			.phys_start = 0x7bf00000 + 0x1000,
			.virt_start = 0x7bf00000 + 0x1000,
			.size = 0x1000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE,
		},
		{ 0 },
		{ 0 },
		/* SPI */ {
			.phys_start = 0x01c05000,
			.virt_start = 0x01c05000,
			.size = 0x00001000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO,
		},
		/* PIO */ {
			.phys_start = 0x01C20800,
			.virt_start = 0x01C20800,
			.size = 0xfc,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO,
		},
		/* Security System */ {
			.phys_start = 0x01C15000,
			.virt_start = 0x01C15000,
			.size = 0x00001000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO,
		},
		/* MMC */ {
			.phys_start = 0x01c0f000,
			.virt_start = 0x01c0f000,
			.size = 0x00001000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO,
		},
		/* USB 0 */ {
			.phys_start = 0x01C13000,
			.virt_start = 0x01C13000,
			.size = 0x00001000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO,
		},
		/* USB + PMU1 */ {
			.phys_start = 0x01c14000,
			.virt_start = 0x01c14000,
			.size = 0x00001000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO,
		},
		/* SATA */ {
			.phys_start = 0x01c18000,
			.virt_start = 0x01c18000,
			.size = 0x00001000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO,
		},
		/* USB + PMU2 */ {
			.phys_start = 0x01c1c000,
			.virt_start = 0x01c1c000,
			.size = 0x00001000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO,
		},
		/* CCU */ {
			.phys_start = 0x01c20000,
			.virt_start = 0x01c20000,
			.size = 0x400,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32,
		},
		/* Ints */ {
			.phys_start = 0x01c20400,
			.virt_start = 0x01c20400,
			.size = 0x400,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32,
		},
		/* GPIO: ports A-G */ {
			.phys_start = 0x01c20800,
			.virt_start = 0x01c20800,
			.size = 0xfc,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32,
		},
		/* GPIO: port H */ {
			.phys_start = 0x01c208fc,
			.virt_start = 0x01c208fc,
			.size = 0x24,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32,
		},
		/* GPIO: port I */ {
			.phys_start = 0x01c20920,
			.virt_start = 0x01c20920,
			.size = 0x24,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32,
		},
		/* GPIO: intr config */ {
			.phys_start = 0x01c20a00,
			.virt_start = 0x01c20a00,
			.size = 0x1c,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32,
		},
		/* Timer */ {
			.phys_start = 0x01c20c00,
			.virt_start = 0x01c20c00,
			.size = 0x400,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32,
		},
		/* UART0-3 */ {
			.phys_start = 0x01c28000,
			.virt_start = 0x01c28000,
			.size = 0x1000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO,
		},
		/* GMAC */ {
			.phys_start = 0x01c50000,
			.virt_start = 0x01c50000,
			.size = 0x00010000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO,
		},
		/* HSTIMER */ {
			.phys_start = 0x01c60000,
			.virt_start = 0x01c60000,
			.size = 0x00001000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO,
		},
		/* RAM */ {
			.phys_start = 0x40000000,
			.virt_start = 0x40000000,
			.size = 0x3bf00000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_EXECUTE,
		},
		/* TWI, PS2-0,PS2-1 memory region */ {
			.phys_start = 0x01c2a000,
			.virt_start = 0x01c2a000,
			.size = 4<<10,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32,
		},
		/* TP memory region */ {
			.phys_start = 0x01c25000,
			.virt_start = 0x01c25000,
			.size = 0x30,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32,
		},
		/* Digital Audio Interface (I2S, LRADC AD/DA) */ {
			.phys_start = 0x01c22000,
			.virt_start = 0x01c22000,
			.size = 0x1000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32
		},
		/* DMA */ {
			.phys_start = 0x01c02000,
			.virt_start = 0x01c02000,
			.size = 0x1000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32
		},
		/* SRAM controller */ {
			.phys_start = 0x01C00000,
			.virt_start = 0x01C00000,
			.size = 0x1000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32,
		},
		/* HDMI */ {
			.phys_start = 0x01C16000,
			.virt_start = 0x01C16000,
			.size = 0x1000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32,
		},
		/* LCD 0 */ {
			.phys_start = 0x01C0C000,
			.virt_start = 0x01C0C000,
			.size = 0x1000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32,
		},
		/* LCD 1 */ {
			.phys_start = 0x01C0D000,
			.virt_start = 0x01C0D000,
			.size = 0x1000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32,
		},
		/* DE_BE0 */ {
			.phys_start = 0x01E60000,
			.virt_start = 0x01E60000,
			.size = (128<<10),
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32,
		},
		/* Framebuffer */ {
			.phys_start = 0x7f736000,
			.virt_start = 0x7f736000,
			.size = 9<<20,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE | JAILHOUSE_MEM_IO,
		},
	},

	.irqchips = {
		/* GIC */ {
			.address = 0x01c81000,
			.pin_base = 32,
			.pin_bitmap = {
				0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff
			},
		},
	},

	.pci_devices = {
		{
			.type = JAILHOUSE_PCI_TYPE_IVSHMEM,
			.bdf = 0 << 3,
			.bar_mask = JAILHOUSE_IVSHMEM_BAR_MASK_INTX,
			.shmem_regions_start = 0,
			.shmem_dev_id = 0,
			.shmem_peers = 2,
			.shmem_protocol = JAILHOUSE_SHMEM_PROTO_UNDEFINED,
		},
	},
};
