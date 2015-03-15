#ifndef _GIC_V2_H_
#define _GIC_V2_H_

#define GICD_OFFSET 0x1000
#define GICC_OFFSET 0x2000
#define GICH_OFFSET 0x4000
#define GICV_OFFSET 0x6000

#define GICD_ISENABLER      0x0100
#define GICD_ITARGETSR      0x0800
#define GICD_IPRIORITYR     0x0400
#define GICC_CTLR		0x0000
#define GICC_PMR		0x0004
#define GICC_CTLR_GRPEN1	(1 << 0)
#define GICC_PMR_DEFAULT	0xf0

int gic_v2_init(void);
void *gic_v2_gicd_get_address(void);
void *gic_v2_gicc_get_address(void);
void gic_v2_irq_enable(unsigned int irqn);
void gic_v2_irq_set_prio(int irqno, int prio);
void gic_v2_irq_activate_exclusive_on_cpu(int irqno, int prio);

#endif
