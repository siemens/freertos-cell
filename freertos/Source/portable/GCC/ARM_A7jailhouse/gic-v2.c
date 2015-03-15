/* {{{1 Includes */
#include <stdint.h>

#include "gic-v2.h"
/* }}} */

void vPortInstallIrqStack(void);

/* {{{1 Local variables */
static void *gicc_base, *gicd_base;
/* }}} */

/* {{{1 GIC */
static void* get_gic_baseaddr(void)
{
  unsigned val;
  asm volatile("mrc p15, 4, %0, c15, c0, 0;"
      : "=r" (val)
      );
  /* FIXME Here we handle only 32 bit addresses. But there are 40 bit addresses with lpae */
  val >>= 15;
  val <<= 15;
  return (void*)val;
}

static uint32_t mmio_read32(void *addr)
{
  return *((volatile uint32_t *)addr);
}

static void mmio_write32(void *addr, uint32_t v)
{
  *((volatile uint32_t *)addr) = v;
}

int gic_v2_init(void)
{
  void *gicbase = get_gic_baseaddr();
  gicd_base = gicbase + GICD_OFFSET;
  gicc_base = gicbase + GICC_OFFSET;
  //printk("gicc_base=%x gicd_base=%x\n\r", (unsigned)gicc_base, (unsigned)gicd_base);
	mmio_write32(gicc_base + GICC_CTLR, GICC_CTLR_GRPEN1);
	mmio_write32(gicc_base + GICC_PMR, GICC_PMR_DEFAULT);
	return 0;
}

void *gic_v2_gicd_get_address(void)
{
  return gicd_base;
}

void *gic_v2_gicc_get_address(void)
{
  return gicc_base;
}

void gic_v2_irq_enable(unsigned int irqn)
{
  unsigned m ,n, val;
  void *address;
  m = irqn;
  n = m / 32;
  address = gicd_base + GICD_ISENABLER + 4*n;
  val = mmio_read32(address);
  val |= 1 << (m % 32);
	mmio_write32(address, val);
}

void gic_v2_irq_set_prio(int irqno, int prio)
{
  /* See doc: ARM Generic Interrupt Controller: Architecture Specification version 2.0
   * section 4.3.11
   */
  int n, m, offset;
  volatile uint8_t *gicd = gic_v2_gicd_get_address();
  m = irqno;
  n = m / 4;
  offset = GICD_IPRIORITYR + 4*n;
  offset += m % 4; /* Byte offset */
  //printk("IRQ%d prio original: 0x%x\n\r", irqno, (unsigned)gicd[offset]);
  gicd[offset] = 0xff;
  //printk("IRQ%d prio readback after 0xff: 0x%x\n\r", irqno, (unsigned)gicd[offset]);
  gicd[offset] = prio << 4;
  //printk("IRQ%d prio modified: 0x%x\n\r", irqno, (unsigned)gicd[offset]);
}

void gic_v2_irq_activate_exclusive_on_cpu(int irqno, int prio)
{
  volatile uint8_t *gicd = gic_v2_gicd_get_address() + GICD_ITARGETSR;
  int n, m, offset;
  m = irqno;
  //printk("gicd=%x CPUID=%d\n\r", (unsigned)gicd, (int)gicd[0]);
  n = m / 4;
  offset = 4*n;
  offset += m % 4;
  //printk("\tOrig GICD_ITARGETSR[%d]=%d\n\r",m, (int)gicd[offset]);
  gicd[offset] = gicd[0]; /* Exclusively route interrupt to the CPU running this code */
  //printk("\tNew  GICD_ITARGETSR[%d]=%d\n\r",m, (int)gicd[offset]);
  gic_v2_irq_set_prio(irqno, prio);
  gic_v2_irq_enable(irqno);
}
/* }}} */

/* vim:foldmethod=marker
 */
