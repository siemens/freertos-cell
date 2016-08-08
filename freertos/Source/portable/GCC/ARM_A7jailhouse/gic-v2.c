/* {{{1 License
    FreeRTOS V8.2.0 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

	***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
	***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
	the FAQ page "My application does not run, what could be wrong?".  Have you
	defined configASSERT()?

	http://www.FreeRTOS.org/support - In return for receiving this top quality
	embedded software for free we request you assist our global community by
	participating in the support forum.

	http://www.FreeRTOS.org/training - Investing in training allows your team to
	be as productive as possible as early as possible.  Now you can receive
	FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
	Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!

    Author:
      Dr. Johann Pfefferl <johann.pfefferl@siemens.com>
      Siemens AG
}}} */

/* {{{1 Includes */
#include <stdint.h>
#include <stdio.h>

#include "gic-v2.h"
/* }}} */

/* {{{1 Prototypes */
void vPortInstallIrqStack(void);
/* }}} */

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

void *gic_v2_init(void)
{
  void *gicbase = get_gic_baseaddr();
  gicd_base = gicbase + GICD_OFFSET;
  gicc_base = gicbase + GICC_OFFSET;
  printf("gicc_base=%x gicd_base=%x\n\r", (unsigned)gicc_base, (unsigned)gicd_base);
  // Global enable forwarding interrupts from distributor to cpu interface
	mmio_write32(gicd_base + GICD_CTLR, GICD_CTLR_GRPEN1);
  // Global enable signalling of interrupt from the cpu interface
	mmio_write32(gicc_base + GICC_CTLR, GICC_CTLR_GRPEN1);
	mmio_write32(gicc_base + GICC_PMR, GICC_PMR_DEFAULT);
	return gicbase;
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
  printf("IRQ%d prio original: 0x%x\n\r", irqno, (unsigned)gicd[offset]);
  gicd[offset] = 0xff;
  printf("IRQ%d prio readback after 0xff: 0x%x\n\r", irqno, (unsigned)gicd[offset]);
  gicd[offset] = prio << 4;
  printf("IRQ%d prio modified: 0x%x\n\r", irqno, (unsigned)gicd[offset]);
}

void gic_v2_irq_activate_exclusive_on_cpu(int irqno, int prio)
{
  volatile uint8_t *gicd = gic_v2_gicd_get_address() + GICD_ITARGETSR;
  int n, m, offset;
  m = irqno;
  printf("gicd=%x CPUID=%d\n\r", (unsigned)gicd, (int)gicd[0]);
  n = m / 4;
  offset = 4*n;
  offset += m % 4;
  printf("\tOrig GICD_ITARGETSR[%d]=%d\n\r",m, (int)gicd[offset]);
  gicd[offset] = gicd[0]; /* Exclusively route interrupt to the CPU running this code */
  printf("\tNew  GICD_ITARGETSR[%d]=%d\n\r",m, (int)gicd[offset]);
  gic_v2_irq_set_prio(irqno, prio);
  gic_v2_irq_enable(irqno);
}
/* }}} */

/* vim:foldmethod=marker
 */
