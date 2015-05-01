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

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "serial.h"

#ifndef CONFIG_MACH_SUN7I
#error Only support for Banana Pi board at the moment
#endif

#define UART7_BASE 0x01C29C00
#define UART_CLOCK_REG	((void *)0x01c2006c)
#define UART_GATE_NR	23

#define UART_TX			0x0
#define UART_DLL		0x0
#define UART_DLM		0x4
#define UART_IER		UART_DLM
#define UART_FCR		0x8
#define UART_LCR		0xc
#define UART_MCR		0x10
#define  UART_LCR_8N1		0x03
#define  UART_LCR_DLAB		0x80
#define UART_LSR		0x14
#define  UART_LSR_THRE		0x20

#define UART_CLK   (24*1000*1000)
#define UART_BAUDRATE  115200

/* Code is from linux kernel: drivers/tty/serial/8250/8250_early.c */
#define DIV_ROUND_CLOSEST(x, divisor)(      \
{             \
  typeof(x) __x = x;        \
  typeof(divisor) __d = divisor;      \
  (((typeof(x))-1) > 0 ||       \
   ((typeof(divisor))-1) > 0 || (__x) > 0) ?  \
    (((__x) + ((__d) / 2)) / (__d)) : \
    (((__x) - ((__d) / 2)) / (__d));  \
}             \
)

static uint32_t mmio_read32(void *addr)
{
  return *((volatile uint32_t*)addr);
}

static void mmio_write32(void *addr, uint32_t val)
{
  *((volatile uint32_t*)addr) = val;
}

sio_fd_t serial_open(void)
{
	unsigned divisor = DIV_ROUND_CLOSEST(UART_CLK, 16 * UART_BAUDRATE);
  sio_fd_t uart_base = (void*)UART7_BASE;

  mmio_write32(UART_CLOCK_REG,
      mmio_read32(UART_CLOCK_REG) |
      (1 << UART_GATE_NR));

	mmio_write32(uart_base + UART_LCR, UART_LCR_8N1);
	mmio_write32(uart_base + UART_IER, 0); /* IRQ off */
	mmio_write32(uart_base + UART_FCR, 7); /* FIFO reset and enable */
	mmio_write32(uart_base + UART_MCR, 7); /* DTR + RTS on */
	/* Set Divisor Latch Access Bit */
	mmio_write32(uart_base + UART_LCR, UART_LCR_DLAB | mmio_read32(uart_base + UART_LCR));
	/* Program baudrate */
	mmio_write32(uart_base + UART_DLL, 0xff & divisor); /* Divisor Latch Low Register */
	mmio_write32(uart_base + UART_DLM, 0xff & (divisor >> 8)); /* Divisor Latch High Register */
	mmio_write32(uart_base + UART_LCR, ~UART_LCR_DLAB & mmio_read32(uart_base + UART_LCR));
  return uart_base;
}

void serial_irq_rx_enable(sio_fd_t fd)
{
  void *uart_ier = fd + UART_IER;
  mmio_write32(uart_ier, 5 | mmio_read32(uart_ier)); /* ERBFI + ELSI */
}

static int serial_ready(sio_fd_t fd)
{
  uint32_t *uart_lsr = fd + UART_LSR;
  return UART_LSR_THRE & mmio_read32(uart_lsr); /* Transmit hold register empty */
}

void serial_putchar(sio_fd_t fd, uint32_t c)
{
  uint32_t *uart_tx = fd + UART_TX;
  while(!serial_ready(fd))
    ; /* Wait for empty transmit */
  mmio_write32(uart_tx, c);
}

int serial_irq_getchar(sio_fd_t fd)
{
  int r = 0;
  volatile uint32_t *uart_rbr = fd + 0x0; /* Receive buffer register */
  volatile uint32_t *uart_iir = fd + 0x8; /* INTERRUPT IDENTITY REGISTER */
  volatile uint32_t *uart_lsr = fd + 0x14;/* Line status register */
  volatile uint32_t *uart_usr = fd + 0x7C;/* UART status register */
  unsigned iir_val;
  iir_val = 0xf & *uart_iir;
  switch(iir_val) {
    case 0x7: /* Busy detect indication */
      printf("USR=%x\n\r", *uart_usr);
      break;
    case 0x6: /* Receiver line status */
      printf("LSR=%x\n\r", *uart_lsr);
      break;
    case 0x4: /* Received data available */
    case 12:  /* Character timeout indication */
      r = *uart_rbr;
      break;
    case 1: /* None */
      break;
    default:
      printf("UNHANDLED: %x\n\r", iir_val);
      break;
  }
  return r;
}
