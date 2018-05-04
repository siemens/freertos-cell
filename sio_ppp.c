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

#include "FreeRTOS.h"
#include "queue.h"
#include "sio_ppp.h"
#include "serial.h"

/* functions for PPP interface */

static TickType_t delay = pdMS_TO_TICKS(3);

sio_fd_t sio_open(int num)
{
  extern sio_fd_t ser_dev;
  return ser_dev;
}

void sio_timeout_set(sio_fd_t fd, unsigned millisec)
{
  (void)fd;
  delay = pdMS_TO_TICKS(millisec);
}

static void green_led_toggle(void)
{
#ifdef CONFIG_MACH_SUN7I
#define PIO_BASE ((void*)0x01c20800)
  uint32_t *led_reg = PIO_BASE + 7*0x24 + 0x10;
  portENTER_CRITICAL();
  *led_reg ^= 1<<24;
  portEXIT_CRITICAL();
#endif
}

static QueueHandle_t ser_rx_queue;

void sio_queue_register(sio_fd_t fd, QueueHandle_t qh)
{
  (void)fd;
  ser_rx_queue = qh;
}

u32_t sio_tryread(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t cnt = 0;
  u8_t c;
  while(cnt < len && pdTRUE == xQueueReceive(ser_rx_queue, &c, delay)) {
    *data++ = c;
    ++cnt;
    if(0) green_led_toggle();
  }
  return cnt;
}

u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t cnt = 0;
  u8_t c;
  while(cnt < len && pdTRUE == xQueueReceive(ser_rx_queue, &c, portMAX_DELAY)) {
    *data++ = c;
    ++cnt;
    if(0) green_led_toggle();
  }
  return cnt;
}

void sio_send(u8_t c, sio_fd_t fd)
{
  serial_putchar(fd, c);
}

u32_t sio_write(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t l = len;
  while(l-- > 0)
    serial_putchar(fd, *data++);
  return len;
}
