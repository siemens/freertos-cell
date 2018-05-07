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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

void *memcpy(void *__dest, const void *__src, size_t __n)
{
	int i = 0;
	unsigned char *d = (unsigned char *)__dest, *s = (unsigned char *)__src;

	for (i = __n >> 3; i > 0; i--) {
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
	}

	if (__n & 1 << 2) {
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
	}

	if (__n & 1 << 1) {
		*d++ = *s++;
		*d++ = *s++;
	}

	if (__n & 1)
		*d++ = *s++;

	return __dest;
}

void *memmove(void *__dest, __const void *__src, size_t count)
{
	unsigned char *d = __dest;
	const unsigned char *s = __src;

	if (__dest == __src)
		return __dest;

	if (__dest < __src)
		return memcpy(__dest, __src, count);

	while (count--)
		d[count] = s[count];
	return __dest;
}

size_t strlen(const char *s)
{
	const char *sc = s;

	while (*sc != '\0')
		sc++;
	return sc - s;
}

int memcmp(const void *cs, const void *ct, size_t count)
{
	const unsigned char *su1 = cs, *su2 = ct, *end = su1 + count;
	int res = 0;

	while (su1 < end) {
		res = *su1++ - *su2++;
		if (res)
			break;
	}
	return res;
}

#if 0
int strcmp(const char *cs, const char *ct)
{
	unsigned char c1, c2;
	int res = 0;

	do {
		c1 = *cs++;
		c2 = *ct++;
		res = c1 - c2;
		if (res)
			break;
	} while (c1);
	return res;
}
#endif

void *memchr(const void *s, int c, size_t count)
{
	const unsigned char *p = s;

	while (count--)
		if ((unsigned char)c == *p++)
			return (void *)(p - 1);
	return NULL;
}

#if 0
char *strchr(const char *s, int c)
{
	while (*s != (char)c)
		if (*s++ == '\0')
			return NULL;
	return (char *)s;
}
#endif

void *memset(void *s, int c, size_t count)
{
  uint8_t u8 = (uint8_t)c;
  uint32_t u32 = (uint32_t)c;
  u32 = (u32 << 24) | (u32 << 16) | (u32 << 8) | u32;
  uint8_t *sp = (uint8_t *)s;
  /* Handle unaligned head */
  while (count && ((uintptr_t)sp & 3)) {
    count--;
    *sp++ = u8;
  }
  /* Faster 32 bit copy */
  uint32_t *lp = (uint32_t *)(void *)sp;
  while (count >= 4) {
    *lp++ = u32;
    count -= 4;
  }
  /* Handle tail */
  sp = (uint8_t *)(void *)lp;
  while (count--)
    *sp++ = u8;
  return s;
}
