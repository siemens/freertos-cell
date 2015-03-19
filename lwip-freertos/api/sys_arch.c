/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/* lwIP includes. */
#include "FreeRTOSConfig.h"
#include "lwip/debug.h"
#include "lwip/def.h"
#include "lwip/sys.h"
#include "lwip/mem.h"
#include "lwip/stats.h"

#include <stdio.h>

/* Message queue constants. */
#define archMESG_QUEUE_LENGTH	(4<<10)

/*-----------------------------------------------------------------------------------*/
//  Creates an empty mailbox.
err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
  if(size <= 0) {
    size = archMESG_QUEUE_LENGTH;
    printf("%s: size=%d\n", __func__, size);
  }
  else
    printf("%s: size=%d\n", __func__, size);
	*mbox = xQueueCreate( size, sizeof( void * ) );
	if (!*mbox) {
		SYS_STATS_INC(mbox.err);
		return ERR_MEM;
	}
  printf("%s: mbox=%p *mbox=%p\n", __func__, mbox, *mbox);

	SYS_STATS_INC_USED(mbox);
	return *mbox ? ERR_OK: ERR_MEM;
}

/*-----------------------------------------------------------------------------------*/
/*
  Deallocates a mailbox. If there are messages still present in the
  mailbox when the mailbox is deallocated, it is an indication of a
  programming error in lwIP and the developer should be notified.
*/
void sys_mbox_free(sys_mbox_t *mbox)
{
  printf("%s: mbox=%p *mbox=%p\n", __func__, mbox, *mbox);
	if( uxQueueMessagesWaiting( *mbox ) ) {
		/* Line for breakpoint.  Should never break here! */
		__asm__ __volatile__ ( "nop" );
	}

	vQueueDelete( *mbox );
  SYS_STATS_DEC(mbox.used);
}

/*-----------------------------------------------------------------------------------*/
/*
  Blocks the thread until a message arrives in the mailbox, but does
  not block the thread longer than "timeout" milliseconds (similar to
  the sys_arch_sem_wait() function). The "msg" argument is a result
  parameter that is set by the function (i.e., by doing "*msg =
  ptr"). The "msg" parameter maybe NULL to indicate that the message
  should be dropped.

  The return values are the same as for the sys_arch_sem_wait() function:
  Number of milliseconds spent waiting or SYS_ARCH_TIMEOUT if there was a
  timeout.

  Note that a function with a similar name, sys_mbox_fetch(), is
  implemented by lwIP.
*/
u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
void *dummyptr;
portTickType StartTime, EndTime, Elapsed;

	StartTime = xTaskGetTickCount();

	if( msg == NULL )
		msg = &dummyptr;
		
	if(	timeout != 0 )
	{
		if(pdTRUE == xQueueReceive( *mbox, &(*msg), timeout / portTICK_RATE_MS ) )
		{
			EndTime = xTaskGetTickCount();
			Elapsed = EndTime - StartTime;
      if(portTICK_RATE_MS != 1)
        Elapsed *= portTICK_RATE_MS;
			return Elapsed ? Elapsed: 1;
		}
		else // timed out blocking for message
		{
			*msg = NULL;
			return SYS_ARCH_TIMEOUT;
		}
	}
	else // block forever for a message.
	{
		while( pdTRUE != xQueueReceive( *mbox, msg, portMAX_DELAY ) ) // time is arbitrary
		{
			;
		}
		EndTime = xTaskGetTickCount();
		Elapsed = EndTime - StartTime;
    return Elapsed ? Elapsed: 1;
	}
}

/*-----------------------------------------------------------------------------------*/
//  Creates and returns a new semaphore. The "count" argument specifies
//  the initial state of the semaphore. TBD finish and test
err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
	// count 0 means a binary semaphore so max value should be 1
	*sem = xSemaphoreCreateCounting((count ? count : 1), count);

	if (*sem == NULL) {
		SYS_STATS_INC(sem.err);
		return ERR_MEM;  // TBD need assert
	}

	SYS_STATS_INC_USED(sem);

	return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/
/*
  Blocks the thread while waiting for the semaphore to be
  signaled. If the "timeout" argument is non-zero, the thread should
  only be blocked for the specified time (measured in
  milliseconds).

  If the timeout argument is non-zero, the return value is the number of
  milliseconds spent waiting for the semaphore to be signaled. If the
  semaphore wasn't signaled within the specified time, the return value is
  SYS_ARCH_TIMEOUT. If the thread didn't have to wait for the semaphore
  (i.e., it was already signaled), the function may return zero.

  Notice that lwIP implements a function with a similar name,
  sys_sem_wait(), that uses the sys_arch_sem_wait() function.
*/
u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
portTickType StartTime, EndTime, Elapsed;

	StartTime = xTaskGetTickCount();

	if(	timeout != 0)
	{
		if( xSemaphoreTake( *sem, timeout / portTICK_RATE_MS ) == pdTRUE )
		{
			EndTime = xTaskGetTickCount();
			Elapsed = EndTime - StartTime;
      if(portTICK_RATE_MS != 1)
        Elapsed *= portTICK_RATE_MS;
      return Elapsed ? Elapsed: 1;
		}
		else
			return SYS_ARCH_TIMEOUT;
	}
	else // must block without a timeout
	{
		while( xSemaphoreTake( *sem, portMAX_DELAY ) != pdTRUE )
		{
			;
		}
		EndTime = xTaskGetTickCount();
		Elapsed = EndTime - StartTime;
    if(portTICK_RATE_MS != 1)
      Elapsed *= portTICK_RATE_MS;
    return Elapsed ? Elapsed: 1;
	}
}

/*-----------------------------------------------------------------------------------*/
// Deallocates a semaphore
void sys_sem_free(sys_sem_t *sem)
{
	vQueueDelete( *sem );
  SYS_STATS_DEC(sem.used);
}

/*-----------------------------------------------------------------------------------*/
// Initialize sys arch
void sys_init(void)
{
  return;
}

/*-----------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------*/
// TBD
/*-----------------------------------------------------------------------------------*/
/*
  Starts a new thread with priority "prio" that will begin its execution in the
  function "thread()". The "arg" argument will be passed as an argument to the
  thread() function. The id of the new thread is returned. Both the id and
  the priority are system dependent.
*/
sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread, void *arg, int stacksize, int prio)
{
  xTaskHandle CreatedTask;
  int result;

  if(!stacksize)
    stacksize = configMINIMAL_STACK_SIZE;
  result = xTaskCreate(thread, (void*)name, stacksize, arg, prio, &CreatedTask );

  return pdPASS == result ? CreatedTask: NULL;
}

/** Create a new mutex
 * @param mutex pointer to the mutex to create
 * @return a new mutex */
err_t sys_mutex_new(sys_mutex_t *mutex)
{
  *mutex = xSemaphoreCreateMutex();
	if (!*mutex) {
		SYS_STATS_INC(mutex.err);
		return ERR_MEM;
	}

	SYS_STATS_INC_USED(mutex);

	return ERR_OK;
}

/** Delete a semaphore
 * @param mutex the mutex to delete */
void sys_mutex_free(sys_mutex_t *mutex)
{
	vQueueDelete( *mutex );
  SYS_STATS_DEC(mutex.used);
}
