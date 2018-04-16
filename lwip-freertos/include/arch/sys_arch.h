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
#ifndef __SYS_RTXC_H__
#define __SYS_RTXC_H__

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define SYS_MBOX_NULL (xQueueHandle)0
#define SYS_SEM_NULL  (xSemaphoreHandle)0
#define SYS_DEFAULT_THREAD_STACK_DEPTH	configMINIMAL_STACK_SIZE

typedef xSemaphoreHandle sys_sem_t;
typedef xQueueHandle sys_mutex_t;

typedef xQueueHandle sys_mbox_t;
typedef xTaskHandle sys_thread_t;

typedef struct _sys_arch_state_t
{
	// Task creation data.
	char cTaskName[configMAX_TASK_NAME_LEN];
	unsigned short nStackDepth;
	unsigned short nTaskCount;
} sys_arch_state_t;

/*
  This optional function does a "fast" critical region protection and returns
  the previous protection level. This function is only called during very short
  critical regions. An embedded system which supports ISR-based drivers might
  want to implement this function by disabling interrupts. Task-based systems
  might want to implement this by using a mutex or disabling tasking. This
  function should support recursive calls from the same task or interrupt. In
  other words, sys_arch_protect() could be called while already protected. In
  that case the return value indicates that it is already protected.

  sys_arch_protect() is only required if your port is supporting an operating
  system.
*/
static inline sys_prot_t sys_arch_protect(void)
{
	portENTER_CRITICAL();
	return 1;
}

/*
  This optional function does a "fast" set of critical region protection to the
  value specified by pval. See the documentation for sys_arch_protect() for
  more information. This function is only required if your port is supporting
  an operating system.
*/
static inline void sys_arch_unprotect(sys_prot_t pval)
{
  (void)pval;
	portEXIT_CRITICAL();
}

static inline u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
  void *dummyptr;

  if (msg == NULL)
    msg = &dummyptr;

  return (pdTRUE == xQueueReceive(*mbox, msg, 0)) ? 0 : SYS_MBOX_EMPTY;
}

/*-----------------------------------------------------------------------------------*/
//   Posts the "msg" to the mailbox.
static inline void sys_mbox_post(sys_mbox_t *mbox, void *data)
{
	xQueueSend( *mbox, &data, portMAX_DELAY );
}

static inline err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
	return pdTRUE == xQueueSend(*mbox, &msg, 0) ? ERR_OK : ERR_MEM;
}

static inline err_t sys_mbox_trypost_fromisr(sys_mbox_t *mbox, void *msg)
{
  BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
  err_t err = pdTRUE == xQueueSendToBackFromISR(*mbox, &msg, &pxHigherPriorityTaskWoken) ? ERR_OK : ERR_MEM; 
  portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
  return err;
}

/*-----------------------------------------------------------------------------------*/
// Signals a semaphore
static inline void sys_sem_signal(sys_sem_t *sem)
{
	xSemaphoreGive( *sem );
}

static inline u32_t sys_now(void)
{
  return xTaskGetTickCount();
}

// It is called extremely often
#define sys_sem_valid(sem)            (NULL != *(sem))
#define sys_mbox_valid(mbox)      (NULL != *(mbox))
#define sys_mutex_valid(mutex)      (NULL != *(mutex))
#define sys_sem_set_invalid(sem)    (*(sem) = NULL)
#define sys_mbox_set_invalid(mbox)    (*(mbox) = NULL)
#define sys_mutex_set_invalid(mutex)  (*(mutex) = NULL)

/** Lock a mutex
 * @param mutex the mutex to lock */
static inline void sys_mutex_lock(sys_mutex_t *mutex)
{
  while(pdPASS != xSemaphoreTake(*mutex, portMAX_DELAY))
    ; /* Try endless */
}

/** Unlock a mutex
 * @param mutex the mutex to unlock */
static inline void sys_mutex_unlock(sys_mutex_t *mutex)
{
  xSemaphoreGive(*mutex);
}

#endif /* __SYS_RTXC_H__ */
