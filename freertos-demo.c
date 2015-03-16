/* {{{1 Includes */
#include <stdint.h>
#include "sysregs.h"
#include "gic-v2.h"
#include "string.h"
#include "serial_printf.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/* }}} */

/* {{{1 Defines */
#define TIMER_IRQ 27
#define BEATS_PER_SEC configTICK_RATE_HZ
#define ARM_SLEEP asm volatile("wfi" : : : "memory")
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))

#define UART_BUFSIZE 72

#define UART_LOCK xSemaphoreTake(uart_sema, portMAX_DELAY)
#define UART_UNLOCK xSemaphoreGive(uart_sema)
#define UART_OUTPUT(args...) do { if(pdPASS == UART_LOCK) { serial_printf(args); UART_UNLOCK;} } while(0)

/* }}} */

/* {{{1 Prototypes */
void FreeRTOS_Tick_Handler( void );
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationIRQHandler( unsigned ulICCIAR );
void __div0(void);
/* }}} */

/* {{{1 Global variables */
static TaskHandle_t uart_task_handle;
static SemaphoreHandle_t uart_sema;
/* }}} */

/* {{{1 FreeRTOS debug hooks */

void vAssertCalled( const char * pcFile, unsigned long ulLine )
{
  volatile unsigned long ul = 0;

  ( void ) pcFile;
  ( void ) ulLine;

  vTaskSuspendAll();
  taskENTER_CRITICAL();
  {
    /* Set ul to a non-zero value using the debugger to step out of this
       function. */
    serial_printf("%s %s: line=%lu\n", __func__, pcFile, ulLine);
    while( ul == 0 ) {
      portNOP();
    }
  }
  taskEXIT_CRITICAL();
}

void vApplicationMallocFailedHook( void )
{
  /* Called if a call to pvPortMalloc() fails because there is insufficient
     free memory available in the FreeRTOS heap.  pvPortMalloc() is called
     internally by FreeRTOS API functions that create tasks, queues, software
     timers, and semaphores.  The size of the FreeRTOS heap is set by the
     configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
  taskDISABLE_INTERRUPTS();
  serial_printf("%s\n", __func__);
  while(1) {
    portNOP();
  }
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
  ( void ) pcTaskName;
  ( void ) pxTask;

  /* Run time stack overflow checking is performed if
     configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
     function is called if a stack overflow is detected. */
  vTaskSuspendAll();
  taskDISABLE_INTERRUPTS();
    serial_printf("%s task=%s\n", __func__, pcTaskName);
  for( ;; )
    ARM_SLEEP;
}

void __div0(void)
{
  serial_printf("PANIC: Div by zero error\n");
  ARM_SLEEP;
}

/* }}} */

/* {{{1 LED control */
static void led_toggle(void)
{
#ifdef CONFIG_MACH_SUN7I
#define PIO_BASE ((void*)0x01c20800)
  uint32_t *led_reg = PIO_BASE + 7*0x24 + 0x10;
  *led_reg ^= 1<<24;
#endif
}
/* }}} */

/* {{{1 Timer control */
static int32_t timer_value_for_period;
static unsigned timer_frq;

static inline void timer_on(void)
{
	arm_write_sysreg(CNTV_CTL_EL0, 1);
}

#if 0
static inline void timer_off(void)
{
	arm_write_sysreg(CNTV_CTL_EL0, 0);
}

static u64 get_actual_ticks(void)
{
  u64 pct64;
  arm_read_sysreg(CNTVCT, pct64);
  return pct64;
}

static inline unsigned ticks_to_ns(unsigned ticks)
{
  return (ticks*1000) / ( timer_frq/1000/1000);
}
#endif

static inline void timer_set(int32_t val)
{
	arm_write_sysreg(CNTV_TVAL_EL0, val);
}

static inline void timer_set_next_event(void)
{
  int32_t time_drift;
  /* The timer indicates an overtime with a negative value inside this register */
  arm_read_sysreg(CNTV_TVAL_EL0, time_drift);
  /* If the drift is greater than timer_value_for_period we have lost a time period */
  //configASSERT(-time_drift < timer_value_for_period);
  /* Correct next period by this time drift. The drift is caused by the software */
	timer_set(timer_value_for_period + time_drift);
}

/* Function called by FreeRTOS_Tick_Handler as last action */
void vClearTickInterrupt(void)
{
  timer_set_next_event();
	//timer_on();
}

static int timer_init(unsigned beats_per_second)
{
	timer_value_for_period = timer_frq / beats_per_second;
	timer_set(timer_value_for_period);
  timer_on();

	return 0;
}
/* }}} */

/* {{{1 UART handling */
static void serial_print(char *buf, int n)
{
  buf[n] = 0;
  UART_OUTPUT("TUA\t%d %s\n", n, buf);
}

static void uartTask(void *pvParameters)
{
  uint32_t c;
  char s[80];
  int idx = 0;
  while(pdTRUE) {
    if(pdTRUE == xTaskNotifyWait(0, 0, &c, pdMS_TO_TICKS(250))) {
      led_toggle();
      s[idx] = c;
      if('\r' == s[idx] || idx >= sizeof(s)-1) {
        serial_print(s, idx);
        idx = 0;
      }
      else
        ++idx;
    }
    else if(idx) { /* Buffer not empty */
      serial_print(s, idx);
      idx = 0;
    }
  }
}

/* }}} */

/* {{{1 Interrupt handling */

void vConfigureTickInterrupt( void )
{
  /* Register the standard FreeRTOS Cortex-A tick handler as the timer's
     interrupt handler.  The handler clears the interrupt using the
     configCLEAR_TICK_INTERRUPT() macro, which is defined in FreeRTOSConfig.h. */
	gic_v2_irq_set_prio(TIMER_IRQ, portLOWEST_USABLE_INTERRUPT_PRIORITY);
	gic_v2_irq_enable(TIMER_IRQ);
	timer_init(BEATS_PER_SEC);
}


static void handle_uart_irq(void)
{
#ifdef CONFIG_MACH_SUN7I
  volatile uint32_t *uart_rbr = (void*)(UART7_BASE + 0x0); /* Receive buffer register */
  volatile uint32_t *uart_iir = (void*)(UART7_BASE + 0x8); /* INTERRUPT IDENTITY REGISTER */
  volatile uint32_t *uart_lsr = (void*)(UART7_BASE + 0x14);/* Line status register */
  volatile uint32_t *uart_usr = (void*)(UART7_BASE + 0x7C);/* UART status register */
  uint32_t iir_val;
  iir_val = 0xf & *uart_iir;
  switch(iir_val) {
    case 0x7: /* Busy detect indication */
      serial_printf("USR=%x\n", *uart_usr);
      break;
    case 0x6: /* Receiver line status */
      serial_printf("LSR=%x\n", *uart_lsr);
      break;
    case 0x4: /* Received data available */
    case 12:  /* Character timeout indication */
      {
        uint32_t v = *uart_rbr;
        BaseType_t do_yield = pdFALSE;
        xTaskNotifyFromISR(uart_task_handle, v, eSetValueWithOverwrite, &do_yield);
        portYIELD_FROM_ISR(do_yield);
      }
      break;
    case 1: /* None */
      break;
    default:
      serial_printf("UNHANDLED: %x\n", iir_val);
      break;
  }
#endif
}

void vApplicationIRQHandler(unsigned int irqn)
{
  switch(irqn) {
    case TIMER_IRQ:
      //timer_off();
      FreeRTOS_Tick_Handler();
      break;
    case UART7_IRQ:
      handle_uart_irq();
      break;
    case 0x3ff:
      /* This irq should be ignored. It is no longer relevant */
      break;
    default:
      serial_printf("Spurious irq %d\n", irqn);
      break;
  }
}

/* }}} */

/* {{{1 FreeRTOS application tasks */

static void testTask( void *pvParameters )
{
  unsigned id = (unsigned)pvParameters;
  TickType_t period = ++id * pdMS_TO_TICKS(100);
  unsigned cnt = 0;
  TickType_t pxPreviousWakeTime = xTaskGetTickCount();
  while(pdTRUE) {
    UART_OUTPUT("T%02u\tperiod:%5u;\tloop:%5u;\ttick:%6u\n", id, period, cnt++, xTaskGetTickCount());
#if 0
    if(0x7 == (0x7 & cnt)) /* Force a task switch */
      taskYIELD();
#else
    vTaskDelayUntil(&pxPreviousWakeTime, period);
#endif
  }
  vTaskDelete( NULL );
}

static void blinkTask(void *pvParameters)
{
  TickType_t pxPreviousWakeTime = xTaskGetTickCount();
  while(1) {
    led_toggle();
    vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(250));
  }
}

static void sendTask(void *pvParameters)
{
  TaskHandle_t recvtask = pvParameters;
  TickType_t pxPreviousWakeTime = xTaskGetTickCount();
  while(1) {
    UART_OUTPUT("Sending ...\n");
    xTaskNotify(recvtask, 0, eIncrement);
    vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(1000));
  }
}

static void recvTask(void *pvParameters)
{
  while(1) {
    uint32_t value;
    if(pdTRUE == xTaskNotifyWait(0, 0, &value, portMAX_DELAY)) {
      UART_OUTPUT("Value received: %u\n", value);
    }
    else {
      serial_printf("No value received\n");
    }
  }
}

static void floatTask( void *pvParameters )
{
  portFLOAT c, d;
  unsigned cnt = 0;
  int id = (int)pvParameters;
  TickType_t pxPreviousWakeTime = xTaskGetTickCount();
  portTASK_USES_FLOATING_POINT();
  c = 1.;
  while(pdTRUE) {
    d = 1e6 * (c - (unsigned)c);
    UART_OUTPUT("FT%d: 1.11^%d=%4d.%06d\n", id, cnt++, (unsigned)c, (unsigned)d);
    vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(1000));
    if(cnt < 133)
      c *= 1.11;
    else {
      c = 1.;
      cnt = 0;
    }
  }
  vTaskDelete( NULL );
}

/* }}} */

/* {{{1 Hardware init */
static void hardware_fpu_enable(void)
{
  unsigned reg;
  /* Enable the VFP */
  asm volatile("mrc	p15, 0, %0, c1, c0, 2;" /* Read Coprocessor Access Control Register CPACR */
      "orr	%0, %0, #(0x3 << 20);"    /* Enable access to cp10 */
      "orr	%0, %0, #(0x3 << 22);"    /* Enable access to cp11 */
      "mcr	p15, 0, %0, c1, c0, 2;"
      "vmrs	%0, FPEXC;"
      "orr	%0, %0, #(1<<30);"  /* Set FPEXC.EN = 1 */
      "vmsr	FPEXC, %0;"
      : "=r" (reg) /* outputs */
      : /* No inputs */
      : /* clobbered */
      );
}

static void hardware_cpu_cache_mmu_enable(void)
{
  /* 3. Enable I/D cache + branch prediction + MMU */
  asm volatile(
      "mov r0, #0;"
      "mcr p15, 0, r0, c8, c3, 0;" // Issue TLBIALL (TLB Invalidate All)
      "mrc p15, 0, r0, c1, c0, 0;" // System control register
      "orr r0, r0, #(1 << 12);" // Instruction cache enable
      "orr r0, r0, #(1 << 11);" // Program flow prediction
      "orr r0, r0, #(1 << 2);"  // d-cache & L2-$ on
      "orr r0, r0, #(1 << 0);"  // MMU on
      "mcr p15, 0, r0, c1, c0, 0;" // System control register
      "isb; dsb;"
      : /* Outputs */
      : /* Inputs */
      : "r0" /* clobbered */
      );
}

static void show_cache_mmu_status(const char *header)
{
  unsigned scr;

  asm volatile("dsb;isb;mrc p15, 0, %0, c1, c0, 0;" : "=r" (scr) : /* Inputs */ : /* clobber */);
  serial_printf("===== %s =====\n", header);
  serial_printf("\tIcache %u\n", !!(scr & (1<<12)));
  serial_printf("\tFlow   %u\n", !!(scr & (1<<11)));
  serial_printf("\tDcache %u\n", !!(scr & (1<<2)));
  serial_printf("\tMMU    %u\n", !!(scr & (1<<0)));
}

static void hardware_cpu_caches_off(void)
{
  /* 1. MMU, L1$ disable */
  asm volatile("mrc p15, 0, r0, c1, c0, 0;" // System control register
      "bic r0, r0, #(1 << 12);" // Instruction cache disable
      "bic r0, r0, #(1 << 11);" // Program flow prediction
      "bic r0, r0, #(1 << 2);" // d-cache & L2-$ off
      "bic r0, r0, #(1 << 0);" // mmu off
      "mcr p15, 0, r0, c1, c0, 0;" // System control register
      : /* Outputs */
      : /* Inputs */
      : "r0" /* clobbered */
      );
  /* 2. invalidate: L1$, TLB, branch predictor */
  asm volatile("mov r0, #0;"
      "mcr p15, 0, r0, c8, c7, 0;" /* Invalidate entire Unified Main TLB */
      "mcr p15, 0, r0, c8, c6, 0;" /* Invalidate entire data TLB */
      "mcr p15, 0, r0, c8, c5, 0;" /* Invalidate entire instruction TLB */
      "mcr p15, 0, r0, c7, c5, 0;" /* Invalidate Instruction Cache */
      "mcr p15, 0, r0, c7, c5, 6;" /* Invalidate branch prediction array */
      "dsb;" /* Data sync barrier */
      "isb;" /* Instruction sync barrier */
      : /* Outputs */
      : /* Inputs */
      : "r0" /* clobbered */
      );
}

static void hardware_mmu_ptable_setup(unsigned long iomem[], int n)
{
  /* See: http://www.embedded-bits.co.uk/2011/mmucode/ */
  int i;
  /* We use only TTBR0 from the ARM cpu. Therefore we manage a page size of 1MB.
   * To map the whole 4GB DDR3 address space we need 4096 entries in the page table
   */
  static uint32_t mmu_pgtable[4096] __attribute__((aligned(16<<10)));
  serial_printf("MMU page table: %p\n", mmu_pgtable);
  /* Create a MMU identity map for the whole 4GB address space */
  for(i = 0; i < ARRAY_SIZE(mmu_pgtable); i++) {
    mmu_pgtable[i] = i<<20; /* Section base address: one section is 1MB */
    mmu_pgtable[i] |= 2<<0; /* This is a 1MB section entry */
    /* See "ARM Architecture Reference Manual" section B3.8 */
    mmu_pgtable[i] |= 5<<10; /* TEX (Type Extension): Outer attribute Write-Back, Write-Allocate */
    mmu_pgtable[i] |= 1<<2; /* Inner attribute (aka C,B): Write-Back, Write-Allocate */
    //mmu_pgtable[i] |= 0<<5; /* Domain */
    mmu_pgtable[i] |= 3<<10; /* Access permissions: AP[2]=0 AP[1:0]=0b11 full access (see Table B3-8) */
    //mmu_pgtable[i] |= 1<<15; /* AP[2] */
    //mmu_pgtable[i] |= 1<<16; /* Shareable */
  }
  /* Do not cache peripheral IO memory sections */
  for(i = 0; i < n; i++) {
    int idx = iomem[i] >> 20;
    serial_printf("%s: [%d]=0x%x\n", __func__, i, idx << 20);
    /* Non-shareable Device: TEX = 0b010 CB = 0b00 */
    mmu_pgtable[idx] &= ~(3<<2); /* Clear C/B bits */
    mmu_pgtable[idx] &= ~(7<<10); /* Clear TEX */
    mmu_pgtable[idx] |= 2<<10; /* TEX = 0b010 */
  }
  asm volatile(
      "mov r1, %0;"
      "orr r1, #(1<<3);" /* Outer region bits: Normal memory, Outer Write-Back Write-Allocate Cacheable. */
      "orr r1, #((0<<6) | (1<<0));" /* Inner region bits: Normal memory, Inner Write-Back Write-Allocate Cacheable. */
      "mcr p15, 0, r1, c2, c0, 0;" /* Set page table address */
      "mov r1, #0x1;"  /* Set access permissions for the domain to "client" */
      "mcr p15, 0, r1, c3, c0, 0;"
      "mrc p15, 0, r1, c2, c0, 2;" /* Read TTBCR */
      "bic r1, #(1<<31);" /* No Extended Address Enable: 32-bit translation system */
      "orr r1, #(1<<5);" /* PD1; TTBR1 should not be used */
      "bic r1, #(1<<4);" /* PD0 */
      "mcr p15, 0, r1, c2, c0, 2;" /* Write TTBCR */
      : /* outputs */
      : "r" (mmu_pgtable) /* inputs */
      : "r0", "r1" /* clobbered */
      );
}

static void uart_irq_enable(void)
{
  volatile uint8_t *gicd = gic_v2_gicd_get_address() + GICD_ITARGETSR;
  int n, m, offset;
  m = UART7_IRQ;
  serial_printf("UART gicd=%p CPUID=%d\n", gicd, (int)gicd[0]);
  n = m / 4;
  offset = 4*n;
  offset += m % 4;
  serial_printf("\tOrig GICD_ITARGETSR[%d]=%d\n",m, (int)gicd[offset]);
  gicd[offset] |= gicd[0];
  serial_printf("\tNew  GICD_ITARGETSR[%d]=%d\n",m, (int)gicd[offset]);
  gic_v2_irq_set_prio(UART7_IRQ, portLOWEST_USABLE_INTERRUPT_PRIORITY);
  gic_v2_irq_enable(UART7_IRQ);
  //ARM_SLEEP;
}

#define USE_CACHE_MMU 1

static void prvSetupHardware(void)
{
  unsigned apsr;
  static unsigned long io_dev_map[2];

  serial_init();
  show_cache_mmu_status("MMU/Cache status at entry");
  serial_printf("Initializing the HW...\n");
  if(USE_CACHE_MMU) hardware_cpu_caches_off();
  gic_v2_init();
  io_dev_map[0] = (unsigned long)gic_v2_gicd_get_address();
#ifdef CONFIG_MACH_SUN7I
  io_dev_map[1] = UART7_BASE;
#else
  io_dev_map[1] = io_dev_map[0]; // duplicate entry from GIC
#endif
  if(USE_CACHE_MMU) hardware_mmu_ptable_setup(io_dev_map, ARRAY_SIZE(io_dev_map));
  if(USE_CACHE_MMU) hardware_cpu_cache_mmu_enable();
  /* Replace the exception vector table by a FreeRTOS variant */
  vPortInstallFreeRTOSVectorTable();
  hardware_fpu_enable();
  uart_irq_enable();
  serial_irq_rx_enable();
  arm_read_sysreg(CNTFRQ, timer_frq);
  if(!timer_frq) {
    serial_printf("Timer frequency is zero\n");
    ARM_SLEEP;
  }
  asm volatile ( "mrs %0, apsr" : "=r" ( apsr ) );
  apsr &= 0x1f;
  serial_printf("FreeRTOS inmate cpu-mode=%x\n", apsr);
  show_cache_mmu_status("MMU/Cache status at runtime");
}
/* }}} */

/* {{{1 main */
void inmate_main(void)
{
  unsigned i;

  prvSetupHardware();
  vSemaphoreCreateBinary(uart_sema);

  xTaskCreate( uartTask, /* The function that implements the task. */
      "uartstat", /* The text name assigned to the task - for debug only; not used by the kernel. */
      configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
      NULL,                                                            /* The parameter passed to the task */
      configMAX_PRIORITIES-1, /* The priority assigned to the task. */
      &uart_task_handle );

  if(1) for(i = 0; i < 20; i++) {
    int prio = 1 + i % (configMAX_PRIORITIES-1);
    serial_printf("Create task %u with prio %d\n", i, prio);
    xTaskCreate( testTask, /* The function that implements the task. */
        "test", /* The text name assigned to the task - for debug only; not used by the kernel. */
        configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
        (void*)i, 								/* The parameter passed to the task */
        prio, /* The priority assigned to the task. */
        NULL );								    /* The task handle is not required, so NULL is passed. */
  }

  if(1) { /* Task notification test */
    TaskHandle_t recv_task_handle;
    xTaskCreate( recvTask, /* The function that implements the task. */
        "receive", /* The text name assigned to the task - for debug only; not used by the kernel. */
        configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
        NULL, 								/* The parameter passed to the task */
        configMAX_PRIORITIES-2, /* The priority assigned to the task. */
        &recv_task_handle );		/* The task handle */
    xTaskCreate( sendTask, /* The function that implements the task. */
        "sender", /* The text name assigned to the task - for debug only; not used by the kernel. */
        configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
        recv_task_handle, 				/* The parameter passed to the task */
        configMAX_PRIORITIES-1, /* The priority assigned to the task. */
        NULL );								    /* The task handle is not required, so NULL is passed. */
  }
  xTaskCreate( blinkTask, /* The function that implements the task. */
      "blink", /* The text name assigned to the task - for debug only; not used by the kernel. */
      configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
      NULL, 								/* The parameter passed to the task */
      tskIDLE_PRIORITY, /* The priority assigned to the task. */
      NULL );								    /* The task handle is not required, so NULL is passed. */
  if(1) for(i = 0; i < 2; i++) {
    xTaskCreate( floatTask, /* The function that implements the task. */
        "float", /* The text name assigned to the task - for debug only; not used by the kernel. */
        configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
        (void*)i, 								/* The parameter passed to the task */
        tskIDLE_PRIORITY+1, /* The priority assigned to the task. */
        NULL );								    /* The task handle is not required, so NULL is passed. */
  }
  serial_printf("vTaskStartScheduler goes active\n");
  vTaskStartScheduler();
  serial_printf("vTaskStartScheduler terminated: strange!!!\n");
	while (1) {
    ARM_SLEEP;
  }
}
/* }}} */

/* vim:foldmethod=marker
 */
