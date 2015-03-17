#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include "serial.h"

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

void serial_init(void)
{
	unsigned divisor = DIV_ROUND_CLOSEST(UART_CLK, 16 * UART_BAUDRATE);
  void *uart_base = (void*)UART7_BASE;

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
}

void serial_irq_rx_enable(void)
{
  void *uart_ier = (void*)UART7_BASE + UART_IER;
  mmio_write32(uart_ier, 5 | mmio_read32(uart_ier)); /* ERBFI + ELSI */
}

void serial_putchar(uint32_t c)
{
  uint32_t *uart_tx = (void*)(UART7_BASE + UART_TX);
  uint32_t *uart_lsr = (void*)(UART7_BASE + UART_LSR);
redo:
  while(!(UART_LSR_THRE & mmio_read32(uart_lsr)))
    ; /* Wait for empty transmit */
  mmio_write32(uart_tx, c);
  if('\n' == c) {
    c = '\r'; /* Append a "carriage return" */
    goto redo;
  }
}
