#ifndef _UART_PRINTF_H_
#define _UART_PRINTF_H_

#define UART7_IRQ 52
#define UART7_BASE 0x01C29C00

void serial_init(void);
void serial_irq_rx_enable(void);
void serial_putchar(uint32_t c);
void serial_printf( const char *ctrl1, ...);
int serial_irq_getchar(void);
#endif
