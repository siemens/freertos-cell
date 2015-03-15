#ifndef _UART_PRINTF_H_
#define _UART_PRINTF_H_
void serial_init(void);
void serial_irq_rx_enable(void);
void serial_printf( const char *ctrl1, ...);
#endif
