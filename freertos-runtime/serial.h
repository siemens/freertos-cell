#ifndef _UART_PRINTF_H_
#define _UART_PRINTF_H_
#include <stdint.h>
#include "arch/cc.h"

#define UART7_IRQ 52

typedef void * sio_fd_t;

sio_fd_t serial_open(void);
void serial_irq_rx_enable(sio_fd_t fd);
void serial_putchar(sio_fd_t fd, uint32_t c);
void serial_printf(sio_fd_t fd, const char *ctrl1, ...);
int serial_irq_getchar(sio_fd_t fd);
/* functions for PPP interface */
u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len);
u32_t sio_write(sio_fd_t fd, u8_t *data, u32_t len);
void sio_read_abort(sio_fd_t fd);
#endif
