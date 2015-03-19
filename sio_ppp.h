#ifndef _SIO_PPP_C_
#define _SIO_PPP_C_
#include "arch/cc.h"
#include "serial.h"
/* functions for PPP interface */
void sio_timeout_set(sio_fd_t fd, unsigned millisec);
u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len);
u32_t sio_write(sio_fd_t fd, u8_t *data, u32_t len);
void sio_read_abort(sio_fd_t fd);
#endif
