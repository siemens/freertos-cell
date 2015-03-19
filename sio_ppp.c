#include "sio_ppp.h"

/* functions for PPP interface */

static int do_a_read_abort = 0;

u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
  return -1; // FIXME
}

u32_t sio_write(sio_fd_t fd, u8_t *data, u32_t len)
{
  return len; // FIXME
}

void sio_read_abort(sio_fd_t fd)
{
  do_a_read_abort = 1;
}
