#include "FreeRTOS.h"
#include "task.h"
#include "sio_ppp.h"
#include "serial.h"

/* functions for PPP interface */

static int do_a_read_abort = 0;
static TickType_t delay = pdMS_TO_TICKS(3);

void sio_timeout_set(sio_fd_t fd, unsigned millisec)
{
  (void)fd;
  delay = pdMS_TO_TICKS(millisec);
}

u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t c, cnt = 0;
  while(cnt < len && pdTRUE == xTaskNotifyWait(0, 0, &c, delay)) {
    *data++ = (u8_t)c;
    ++cnt;
    if(do_a_read_abort) {
      do_a_read_abort = 0;
      return 0;
    }
  }
  return cnt;
}

u32_t sio_write(sio_fd_t fd, u8_t *data, u32_t len)
{
  while(len-- > 0)
    serial_putchar(fd, *data++);
  return len;
}

void sio_read_abort(sio_fd_t fd)
{
  do_a_read_abort = 1;
}
