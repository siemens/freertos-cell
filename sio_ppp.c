#include "FreeRTOS.h"
#include "queue.h"
#include "sio_ppp.h"
#include "serial.h"

/* functions for PPP interface */

static TickType_t delay = pdMS_TO_TICKS(3);

sio_fd_t sio_open(int num)
{
  extern sio_fd_t ser_dev;
  return ser_dev;
}

void sio_timeout_set(sio_fd_t fd, unsigned millisec)
{
  (void)fd;
  delay = pdMS_TO_TICKS(millisec);
}

static void green_led_toggle(void)
{
#ifdef CONFIG_MACH_SUN7I
#define PIO_BASE ((void*)0x01c20800)
  uint32_t *led_reg = PIO_BASE + 7*0x24 + 0x10;
  *led_reg ^= 1<<24;
#endif
}

static QueueHandle_t ser_rx_queue;

void sio_queue_register(sio_fd_t fd, QueueHandle_t qh)
{
  (void)fd;
  ser_rx_queue = qh;
}

u32_t sio_tryread(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t cnt = 0;
  u8_t c;
  while(cnt < len && pdTRUE == xQueueReceive(ser_rx_queue, &c, delay)) {
    *data++ = c;
    ++cnt;
    green_led_toggle();
  }
  return cnt;
}

u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t cnt = 0;
  u8_t c;
  while(cnt < len && pdTRUE == xQueueReceive(ser_rx_queue, &c, portMAX_DELAY)) {
    *data++ = c;
    ++cnt;
    green_led_toggle();
  }
  return cnt;
}

void sio_send(u8_t c, sio_fd_t fd)
{
  serial_putchar(fd, c);
}

u32_t sio_write(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t l = len;
  while(l-- > 0)
    serial_putchar(fd, *data++);
  return len;
}
