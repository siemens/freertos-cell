#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include "serial_printf.h"

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

typedef struct params_s {
  int len;
  int num1;
  int num2;
  char pad_character;
  int do_padding;
  int left_flag;
} params_t;

typedef char* charptr;

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

static int isdigit(int c)
{
  return c >= '0' && c <= '9';
}

static void serial_putchar(uint32_t c)
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

static int tolower(int c)
{
  if(c >= 'A' && c <= 'Z')
    c += 'a' - 'A';
  return c;
}

static int getnum( charptr* linep)
{
  int n;
  charptr cp;

  n = 0;
  cp = *linep;
  while (isdigit((int)*cp))
    n = n*10 + ((*cp++) - '0');
  *linep = cp;
  return n;
}

static void padding( const int l_flag, params_t *par)
{
  int i;

  if (par->do_padding && l_flag && (par->len < par->num1))
    for (i=par->len; i<par->num1; i++) {
      serial_putchar( par->pad_character);
    }
}

static void serial_outs( charptr lp, params_t *par)
{
  /* pad on left if needed                         */
  par->len = strlen( lp);
  padding( !(par->left_flag), par);

  /* Move string to the buffer                     */
  while (*lp && (par->num2)--) {
    serial_putchar( *lp++);
  }

  /* Pad on right if needed                        */
  /* CR 439175 - elided next stmt. Seemed bogus.   */
  /* par->len = strlen( lp);                       */
  padding( par->left_flag, par);
}

static void serial_outnum( const long n, const long base, params_t *par)
{
  charptr cp;
  int negative;
  char outbuf[32];
  const char digits[] = "0123456789ABCDEF";
  unsigned long num;

  /* Check if number is negative                   */
  if (base == 10 && n < 0L) {
    negative = 1;
    num = -(n);
  }
  else{
    num = (n);
    negative = 0;
  }

  /* Build number (backwards) in outbuf            */
  cp = outbuf;
  do {
    *cp++ = digits[(int)(num % base)];
  } while ((num /= base) > 0);
  if (negative)
    *cp++ = '-';
  *cp-- = 0;

  /* Move the converted number to the buffer and   */
  /* add in the padding where needed.              */
  par->len = strlen(outbuf);
  padding( !(par->left_flag), par);
  while (cp >= outbuf) {
    serial_putchar( *cp--);
  }
  padding( par->left_flag, par);
}

void serial_printf( const char *ctrl1, ...)
{
  int long_flag;
  int dot_flag;

  params_t par;

  char ch;
  va_list argp;
  char *ctrl = (char *)ctrl1;

  va_start( argp, ctrl1);

  for ( ; *ctrl; ctrl++) {

    /* move format string chars to buffer until a  */
    /* format control is found.                    */
    if (*ctrl != '%') {
      serial_putchar(*ctrl);
      continue;
    }

    /* initialize all the flags for this format.   */
    dot_flag   = long_flag = par.left_flag = par.do_padding = 0;
    par.pad_character = ' ';
    par.num2=32767;

try_next:
    ch = *(++ctrl);

    if (isdigit((int)ch)) {
      if (dot_flag)
        par.num2 = getnum(&ctrl);
      else {
        if (ch == '0')
          par.pad_character = '0';

        par.num1 = getnum(&ctrl);
        par.do_padding = 1;
      }
      ctrl--;
      goto try_next;
    }

    switch (tolower((int)ch)) {
      case '%':
        serial_putchar( '%');
        continue;

      case '-':
        par.left_flag = 1;
        break;

      case '.':
        dot_flag = 1;
        break;

      case 'l':
        long_flag = 1;
        break;

      case 'd':
        if (long_flag || ch == 'D') {
          serial_outnum( va_arg(argp, long), 10L, &par);
          continue;
        }
        else {
          serial_outnum( va_arg(argp, int), 10L, &par);
          continue;
        }
      case 'u':
        if (long_flag || ch == 'D') {
          serial_outnum( va_arg(argp, unsigned long), 10L, &par);
          continue;
        }
        else {
          serial_outnum( va_arg(argp, unsigned), 10L, &par);
          continue;
        }
      case 'x':
        serial_outnum((long)va_arg(argp, int), 16L, &par);
        continue;

      case 'p':
        serial_putchar('0');
        serial_putchar('x');
        serial_outnum( va_arg(argp, unsigned long), 16L, &par);
        continue;

      case 's':
        serial_outs( va_arg( argp, char *), &par);
        continue;

      case 'c':
        serial_putchar( va_arg( argp, int));
        continue;

      case '\\':
        switch (*ctrl) {
          case 'a':
            serial_putchar( 0x07);
            break;
          case 'h':
            serial_putchar( 0x08);
            break;
          case 'r':
            serial_putchar( 0x0D);
            break;
          case 'n':
            serial_putchar( 0x0D);
            serial_putchar( 0x0A);
            break;
          default:
            serial_putchar( *ctrl);
            break;
        }
        ctrl++;
        break;
      default:
        continue;
    }
    goto try_next;
  }
  va_end( argp);
}
