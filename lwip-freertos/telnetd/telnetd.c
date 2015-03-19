#include <string.h>
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "telnetd.h"
#include "memb.h"
#include "../../hexdump.h"

#define ISO_nl       0x0a
#define ISO_cr       0x0d

MEMB(linemem, TELNETD_LINELEN, TELNETD_NUMLINES);

#define STATE_NORMAL 0
#define STATE_IAC    1
#define STATE_WILL   2
#define STATE_WONT   3
#define STATE_DO     4  
#define STATE_DONT   5
#define STATE_CLOSE  6

#define TELNET_IAC   255
#define TELNET_WILL  251
#define TELNET_WONT  252
#define TELNET_DO    253
#define TELNET_DONT  254
/*-----------------------------------------------------------------------------------*/
static char *
_alloc_line(void)
{  
  return memb_alloc(&linemem);
}
/*-----------------------------------------------------------------------------------*/
static void
_dealloc_line(char *line)
{
  memb_free(&linemem, line);
}

static err_t telnetd_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);

static void
_senddata(struct tcp_pcb *pcb, struct telnetd_state *s)
{
  if(s->lines[0] != NULL) {
    if(ERR_OK == tcp_write(pcb, s->lines[0], strlen(s->lines[0]), TCP_WRITE_FLAG_COPY)) {
      telnetd_sent(s, pcb, 0);
    }
  }
}

/*-----------------------------------------------------------------------------------*/
static void
_sendline(struct telnetd_state *s, char *line)
{
  static unsigned int i;
  for(i = 0; i < TELNETD_NUMLINES; ++i) {
    if(s->lines[i] == NULL) {
      s->lines[i] = line;
      break;
    }
  }
  if(i == TELNETD_NUMLINES) {
    _dealloc_line(line);
  }
  _senddata(s->pcb, s);
}

/*-----------------------------------------------------------------------------------*/
/**
 * Print a prompt on a telnet connection.
 *
 * This function can be called by the telnet command shell in order to
 * print out a command prompt.
 *
 * \param s A telnet connection.
 *
 * \param str The command prompt.
 *
 */
/*-----------------------------------------------------------------------------------*/
void
telnetd_prompt(struct telnetd_state *s, char *str)
{
  char *line;
  line = _alloc_line();
  if(line != NULL) {
    strncpy(line, str, TELNETD_LINELEN);
    _sendline(s, line);
  }         
}
/*-----------------------------------------------------------------------------------*/
/**
 * Print out a string on a telnet connection.
 *
 * This function can be called from a telnet command parser in order
 * to print out a string of text on the connection. The two strings
 * given as arguments to the function will be concatenated, a carrige
 * return and a new line character will be added, and the line is
 * sent.
 *
 * \param s The telnet connection.
 *
 * \param str1 The first string.
 *
 * \param str2 The second string.
 *
 */
/*-----------------------------------------------------------------------------------*/
void
telnetd_output(struct telnetd_state *s, char *str1, char *str2)
{
  static unsigned len;
  char *line;
  
  line = _alloc_line();
  if(line != NULL) {
    len = strlen(str1);
    strncpy(line, str1, TELNETD_LINELEN);
    if(len < TELNETD_LINELEN) {
      strncpy(line + len, str2, TELNETD_LINELEN - len);
    }
    len = strlen(line);
    if(len < TELNETD_LINELEN - 2) {
      line[len] = ISO_cr;
      line[len+1] = ISO_nl;
      line[len+2] = 0;
    }
    _sendline(s, line);
  }
}

/*-----------------------------------------------------------------------------------*/
static err_t
telnetd_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  int i;
  struct telnetd_state *s = arg;
  _dealloc_line(s->lines[0]);
  for(i = 1; i < TELNETD_NUMLINES; ++i) {
    s->lines[i - 1] = s->lines[i];
  }
  _senddata(tpcb, s);
  return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/
static err_t telnetd_poll(void *arg, struct tcp_pcb *pcb)
{
  _senddata(pcb, arg);
  return ERR_OK;
}

static void
t_getchar(struct telnetd_state *s, u8_t c)
{
  if(c == ISO_cr) {
    return;
  }
  
  s->buf[(int)s->bufptr] = c;  
  if(s->buf[(int)s->bufptr] == ISO_nl ||
     s->bufptr == sizeof(s->buf) - 1) {    
    if(s->bufptr > 0) {
      s->buf[(int)s->bufptr] = 0;
    }
    telnetd_input(s, s->buf);
    s->bufptr = 0;
  } else {
    ++s->bufptr;
  }
}

static void
_sendopt(struct telnetd_state *s, u8_t option, u8_t value)
{
  char *line;
  line = _alloc_line();
  if(line != NULL) {
    line[0] = TELNET_IAC;
    line[1] = option;
    line[2] = value;
    line[3] = 0;
    _sendline(s, line);
  }       
}

static void close_conn(struct tcp_pcb *pcb, struct telnetd_state *s)
{
  tcp_arg(pcb, NULL);
  tcp_sent(pcb, NULL);
  tcp_recv(pcb, NULL);
  mem_free(s);
  tcp_close(pcb);
}

void telnetd_close(struct telnetd_state *s)
{
  close_conn(s->pcb, s);
}

/*-----------------------------------------------------------------------------------*/
static err_t telnetd_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
  struct telnetd_state *s = arg;
  u8_t c;

  if (err == ERR_OK && p != NULL) {
    char *pc = p->payload;
    int len = p->len;

    while(len > 0 && s->bufptr < sizeof(s->buf)) {
      c = *pc;
      ++pc;
      --len;
      switch(s->state) {
        case STATE_IAC:
          if(c == TELNET_IAC) {
            t_getchar(s, c);
            s->state = STATE_NORMAL;
          } else {
            switch(c) {
              case TELNET_WILL:
                s->state = STATE_WILL;
                break;
              case TELNET_WONT:
                s->state = STATE_WONT;
                break;
              case TELNET_DO:
                s->state = STATE_DO;
                break;
              case TELNET_DONT:
                s->state = STATE_DONT;
                break;
              default:
                s->state = STATE_NORMAL;
                break;
            }
          }
          break;
        case STATE_WILL:
          /* Reply with a DONT */
          _sendopt(s, TELNET_DONT, c);
          s->state = STATE_NORMAL;
          break;

        case STATE_WONT:
          /* Reply with a DONT */
          _sendopt(s, TELNET_DONT, c);
          s->state = STATE_NORMAL;
          break;
        case STATE_DO:
          /* Reply with a WONT */
          _sendopt(s, TELNET_WONT, c);
          s->state = STATE_NORMAL;
          break;
        case STATE_DONT:
          /* Reply with a WONT */
          _sendopt(s, TELNET_WONT, c);
          s->state = STATE_NORMAL;
          break;
        case STATE_NORMAL:
          if(c == TELNET_IAC) {
            s->state = STATE_IAC;
          } else {
            t_getchar(s, c);
          }      
          break;
      } 
    }  
  }
  //Free the packet buffer
  pbuf_free(p);
  if (err == ERR_OK && p == NULL) {
    close_conn(s->pcb, s);
  }
  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
static err_t telnetd_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
  int i;
  struct telnetd_state *s;
  LWIP_UNUSED_ARG(err);

  s = mem_malloc(sizeof(*s));
  if(!s) {
    return ERR_MEM;
  }
  for(i = 0; i < TELNETD_NUMLINES; ++i) {
    s->lines[i] = NULL;
  }
  s->bufptr = 0;
  s->state = STATE_NORMAL;
  s->pcb = pcb;

  tcp_setprio(pcb, TCP_PRIO_MIN);
  tcp_arg(pcb, s);
  tcp_recv(pcb, telnetd_recv);
  tcp_err(pcb, NULL); //Don't care about error here
  tcp_poll(pcb, telnetd_poll, 4); //No polling here
  return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/
/**
 * Initialize the telnet server.
 *
 * This function will perform the necessary initializations and start
 * listening on TCP port 23.
 */
/*-----------------------------------------------------------------------------------*/
void
telnetd_init(void)
{
  struct tcp_pcb *pcb;

  pcb = tcp_new();
  tcp_bind(pcb, IP_ADDR_ANY, 23);
  pcb = tcp_listen(pcb);
  tcp_accept(pcb, telnetd_accept);
}

/*-----------------------------------------------------------------------------------*/
