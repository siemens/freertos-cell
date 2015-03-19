#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "echod.h"
#include "../../hexdump.h"

#include <string.h>

static void close_conn(struct tcp_pcb *pcb)
{
  tcp_arg(pcb, NULL);
  tcp_sent(pcb, NULL);
  tcp_recv(pcb, NULL);
  tcp_close(pcb);
}

err_t echo_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
  return ERR_OK;
}

static err_t echo_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
  if (err == ERR_OK && p != NULL) {
    /* Inform TCP that we have taken the data. */
    tcp_recved(pcb, p->tot_len);  
    //Send out the data
    err = tcp_write(pcb, p->payload, p->len, TCP_WRITE_FLAG_COPY);
  }
  //Free the packet buffer
  pbuf_free(p);

  if (err == ERR_OK && p == NULL) {
    close_conn(pcb);
  }
  return ERR_OK;
}

static err_t echo_poll(void *arg, struct tcp_pcb *pcb)
{
  //if(++retries == 40) {
  //  tcp_abort(pcb);
  //  return ERR_ABRT;
  //}
  return ERR_OK;
}

static err_t echo_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);
  tcp_setprio(pcb, TCP_PRIO_MIN);
  tcp_arg(pcb, NULL);
  tcp_recv(pcb, echo_recv);
  tcp_sent(pcb, echo_sent);
  tcp_err(pcb, NULL); //Don't care about error here
  tcp_poll(pcb, echo_poll, 4); //No polling here
  return ERR_OK;
}

void tcp_echod_init(void)
{
  struct tcp_pcb *pcb;

  pcb = tcp_new();
  tcp_bind(pcb, IP_ADDR_ANY, 32000);
  pcb = tcp_listen(pcb);
  tcp_accept(pcb, echo_accept);
}

#define MAX_UDP_MSG_SIZE (IP_FRAG_MAX_MTU-IP_HLEN-UDP_HLEN)

static void __attribute__((__unused__)) task_udp_blaster(void *arg)
{
  struct pbuf *p;
  static char msg[MAX_UDP_MSG_SIZE]="testing";
  struct udp_pcb *ptel_pcb = arg;
  struct ip_addr xIpAddr;
  portTickType period;

  IP4_ADDR( &xIpAddr, 255,255,255,255);
  udp_connect(ptel_pcb, &xIpAddr, 32001);
  period = xTaskGetTickCount();
  while(1) {
    p = pbuf_alloc(PBUF_TRANSPORT,MAX_UDP_MSG_SIZE,PBUF_RAM);
    if(p) {
      memcpy (p->payload, msg, sizeof(msg));
      //int cnt=0; *((unsigned*)p->payload) = ++cnt;
      //puts("task_udp_blaster FIXME");
      udp_send(ptel_pcb, p);
      pbuf_free(p);
    }
    vTaskDelayUntil(&period, 10*1000 / portTICK_RATE_MS);
    //portYIELD();
  }
}

void udp_echo_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, struct ip_addr *addr, u16_t port)
{
  struct ip_addr xIpAddr;

  IP4_ADDR( &xIpAddr, 172,123,0,1);
  if (p != NULL) {
    udp_sendto(pcb, p, &xIpAddr, 32000); //dest port
    pbuf_free(p);
  }
}

void echod_init(void)
{
  struct udp_pcb *pcb;

  tcp_echod_init();
  pcb = udp_new();
  udp_bind(pcb, IP_ADDR_ANY, 32000);
  udp_recv(pcb, udp_echo_recv, NULL);
  //xTaskCreate( task_udp_blaster, (const signed char*)"dhcp", configMINIMAL_STACK_SIZE, pcb, TCPIP_THREAD_PRIO, NULL );
}
/*-----------------------------------------------------------------------------------*/

