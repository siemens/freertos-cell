/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/*
 * This file is a skeleton for developing Ethernet network interface
 * drivers for lwIP. Add code to the low_level functions and do a
 * search-and-replace for the word "ethernetif" to replace it with
 * something that better describes your network interface.
 */

#include <stdio.h>
#include <string.h>
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include <lwip/stats.h>
#include <lwip/timers.h>

#include "sys/alt_cache.h"

#include "netif/etharp.h"
#include "tse_emac.h"
//#include "../../memcpy_dma.h"
#include "../../hexdump.h"

#include "ethernetif.h"

#define netifDO_NOT_LOCK_ACCESS 0
#define netifINTERFACE_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE )
#define netifINTERFACE_TASK_PRIORITY		( configMAX_PRIORITIES-1 )
#define netifGUARD_BLOCK_TIME			portMAX_DELAY
#define IFNAME0 'e'
#define IFNAME1 'm'

/* The time to block waiting for input. */
#define emacBLOCK_TIME_WAITING_FOR_INPUT	portMAX_DELAY

/* Forward declarations. */
static err_t ethernetif_output(struct netif *netif, struct pbuf *p, struct ip_addr *ipaddr);
static void task_ethernetif_input( void * pvParameters );

static void low_level_init(struct netif *netif)
{
  struct ethernetif *ei = netif->state;

  //memcpy_dma_init("/dev/dma_0");
  if( ei->xmit_sem == NULL )
      vSemaphoreCreateBinary( ei->xmit_sem );
  if( ei->recv_sem == NULL )
    vSemaphoreCreateBinary( ei->recv_sem );

  /* set MAC hardware address length */
  netif->hwaddr_len = ETHARP_HWADDR_LEN;

  /* maximum transfer unit */
  netif->mtu = netifMTU;

  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

  /* Do whatever else is needed to initialize interface. */
  /* Initialise the MAC. */
  printf("%s: %02x:%02x:%02x:%02x:%02x:%02x\n", __func__, 
  netif->hwaddr[0], netif->hwaddr[1], netif->hwaddr[2], netif->hwaddr[3], netif->hwaddr[4], netif->hwaddr[5]);
  if(0 > tse_emac_init(ei, netif->hwaddr))
    printf("%s: tse_emac_init() failed\n", __func__);
	
  /* set MAC hardware address */
  tse_emac_get_mac_address(ei->trans_info.base, netif->hwaddr);

  /* Create the task that handles the EMAC. */
  xTaskCreate( task_ethernetif_input, ( signed char * ) "ethinp", netifINTERFACE_TASK_STACK_SIZE, netif, netifINTERFACE_TASK_PRIORITY, NULL );
  tse_emac_start_operation(ei);
}	


/*
 * low_level_output():
 *
 * Should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 */

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  struct ethernetif *ei = netif->state;
#if ETH_PAD_SIZE_TSE
  pbuf_header(p, -ETH_PAD_SIZE);			/* drop the padding word */
#endif

  /* Access to the EMAC is guarded using a semaphore. */
  if( netifDO_NOT_LOCK_ACCESS || xSemaphoreTake( ei->xmit_sem, netifGUARD_BLOCK_TIME ) )
  {
    tse_emac_send_chain(netif, p);

      #if ETH_PAD_SIZE_TSE
        pbuf_header(p, ETH_PAD_SIZE);			/* reclaim the padding word */
      #endif

      LINK_STATS_INC(link.xmit);

      xSemaphoreGive( ei->xmit_sem );
  }

  return ERR_OK;
}

/*
 * low_level_input():
 *
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 */

static struct pbuf *low_level_input(struct netif *netif)
{
  struct ethernetif *ei = netif->state;
  struct pbuf *p = NULL;

  /* Access to the emac is guarded using a semaphore. */
  if( netifDO_NOT_LOCK_ACCESS || xSemaphoreTake( ei->recv_sem, netifGUARD_BLOCK_TIME ) )
  {
    if(ei->lwipRxCount) {
      //  Dump current packet if there is no memory for the next packet.
      //  The lwipRxPbuf must contain pointers to pbufs at all times.
      struct pbuf *nextPkt = pbuf_alloc(PBUF_RAW,  PBUF_POOL_BUFSIZE, PBUF_POOL);

      if(nextPkt) {
        if(1 != pbuf_clen(nextPkt)) {
          pbuf_free(nextPkt);
          printf("%s-ERROR: Splitted pbuf created\n", __func__);
          return NULL;
        }

        // Invalidate cache before giving it to the DMA engine
        //if(ALT_CPU_DCACHE_SIZE) {
        //  //nextPkt = (void *) alt_remap_uncached(nextPkt,sizeof *nextPkt);
        //  nextPkt->payload = (void *) alt_remap_uncached(nextPkt->payload, PBUF_POOL_BUFSIZE);
        //  //alt_dcache_flush(nextPkt->payload, nextPkt->len);
        //}

        portENTER_CRITICAL();

        // Hand over the packet to the upper layer
        --ei->lwipRxCount;
        p = ei->lwipRxPbuf[ei->lwipRxIndex];
        ei->lwipRxPbuf[ei->lwipRxIndex] = nextPkt;
        if(NUM_RX_ETH_BUFFER <= ++ei->lwipRxIndex)
          ei->lwipRxIndex = 0;

        portEXIT_CRITICAL();

        LWIP_ASSERT("low_level_input: pbuf in rx buffer is NULL", p != NULL );
        LWIP_ASSERT("low_level_input: pbuf->len in rx buffer is 0", p->len != 0 );

        // Length is zero. Should not happen
        if(!p->len) {
          pbuf_free(p);
          p = NULL;
          printf("%s-CRITICAL: zero length packet received\n", __func__);
        }
      }
      else {
        LINK_STATS_INC(link.memerr);
        LINK_STATS_INC(link.drop);
      }
    }

    xSemaphoreGive( ei->recv_sem );
  }

  return p;
}

/*
 * ethernetif_output():
 *
 * This function is called by the TCP/IP stack when an IP packet
 * should be sent. It calls the function called low_level_output() to
 * do the actual transmission of the packet.
 *
 */

static err_t ethernetif_output(struct netif *netif, struct pbuf *p,  struct ip_addr *ipaddr)
{

 /* resolve hardware address, then send (or queue) packet */
  return etharp_output(netif, p, ipaddr);

}

/*
 * task_ethernetif_input():
 *
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface.
 *
 */

static void task_ethernetif_input( void * pvParameters )
{
  struct netif *netif = pvParameters;
  struct ethernetif *ei = netif->state;
  struct eth_hdr *ethhdr;
  struct pbuf *p = NULL;
  err_t err;

  while(1) {
    //printf("%s: %u\n", __func__, uxQueueMessagesWaiting(ei->sem_signal_packet_arrived));
    if(pdTRUE == xSemaphoreTake( ei->sem_signal_packet_arrived, emacBLOCK_TIME_WAITING_FOR_INPUT)
        && NULL != (p = low_level_input( netif ))) {
      //putchar('<');
      /* At least one packet arrived: This is signaled by an interrupt */
      //printf("%s: len=%d tot_len=%d\n", __func__, p->len, p->tot_len);
      /* points to packet payload, which starts with an Ethernet header */
      ethhdr = p->payload;

      switch (ethhdr->type) {
        /* IP or ARP packet? */
        case PP_HTONS(ETHTYPE_ARP):
        case PP_HTONS(ETHTYPE_IP):
#if PPPOE_SUPPORT
          /* PPPoE packet? */
        case PP_HTONS(ETHTYPE_PPPOEDISC):
        case PP_HTONS(ETHTYPE_PPPOE):
#endif /* PPPOE_SUPPORT */
          /* full packet send to tcpip_thread to process */
          if(ERR_OK != (err = netif->input(p, netif))) {
            LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
            printf("%s-CRITICAL: netif->input failed: %d\n", __func__, err);
            LINK_STATS_INC(link.drop);
            if(ERR_MEM == err)
              LINK_STATS_INC(link.memerr);
            pbuf_free(p);
            p = NULL;
          }
          break;
        default:
          pbuf_free(p);
          p = NULL;
          break;
      }
    }
  }
}

/*
 * ethernetif_init():
 *
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 */

err_t
ethernetif_init(struct netif *netif)
{
  int idx;
  struct ethernetif *ethernetif;

  ethernetif = mem_malloc(sizeof(struct ethernetif));

  if (ethernetif == NULL)
  {
  	LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_init: out of memory\n"));
  	return ERR_MEM;
  }

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
    netif->hostname = "asmart";
#endif /* LWIP_NETIF_HOSTNAME */

  /*
   * Initialize the snmp variables and counters inside the struct netif.
   * The last argument should be replaced with your link speed, in units
   * of bits per second.
   */
  NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

  netif->state = ethernetif;
  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  netif->output = ethernetif_output;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */
  netif->output = etharp_output;
  netif->linkoutput = low_level_output;

  ethernetif->ethaddr = (struct eth_addr *)&(netif->hwaddr[0]);

  /* initialize the rx buffer for access by the TSE ISR */
  for(idx = 0; idx < sizeof(ethernetif->lwipRxPbuf)/sizeof(ethernetif->lwipRxPbuf[0]); ++idx) {
    struct pbuf *p = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL);
    if (!p) {
      while(--idx >= 0)
        pbuf_free(ethernetif->lwipRxPbuf[idx]);
      return ERR_MEM;
    }
    if(1 < pbuf_clen(p)) {
      while(--idx >= 0)
        pbuf_free(ethernetif->lwipRxPbuf[idx]);
      pbuf_free(p);
      printf("%s-CRITICAL: pbuf contains more than one element\n", __func__);
      return ERR_BUF;
    }
    ethernetif->lwipRxPbuf[idx] = p;
  }
  printf("%s: Allocatated %d pbufs for input\n", __func__, idx);
  ethernetif->lwipRxCount = ethernetif->lwipRxIndex = ethernetif->lwipRxIndexIsr = 0;

  low_level_init(netif);
  return ERR_OK;
}
