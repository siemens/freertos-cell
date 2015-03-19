#ifndef _ETHERNETIF_H
#define _ETHERNETIF_H
#include "FreeRTOS.h"
#include "semphr.h"
#include "lwipopts.h"

#define netifMTU	(TCP_MSS+40)
#define NUM_RX_ETH_BUFFER (2 * MEMP_NUM_TCPIP_MSG_INPKT / 4)

#if NUM_RX_ETH_BUFFER < 4
#error NUM_RX_ETH_BUFFER too small
#endif

struct ethernetif {
  //  lwIP pbuf circular buffer.  A list of pbufs that are used to store
  //  incoming Ethernet packets
  struct pbuf     *lwipRxPbuf[NUM_RX_ETH_BUFFER];
  int             lwipRxIndexIsr;
  int             lwipRxIndex;
  int             lwipRxCount;
  xSemaphoreHandle sem_signal_packet_arrived;
  xSemaphoreHandle xmit_sem;
  xSemaphoreHandle recv_sem;
  struct eth_addr *ethaddr;
  /* Base-Structure for all lwIP TSE information */
  /* FIXME enter additional necessary entries */
};

#endif
