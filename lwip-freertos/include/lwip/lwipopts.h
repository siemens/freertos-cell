/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
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
#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__
#include "FreeRTOS.h"

#define sys_msleep(ms) vTaskDelay(pdMS_TO_TICKS(ms))

#define PPP_SUPPORT 1
/* Enable PPP over serial */
#define PPPOS_SUPPORT PPP_SUPPORT
/* Disable PPP over ethernet */
#define PPPOE_SUPPORT 0
#define PAP_SUPPORT 0
#define CHAP_SUPPORT 0

#define LWIP_DEBUG
#undef LWIP_DEBUG
#ifdef LWIP_DEBUG
#define X32_F "x"
#define X16_F "x"
#define S32_F "d"
#define S16_F "d"
// Switch on debugging of subsystems
//#define DHCP_DEBUG LWIP_DBG_ON
//#define NETIF_DEBUG                     LWIP_DBG_ON
//#define ETHARP_DEBUG                    LWIP_DBG_ON
//#define UDP_DEBUG                    LWIP_DBG_ON
//#define TCP_DEBUG LWIP_DBG_ON
//#define TCP_INPUT_DEBUG LWIP_DBG_ON
//#define IP_DEBUG LWIP_DBG_ON
//#define UDP_DEBUG LWIP_DBG_ON
//#define PPP_DEBUG LWIP_DBG_ON
#endif

#define LWIP_ERR_T int
#define ETH_PAD_SIZE 2 //!!
// Align everything to 8 byte addresses
#define MEM_ALIGNMENT           (2U * portBYTE_ALIGNMENT) // This is necessary if we use the hardware accelerated tcpip checksum calculation

#ifndef MSS_MAX
#define MSS_MAX 1460U
#endif

#define MEM_SIZE (128<<10)
#define TCP_MSS                 ((MSS_MAX / MEM_ALIGNMENT) * MEM_ALIGNMENT)
#if 0 /* Over PPP this settings are not optimal */
#define TCP_WND                 ((0xffffU / TCP_MSS) * TCP_MSS)
#define TCP_SND_BUF             TCP_WND
#define TCP_QUEUE_OOSEQ         1
// FIXME Check if this setting is necessary
#define TCP_OVERSIZE                    0
#endif

//#define TCP_SND_QUEUELEN                (4 * (TCP_SND_BUF)/(TCP_MSS))
#define MEMP_NUM_TCP_PCB                10

// Do not use this feature if the whole stacks runs in zero-copy mode
#define LWIP_NETIF_TX_SINGLE_PBUF 0

//#define MEMP_NUM_SYS_TIMEOUT    8

// FIXME The recommendation is that MEMP_NUM_TCP_SEG is at least TCP_SND_QUEUELEN
// At the moment this value is smaller. 
// But the throughput is better. 
// The internet says that we should set it to ((TCP_WND + TCP_SND_BUF) / TCP_MSS )
// http://old.nabble.com/MEMP_NUM_TCP_SEG-setting-td12413760.html
#define MEMP_NUM_TCP_SEG        ((4 * TCP_SND_QUEUELEN) / 4)
#define MEMP_NUM_TCPIP_MSG_INPKT        128
#define MEMP_NUM_NETBUF                 16
#define MEMP_NUM_NETCONN                16
#define MEMP_NUM_PBUF           (MEMP_NUM_TCP_SEG + 32) // tcp_write without TCP_WRITE_FLAG_COPY
#define PBUF_POOL_SIZE          ( 2 * TCP_SND_BUF / TCP_MSS + MEMP_NUM_TCPIP_MSG_INPKT)
/**
 * PBUF_POOL_BUFSIZE: the size of each pbuf in the pbuf pool. The default is
 * designed to accomodate single full size TCP frame in one pbuf, including
 * TCP_MSS, IP header, and link header.
 */
#define PBUF_POOL_BUFSIZE       LWIP_MEM_ALIGN_SIZE(MSS_MAX+40+PBUF_LINK_HLEN)

// Speedup arp cache lookup
#define LWIP_NETIF_HWADDRHINT           1
// We use hardware checksum calculation
#define LWIP_CHECKSUM_ON_COPY 0

#define LWIP_STATS 0
#define LWIP_STATS_LARGE 1
#if LWIP_STATS
#define LWIP_STATS_DISPLAY LWIP_STATS
#endif
#define LWIP_NETCONN                    1
#define LWIP_SOCKET                     0
#define LWIP_RAW                        0

//#define LWIP_NOASSERT 1 // To suppress some errors for now (no debug output)
#define SYS_LIGHTWEIGHT_PROT            1
#define LWIP_ALLOW_MEM_FREE_FROM_OTHER_CONTEXT 0
#define LWIP_SO_RCVTIMEO 0 // Needed to interrupt a call to netconn_recv

#ifndef LWIP_DHCP
#define LWIP_DHCP               0
#endif

#define DHCP_DOES_ARP_CHECK     LWIP_DHCP

#define CHECKSUM_GEN_UDP                1
#define CHECKSUM_CHECK_UDP              CHECKSUM_GEN_UDP

//#define SO_REUSE                        0
#define TCP_LISTEN_BACKLOG          0
#define TCPIP_THREAD_STACKSIZE          (4<<10)
#define TCPIP_THREAD_PRIO               (tskIDLE_PRIORITY+2)
#define TCPIP_MBOX_SIZE         (4<<10)
#define DEFAULT_THREAD_STACKSIZE            configMINIMAL_STACK_SIZE
#define DEFAULT_THREAD_PRIO                 TCPIP_THREAD_PRIO
#define DEFAULT_RAW_RECVMBOX_SIZE     (4<<10)
#define DEFAULT_UDP_RECVMBOX_SIZE     (4<<10)
#define DEFAULT_TCP_RECVMBOX_SIZE     (4<<10)
#define DEFAULT_ACCEPTMBOX_SIZE       (2<<10)

//#define LWIP_TCPIP_CORE_LOCKING         1

#if 0 /* XXX */
//#define LWIP_NOASSERT 1 // To suppress some errors for now (no debug output)
//#define SYS_LIGHTWEIGHT_PROT            1

//#define TCPIP_THREAD_PRIO	(configMAX_PRIORITIES - 1)

#define ETH_PAD_SIZE 2 //!!

/* ---------- Memory options ---------- */
/* MEM_ALIGNMENT: should be set to the alignment of the CPU for which
   lwIP is compiled. 4 byte alignment -> define MEM_ALIGNMENT to 4, 2
   byte alignment -> define MEM_ALIGNMENT to 2. */
#define MEM_ALIGNMENT           4 //!!

/* MEM_SIZE: the size of the heap memory. If the application will send
a lot of data that needs to be copied, this should be set high. */
//++#define MEM_SIZE                (8<<10)

/* MEMP_NUM_PBUF: the number of memp struct pbufs. If the application
   sends a lot of data out of ROM (or other static memory), this
   should be set high. */
//++#define MEMP_NUM_PBUF           32
/* MEMP_NUM_UDP_PCB: the number of UDP protocol control blocks. One
   per active UDP "connection". */
//++#define MEMP_NUM_UDP_PCB        4
/* MEMP_NUM_TCP_PCB: the number of simulatenously active TCP
   connections. */
//++#define MEMP_NUM_TCP_PCB        10
/* MEMP_NUM_TCP_PCB_LISTEN: the number of listening TCP
   connections. */
//++#define MEMP_NUM_TCP_PCB_LISTEN 8
/* MEMP_NUM_TCP_SEG: the number of simultaneously queued TCP
   segments. */
//++#define MEMP_NUM_TCP_SEG        16
/* MEMP_NUM_SYS_TIMEOUT: the number of simulateously active
   timeouts. */
//++#define MEMP_NUM_SYS_TIMEOUT    8


/* The following four are used only with the sequential API and can be
   set to 0 if the application only will use the raw API. */
/* MEMP_NUM_NETBUF: the number of struct netbufs. */
//#define MEMP_NUM_NETBUF         4
/* MEMP_NUM_NETCONN: the number of struct netconns. */
//#define MEMP_NUM_NETCONN        4
/* MEMP_NUM_APIMSG: the number of struct api_msg, used for
   communication between the TCP/IP stack and the sequential
   programs. */
//#define MEMP_NUM_API_MSG        8
/* MEMP_NUM_TCPIPMSG: the number of struct tcpip_msg, which is used
   for sequential API communication and incoming packets. Used in
   src/api/tcpip.c. */
//#define MEMP_NUM_TCPIP_MSG      8

/* These two control is reclaimer functions should be compiled
   in. Should always be turned on (1). */
//++#define MEM_RECLAIM             1
//++#define MEMP_RECLAIM            1

/* ---------- Pbuf options ---------- */
/* PBUF_POOL_SIZE: the number of buffers in the pbuf pool. */
//++#define PBUF_POOL_SIZE          16

/**
 * PBUF_POOL_BUFSIZE: the size of each pbuf in the pbuf pool. The default is
 * designed to accomodate single full size TCP frame in one pbuf, including
 * TCP_MSS, IP header, and link header.
 */
//++#define PBUF_POOL_BUFSIZE               LWIP_MEM_ALIGN_SIZE(TCP_MSS+40+PBUF_LINK_HLEN)

/* PBUF_LINK_HLEN: the number of bytes that should be allocated for a
   link level header. */
//++#define PBUF_LINK_HLEN          14

/* ---------- TCP options ---------- */
//++#define LWIP_TCP                1
//++#define TCP_TTL                 255

/* Controls if TCP should queue segments that arrive out of
   order. Define to 0 if your device is low on memory. */
//++#define TCP_QUEUE_OOSEQ         1

/* TCP Maximum segment size. */
/* This parameter is very important for the network throughput */
#define TCP_MSS                 1460

/* TCP sender buffer space (bytes). */
/* This parameter is very important for the network throughput */
#define TCP_SND_BUF             (8*TCP_MSS)

/* TCP sender buffer space (pbufs). This must be at least = 2 *
   TCP_SND_BUF/TCP_MSS for things to work. */
//++#define TCP_SND_QUEUELEN        (4 * TCP_SND_BUF/TCP_MSS)

/* TCP receive window. */
//++#define TCP_WND                 (4*TCP_MSS)

/* Maximum number of retransmissions of data segments. */
//++#define TCP_MAXRTX              12

/* Maximum number of retransmissions of SYN segments. */
//++#define TCP_SYNMAXRTX           6

/* ---------- ARP options ---------- */
//++#define ARP_TABLE_SIZE 10
//++#define ARP_QUEUEING 1

/* ---------- IP options ---------- */
/* Define IP_FORWARD to 1 if you wish to have the ability to forward
   IP packets across network interfaces. If you are going to run lwIP
   on a device with only one network interface, define this to 0. */
//++#define IP_FORWARD              0

/* If defined to 1, IP options are allowed (but not parsed). If
   defined to 0, all packets with IP options are dropped. */
//++#define IP_OPTIONS              1

/** IP reassembly and segmentation. Even if they both deal with IP
 *  fragments, note that these are orthogonal, one dealing with incoming
 *  packets, the other with outgoing packets
 */

/** Reassemble incoming fragmented IP packets */
//++#define IP_REASSEMBLY                   1

/** Fragment outgoing IP packets if their size exceeds MTU */
//++#define IP_FRAG                         1

/* IP reassemly default age in seconds */
//++#define IP_REASS_MAXAGE 								3


/* ---------- ICMP options ---------- */
//++#define ICMP_TTL                255


/* ---------- DHCP options ---------- */
/* Define LWIP_DHCP to 1 if you want DHCP configuration of
   interfaces. DHCP is not implemented in lwIP 0.5.1, however, so
   turning this on does currently not work. */
//++#define LWIP_DHCP               0

/* 1 if you want to do an ARP check on the offered address
   (recommended). */
//++#define DHCP_DOES_ARP_CHECK     0

/* ---------- UDP options ---------- */
//++#define LWIP_UDP                1
//++#define UDP_TTL                 255


/* ---------- Statistics options ---------- */
//++#define STATS
//++#undef STATS

//++#ifdef STATS
//++#define LINK_STATS 1
//++#define IP_STATS   1
//++#define ICMP_STATS 1
//++#define UDP_STATS  1
//++#define TCP_STATS  1
//++#define MEM_STATS  1
//++#define MEMP_STATS 1
//++#define PBUF_STATS 1
//++#define SYS_STATS  1
//++#endif /* STATS */

#define LWIP_ERR_T int

/**
 * LWIP_NETCONN==1: Enable Netconn API (require to use api_lib.c)
 */
//++#define LWIP_NETCONN                    1
/**
 * LWIP_SOCKET==1: Enable Socket API (require to use sockets.c)
 */
//++#define LWIP_SOCKET                     0
/**
 * LWIP_STATS==1: Enable statistics collection in lwip_stats.
 */
//++#define LWIP_STATS                      0

#endif /* XXX */

#define LWIP_ETHADDR0    0x00  /**< The first octet of the Ethernet
				 address if LWIP_FIXEDETHADDR is
				 1. \hideinitializer */
#define LWIP_ETHADDR1    0xbd  /**< The second octet of the Ethernet
				 address if LWIP_FIXEDETHADDR is
				 1. \hideinitializer */
#define LWIP_ETHADDR2    0x3b  /**< The third octet of the Ethernet
				 address if LWIP_FIXEDETHADDR is
				 1. \hideinitializer */
#define LWIP_ETHADDR3    0x33  /**< The fourth octet of the Ethernet
				 address if LWIP_FIXEDETHADDR is
				 1. \hideinitializer */
#define LWIP_ETHADDR4    0x06  /**< The fifth octet of the Ethernet
				 address if LWIP_FIXEDETHADDR is
				 1. \hideinitializer */
#define LWIP_ETHADDR5    0x65  /**< The sixth octet of the Ethernet
				 address if UIP_FIXEDETHADDR is
				 1. \hideinitializer */

#undef LWIP_NO_CTYPE_H
#define LWIP_NO_CTYPE_H 1
#endif /* __LWIPOPTS_H__ */
