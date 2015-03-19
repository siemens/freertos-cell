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
 * Author: Stefano Oliveri <stefano.oliveri@st.com>
 *
 */
#ifndef __CC_H__
#define __CC_H__

#include <stdio.h>
#include <stdint.h>
#include "cpu.h"
//#include "system.h"

//#define LWIP_PROVIDE_ERRNO 1
//#include "lwip_errno.h"

// Typedefs for the types used by lwip

typedef uint8_t    u8_t;
typedef int8_t    s8_t;
typedef uint16_t   u16_t;
typedef int16_t   s16_t;
typedef uint32_t    u32_t;
typedef int32_t    s32_t;
typedef uint32_t mem_ptr_t;
typedef int sys_prot_t;

#define __sio_fd_t_defined 1

// Compiler hints for packing lwip's structures

#define PACK_STRUCT_BEGIN
//#define PACK_STRUCT_BEGIN _Pragma("pack(2)")
#define PACK_STRUCT_STRUCT
#define PACK_STRUCT_END
//#define PACK_STRUCT_END _Pragma("pack()")
#define PACK_STRUCT_FIELD(x) x

// Platform specific diagnostic output

// non-fatal, print a message.
#if LWIP_DEBUG || LWIP_STATS
#define U16_F "u"
#define U32_F "lu"
#define LWIP_PLATFORM_DIAG( x, ... ) do{ printf x; } while( 0 )
#else
#define LWIP_PLATFORM_DIAG(x)
#endif

// fatal, print message and abandon execution.
#define LWIP_PLATFORM_ASSERT(x)
//#define LWIP_PLATFORM_ASSERT(x, ...) do{ printf(x);}while(0)

//#define LWIP_CHKSUM linux_do_csum
u16_t linux_do_csum(const unsigned char *buff, int len);
u16_t lwip_nios_custom_instruction_chksum(void *dataptr, int len);
#define LWIP_CHKSUM_ALGORITHM 3

#endif /* __CC_H__ */
