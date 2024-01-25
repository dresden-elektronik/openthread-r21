

/*
 *  Copyright (c) 2017, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file includes samr21 compile-time configuration constants
 *   for OpenThread.
 */

#ifndef OPENTHREAD_CORE_SAMR21_CONFIG_H_
#define OPENTHREAD_CORE_SAMR21_CONFIG_H_

#include <stdint.h>

extern uint32_t __d_nv_mem_start;
extern uint32_t __d_nv_mem_end;

#ifndef OPENTHREAD_CONFIG_THREAD_VERSION
#define OPENTHREAD_CONFIG_THREAD_VERSION OT_THREAD_VERSION_1_3
#endif

/**
 * @def OPENTHREAD_CONFIG_PLATFORM_INFO
 *
 * The platform-specific string to insert into the OpenThread version string.
 *
 */
#define OPENTHREAD_CONFIG_PLATFORM_INFO "CONBEE2_RASPBEE2"

/**
 * @def OPENTHREAD_CONFIG_PLATFORM_FLASH_API_ENABLE
 *
 * Define to 1 to enable otPlatFlash* APIs to support non-volatile storage.
 *
 * When defined to 1, the platform MUST implement the otPlatFlash* APIs instead of the otPlatSettings* APIs.
 *
 */
#define OPENTHREAD_CONFIG_PLATFORM_FLASH_API_ENABLE 1

/**
 * @def OPENTHREAD_CONFIG_MAC_SOFTWARE_TX_SECURITY_ENABLE
 *
 * Define to 1 to enable software transmission security logic.
 *
 */
#define OPENTHREAD_CONFIG_MAC_SOFTWARE_TX_SECURITY_ENABLE 1


/**
 * @def RADIO_CONFIG_SRC_MATCH_ENTRY_NUM
 *
 * The number of source address table entries.
 *
 */
#define RADIO_CONFIG_SRC_MATCH_ENTRY_NUM 16
#define RADIO_CONFIG_SRC_MATCH_EXT_ENTRY_NUM 16

/**
 * @def OPENTHREAD_CONFIG_DEFAULT_TRANSMIT_POWER
 *
 * The default IEEE 802.15.4 transmit power (dBm)
 *
 */
#define OPENTHREAD_CONFIG_DEFAULT_TRANSMIT_POWER 5

#define OPENTHREAD_CONFIG_DUA_ENABLE                1
/**
 * @def OPENTHREAD_CONFIG_PLATFORM_USEC_TIMER_ENABLE
 *
 * Define to 1 if you want to support microsecond timer in platform.
 *
 */
#define OPENTHREAD_CONFIG_PLATFORM_USEC_TIMER_ENABLE 1

// <q> CSL Auto Synchronization using data polling
#define OPENTHREAD_CONFIG_MAC_CSL_AUTO_SYNC_ENABLE  0

// <q>  CSL (Coordinated Sampled Listening) Debug
#define OPENTHREAD_CONFIG_MAC_CSL_DEBUG_ENABLE      0

// <q>  CSL (Coordinated Sampled Listening) Receiver
#define OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE   0

#define OPENTHREAD_CONFIG_DNS_CLIENT_ENABLE         1

#define OPENTHREAD_CONFIG_MLR_ENABLE                0

#define OPENTHREAD_CONFIG_MLE_LINK_METRICS_INITIATOR_ENABLE 0

#define OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE 0

#ifndef OPENTHREAD_CONFIG_PING_SENDER_ENABLE
#define OPENTHREAD_CONFIG_PING_SENDER_ENABLE        1
#endif

// <q>  Service Registration Protocol (SRP) Client (Thread 1.3)
#define OPENTHREAD_CONFIG_SRP_CLIENT_ENABLE         1


/**
 * @def OPENTHREAD_CONFIG_TCP_ENABLE
 *
 * Define as 1 to enable TCP.
 *
 */
#define OPENTHREAD_CONFIG_TCP_ENABLE 0

/**
 * @def OPENTHREAD_CONFIG_STACK_VENDOR_OUI
 *
 * The Organizationally Unique Identifier for the vendor.
 *
 */
#ifndef OPENTHREAD_CONFIG_STACK_VENDOR_OUI
#define OPENTHREAD_CONFIG_STACK_VENDOR_OUI 0x00212e
#endif

#endif // OPENTHREAD_CORE_SAMR21_CONFIG_H_