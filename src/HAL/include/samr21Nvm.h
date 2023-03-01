/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#include "samr21.h"
#include <stdint.h>
#include <string.h>

#ifndef _SAMR21_NVM_H_
#define _SAMR21_NVM_H_

#define NVM_NUM_PAGES              4096
#define NVM_SIZE_PAGE              64
#define NVM_PAGES_PER_ROW          4

#define NVM_USER_ROW_ADDR          0x00804000
#define NVM_USER_ROW_IEEE_ADDR     (NVM_USER_ROW_ADDR + 0x0000000A)
#define NVM_USER_ROW_USB_I_SERIAL  (NVM_USER_ROW_ADDR + 0x00000012)

void samr21NvmInit();

uint64_t samr21NvmGetIeeeAddr();
void samr21NvmRead(uint32_t addr, uint8_t* buffer_p, uint32_t len);
void samr21NvmWriteWithinRow(uint32_t addr, uint8_t* data_p, uint32_t len);

#endif //_SAMR21_PM_H_