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

#ifndef _SAMR21_NVM_H_
#define _SAMR21_NVM_H_

#define NVM_USER_ROW_ADDR          0x00804000
#define NVM_USER_ROW_IEEE_ADDR     (NVM_USER_ROW_ADDR + 0x0000000A)

void samr21NvmInit();

uint64_t samr21NvmGetIeeeAddr();

#endif //_SAMR21_PM_H_