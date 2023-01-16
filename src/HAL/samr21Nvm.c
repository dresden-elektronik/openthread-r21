/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#include "samr21Nvm.h"

void samr21NvmInit(){
    NVMCTRL->CTRLB.bit.RWS = 1;
}

uint64_t samr21NvmGetIeeeAddr()
{
    uint64_t res;

    for (uint8_t i = 0; i < sizeof(uint64_t); i++)
    {
        ((uint8_t *)(&res))[0] = ((uint8_t *)NVM_USER_ROW_IEEE_ADDR)[0];
    }
    
    return res;
}

