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

#define _DEBUG 1
#ifdef _DEBUG
#include <assert.h>
#endif

void samr21NvmInit()
{

    NVMCTRL->CTRLB.reg =
        NVMCTRL_CTRLB_RWS(NVMCTRL_CTRLB_RWS_HALF_Val) // Configure 2 Wait-States
        | NVMCTRL_CTRLB_MANW                          // Enable Manual Write
        | NVMCTRL_CTRLB_SLEEPPRM(NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val) | NVMCTRL_CTRLB_READMODE(NVMCTRL_CTRLB_READMODE_DETERMINISTIC_Val)
        //|NVMCTRL_CTRLB_CACHEDIS //enable vmw caching
        ;
}

void samr21NvmCtrlExecCommand(uint8_t a_cmd)
{

    NVMCTRL->CTRLA.reg =
        NVMCTRL_CTRLA_CMD(a_cmd) | NVMCTRL_CTRLA_CMDEX(NVMCTRL_CTRLA_CMDEX_KEY_Val);

    while (!NVMCTRL->INTFLAG.bit.READY)
    {
#ifdef _DEBUG
        assert(!NVMCTRL->INTFLAG.bit.ERROR);
        assert(!NVMCTRL->STATUS.bit.PROGE);
        assert(!NVMCTRL->STATUS.bit.NVME);
        assert(!NVMCTRL->STATUS.bit.LOCKE);
#endif
    }
}

uint64_t samr21NvmGetIeeeAddr()
{
    uint64_t res;

    samr21NvmRead(NVM_USER_ROW_IEEE_ADDR, &res, sizeof(uint32_t));

    return res;
}

void samr21NvmRead(uint32_t a_addr, uint8_t *a_buffer_p, uint32_t a_len)
{
    for (uint32_t i = 0; i < a_len; i++)
    {
        a_buffer_p[i] = ((uint8_t *)a_addr)[i];
    }
}

void samr21NvmWriteWithinRow(uint32_t a_addr, uint8_t *a_data_p, uint32_t a_len)
{

    uint32_t rowOffset = a_addr % (NVM_SIZE_PAGE * NVM_PAGES_PER_ROW);

#ifdef _DEBUG
    assert(rowOffset + a_len <= (NVM_SIZE_PAGE * NVM_PAGES_PER_ROW));
#endif

    uint32_t rowBaseAddr = a_addr - rowOffset;

    // Copy the Row which contains the addr to be written to
    // The whole row has to be erased in order to write to it
    uint32_t tempRowMemory_1D[NVM_PAGES_PER_ROW * (NVM_SIZE_PAGE / sizeof(uint32_t))];

    for (uint16_t i = 0; i < ( NVM_PAGES_PER_ROW * (NVM_SIZE_PAGE / sizeof(uint32_t ) ) ); i++)
    {
            tempRowMemory_1D[i] = *( ( uint32_t * )( rowBaseAddr + ( i * sizeof(uint32_t) ) ) );
    }

    // Erase the row containing the data to be written
    NVMCTRL->ADDR.reg = rowBaseAddr >> 1;
    samr21NvmCtrlExecCommand(NVMCTRL_CTRLA_CMD_ER_Val);

    // Modify the data in Question
    memcpy( ( ( uint8_t * )( tempRowMemory_1D ) + rowOffset ), a_data_p, a_len );

    // Write back the modified row page by page
    for (uint16_t i = 0; i < NVM_PAGES_PER_ROW; i++)
    {

        // Clear Page Cache
        samr21NvmCtrlExecCommand(NVMCTRL_CTRLA_CMD_PBC_Val);

        // Write changes to Page Cache (Addr is the normal memory mapped NVM-Addr)
        for ( uint16_t j = 0; j < ( NVM_SIZE_PAGE / sizeof(uint32_t ) ); j++ )
        {
            uint32_t *tempPtr = (uint32_t *)(rowBaseAddr + (i * NVM_SIZE_PAGE) + j * sizeof(uint32_t));
            uint32_t tempVal = tempRowMemory_1D[ ( i * ( NVM_SIZE_PAGE / sizeof(uint32_t) ) ) + j ];
        
            *tempPtr = tempVal;
        }
        // Write to Page Cache to Flash
        samr21NvmCtrlExecCommand(NVMCTRL_CTRLA_CMD_WP_Val);
    }
}