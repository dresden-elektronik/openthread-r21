/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#ifndef _SAMR21_NVM_H_
#define _SAMR21_NVM_H_

#include "samr21.h"
#include "samr21At86rf233.h"
#include <stdint.h>
#include <string.h>

#define SAMR21_NVM_NUM_PAGES              4096
#define SAMR21_NVM_SIZE_PAGE              64
#define SAMR21_NVM_PAGES_PER_ROW          4
#define SAMR21_NVM_SIZE_ROW               (SAMR21_NVM_PAGES_PER_ROW * SAMR21_NVM_SIZE_PAGE)

#define SAMR21_NVM_USER_ROW_ADDR          0x00804000
#define SAMR21_NVM_USER_ROW_IEEE_ADDR     (SAMR21_NVM_USER_ROW_ADDR + 0x0000000A)
#define SAMR21_NVM_USER_ROW_USB_I_SERIAL  (SAMR21_NVM_USER_ROW_ADDR + 0x00000012)


/**
 * Inits the NVM-Controller. Enables Manual Write and sets appropiate Wait States
 * See SAMR21 datasheet 20. NVMCTRL – Non-Volatile Memory Controller
 */
void samr21NvmInit();

/**
 * Reads out the Unique IEEE 64Bit Extended Address from NVMEM to a given buffer
 * 
 * @param[out] ieeeAddr     pointer to Buffer where the address is written to
 */
void samr21NvmGetIeeeAddr(uint8_t * ieeeAddr);


/**
 * Reads a given NV-Memory Region for a specified Length and copys the data to a given Buffer
 * 
 * @param[in]  addr      address when the NVMEM Read start
 * @param[in]  len       length of the data to be read
 * @param[out] buffer    Pointer to Buffer where data is written to
 * 
 * 
 */
void samr21NvmRead(uint32_t addr, uint8_t* buffer_p, uint32_t len);


/**
 * Writes to a given location in NV-Memory
 * 
 * @param[in]  addr      address when the NVMEM Write takes place
 * @param[in]  len       length of the data to be written
 * @param[out] buffer    Pointer to data to be written into NVMEM
 * 
 * 
 */
void samr21NvmWriteWithinRow(uint32_t addr, uint8_t* data_p, uint32_t len);

/**
 * Erases the row a given location is part of
 * 
 * @param[in]  addr      address within the row to be erased
 * 
 */
void samr21NvmEraseRow(uint32_t addr);

#endif //_SAMR21_PM_H_