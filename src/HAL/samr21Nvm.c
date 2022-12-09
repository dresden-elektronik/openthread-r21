//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21Nvm.h"

void samr21NvmInit(){
    NVMCTRL->CTRLB.bit.RWS = 1;
}

uint64_t samr21NvmGetIeeeAddr()
{
    uint64_t res;

    ((uint32_t *)(&res))[0] = ((uint32_t *) NVM_USER_ROW_IEEE_ADDR)[0];
    ((uint32_t *)(&res))[1] = ((uint32_t *) NVM_USER_ROW_IEEE_ADDR)[0];

    return res;
}

