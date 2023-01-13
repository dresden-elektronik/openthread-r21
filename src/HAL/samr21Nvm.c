//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
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

