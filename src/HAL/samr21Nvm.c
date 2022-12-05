//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21Nvm.h"

void samr21NvmInit(){
    NVMCTRL->CTRLB.bit.RWS = 1;
}

uint64_t samr21NvmGetIeeeAddr()
{
    return *( ( uint32_t * ) NVM_USER_ROW_IEEE_ADDR );
}

