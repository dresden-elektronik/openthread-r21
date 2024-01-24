// Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#include "openthread/platform/radio.h"
#include "samr21Nvm.h"

#ifdef _DEBUG
    #include <assert.h>
#endif


#ifdef _GCF_RELEASE_
    #define SAMR21_OT_NVM_ROWS 14
#else
    #define SAMR21_OT_NVM_ROWS 16
#endif

#define SAMR21_OT_SWAP_SIZE ((SAMR21_OT_NVM_ROWS * SAMR21_NVM_SIZE_ROW) / 2)


//From Linker Script
extern uint32_t _snvmem;


void otPlatFlashInit(otInstance *a_instance){
    OT_UNUSED_VARIABLE(a_instance);
}

uint32_t otPlatFlashGetSwapSize(otInstance *a_instance){
    OT_UNUSED_VARIABLE(a_instance);

    //Flash is Split into 2 Swap Areas
    return SAMR21_OT_SWAP_SIZE;
}


void otPlatFlashErase(otInstance *a_instance, uint8_t a_swapIndex){
    OT_UNUSED_VARIABLE(a_instance);

#ifdef _DEBUG
    assert(a_SwapIndex <= 1)
#endif

    uint32_t swapBaseAddress = &_snvmem;

    if(a_swapIndex){
        swapBaseAddress += SAMR21_OT_SWAP_SIZE;
    }

    for (uint8_t i = 0; i < (SAMR21_OT_NVM_ROWS / 2); i++)
    {
        samr21Nvm_eraseRowAt( swapBaseAddress + (SAMR21_NVM_SIZE_ROW * i));
    }
}

void otPlatFlashRead(otInstance *a_instance, uint8_t a_swapIndex, uint32_t a_offset, void *a_data_p, uint32_t a_size){
    OT_UNUSED_VARIABLE(a_instance);

#ifdef _DEBUG
    assert(a_SwapIndex <= 1)
#endif

    uint32_t swapBaseAddress = &_snvmem;

    if(a_swapIndex){
        swapBaseAddress += SAMR21_OT_SWAP_SIZE;
    }

    samr21Nvm_readAt(swapBaseAddress+a_offset, a_data_p, a_size);
}

void otPlatFlashWrite(otInstance *a_instance, uint8_t a_swapIndex, uint32_t a_offset, const void *a_data_p, uint32_t a_size){
    OT_UNUSED_VARIABLE(a_instance);

#ifdef _DEBUG
    assert(a_SwapIndex <= 1)
#endif

    uint32_t swapBaseAddress = &_snvmem;

    if(a_swapIndex){
        swapBaseAddress += SAMR21_OT_SWAP_SIZE;
    }

    uint32_t rowOffset = a_offset % (SAMR21_NVM_SIZE_PAGE * SAMR21_NVM_PAGES_PER_ROW);

    if( SAMR21_NVM_SIZE_ROW - rowOffset >= a_size){
        samr21Nvm_writeWithinRow(swapBaseAddress + a_offset, a_data_p, a_size);
        return;
    }

    samr21Nvm_writeWithinRow(swapBaseAddress + a_offset, a_data_p, SAMR21_NVM_SIZE_ROW - rowOffset);

    uint32_t bytesWritten = SAMR21_NVM_SIZE_ROW - rowOffset;

    while ( bytesWritten < a_size)
    {
        uint16_t sizeNextWriteBlock = (a_size - bytesWritten > SAMR21_NVM_SIZE_ROW ? SAMR21_NVM_SIZE_ROW : a_size - bytesWritten);
        samr21Nvm_writeWithinRow(
            swapBaseAddress + a_offset + bytesWritten, 
            a_data_p + bytesWritten,
            sizeNextWriteBlock
        );

        bytesWritten += sizeNextWriteBlock;
    }
}