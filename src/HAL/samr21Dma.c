#include "samr21Dma.h"

typedef struct DmacDescriptorSection_s
{
    DmacDescriptor channel[SAMR21_NUM_DMA_CHANNEL];
}DmacDescriptorSection_t;

static DmacDescriptorSection_t s_dmacDescriptor;
static DmacDescriptorSection_t s_dmacWriteBack;

static samr21Dma_done_cb s_channelDoneCallbacks[SAMR21_NUM_DMA_CHANNEL] = { NULL };
static bool s_channelInitted[SAMR21_NUM_DMA_CHANNEL] = { false };

void samr21Dma_init(void)
{
    // Enable DMA in Power Manager 
    PM->APBBMASK.bit.DMAC_ = 1;
    PM->AHBMASK.bit.DMAC_ = 1;

    // Disable DMA (and the adjacent CRC-Module) at first
    DMAC->CTRL.bit.DMAENABLE = 0;
    DMAC->CTRL.bit.CRCENABLE = 0;

    // Wait till both are disabled
    while (DMAC->CTRL.bit.DMAENABLE || DMAC->CTRL.bit.CRCENABLE)
        ;

    // Reset DMA Module
    DMAC->CTRL.bit.SWRST = 1;

    // Wait for reset to finish
    while (DMAC->CTRL.bit.SWRST)
        ;

    //Reset everything stored in RAM
    memset(&s_dmacDescriptor, 0x00, sizeof(DmacDescriptorSection_t));
    memset(&s_dmacWriteBack, 0x00, sizeof(DmacDescriptorSection_t));
    memset(&s_channelDoneCallbacks, 0x00, sizeof(s_channelDoneCallbacks));
    memset(&s_channelInitted, 0x00, sizeof(s_channelInitted));

    // Set relevant DMAC Discriptor Adresses
    DMAC->BASEADDR.reg = &s_dmacDescriptor.channel[0];
    DMAC->WRBADDR.reg = &s_dmacWriteBack.channel[0];

    // Enable the DMAC Module and Prio-Level 0  (All DMA channel use LVL0, Prio is done by lower DMA Channel index)
    DMAC->CTRL.reg =
        DMAC_CTRL_DMAENABLE
        |DMAC_CTRL_LVLEN0;

    // Prio Setup:
    // Only use Level 0
    DMAC->PRICTRL0.reg =
        DMAC_PRICTRL0_LVLPRI0(0) //Channel 0 has the highest Prio
        //| DMAC_PRICTRL0_RRLVLEN0 //If Set RoundRobin is used 
        //| DMAC_PRICTRL0_LVLPRI1(0) 
        //| DMAC_PRICTRL0_RRLVLEN1
        //| DMAC_PRICTRL0_LVLPRI2(0) 
        //| DMAC_PRICTRL0_RRLVLEN2
        //| DMAC_PRICTRL0_LVLPRI3(0) 
        //| DMAC_PRICTRL0_RRLVLEN3
    ;

    NVIC_EnableIRQ(DMAC_IRQn);
}

bool samr21Dma_initChannel(uint8_t channel, uint32_t targetAddress, uint8_t triggerSource, samr21Dma_done_cb callbackFunc)
{
    if(channel >= SAMR21_NUM_DMA_CHANNEL)
    {
        return false;
    }

    __disable_irq();
    DMAC->CHID.bit.ID = channel;
    if(DMAC->CHCTRLA.bit.ENABLE)
    {
        __enable_irq();
        return false;
    }

    // Setup channel 0 for Transmission of 1 Byte till Sercom is ready for the next byte
    DMAC->CHCTRLB.reg =
        DMAC_CHCTRLB_CMD_NOACT 
        | DMAC_CHCTRLB_TRIGACT_BEAT 
        | DMAC_CHCTRLB_TRIGSRC(triggerSource) //  TC3 Overflowtrigger (Table 18-8. Peripheral Trigger Source SAMR21-Datasheet)
        | DMAC_CHCTRLB_LVL_LVL0      //  Set Prio-Level 0 
        //| DMAC_CHCTRLB_EVOE        //  Event Generation is disabled
        //| DMAC_CHCTRLB_EVIE        //  Event Input is disabled
        | DMAC_CHCTRLB_EVACT_NOACT   //  Not Action defined for an input Event
        ;

    //Enable Transfer Completed Callback
    DMAC->CHINTENSET.reg =
       DMAC_CHINTENSET_TCMPL;

    s_dmacDescriptor.channel[channel].BTCTRL.bit.VALID = 0; //Set to True when transfer starts
    s_dmacDescriptor.channel[channel].BTCTRL.bit.EVOSEL = DMAC_BTCTRL_EVOSEL_DISABLE_Val; // Don't use Event System
    s_dmacDescriptor.channel[channel].BTCTRL.bit.BLOCKACT = DMAC_BTCTRL_BLOCKACT_NOACT_Val; // Fulfill all Linked Descriptors without intervention 
    s_dmacDescriptor.channel[channel].BTCTRL.bit.BEATSIZE = DMAC_BTCTRL_BEATSIZE_BYTE_Val; // Always Single Byte
    s_dmacDescriptor.channel[channel].BTCTRL.bit.SRCINC = 1;
    s_dmacDescriptor.channel[channel].BTCTRL.bit.DSTINC = 0;   //SERCOM[X]->Data in all Cases
    s_dmacDescriptor.channel[channel].BTCTRL.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_SRC_Val;
    s_dmacDescriptor.channel[channel].BTCTRL.bit.STEPSIZE = DMAC_BTCTRL_STEPSIZE_X1_Val; //Single Byte jumps in memory

    s_dmacDescriptor.channel[channel].DSTADDR.bit.DSTADDR = targetAddress;

    s_channelDoneCallbacks[channel] = callbackFunc;
    s_channelInitted[channel] = true;

    __enable_irq();
    return true;
}

bool samr21Dma_activateChannel(uint8_t channel, uint8_t * data, uint32_t dataLength, DmacDescriptor * linkedDescriptor)
{
    if((channel >= SAMR21_NUM_DMA_CHANNEL) || !s_channelInitted[channel])
    {
        return false;
    }

    __disable_irq();
    DMAC->CHID.bit.ID = channel;
    if(DMAC->CHCTRLA.bit.ENABLE)
    {
        __enable_irq();
        return false;
    }

    s_dmacDescriptor.channel[channel].BTCTRL.bit.VALID = 0;

    s_dmacDescriptor.channel[channel].BTCNT.bit.BTCNT = dataLength;
    s_dmacDescriptor.channel[channel].SRCADDR.bit.SRCADDR = (uint32_t)data + dataLength;
    s_dmacDescriptor.channel[channel].DESCADDR.bit.DESCADDR = (uint32_t)linkedDescriptor;
    
    s_dmacDescriptor.channel[channel].BTCTRL.bit.VALID = 1;
    
    DMAC->CHID.bit.ID = channel;
    DMAC->CHCTRLA.bit.ENABLE = 1;

    __enable_irq();
    return true;
}

void samr21Dma_triggerChannelAction(uint8_t channel)
{
    if(channel >= SAMR21_NUM_DMA_CHANNEL)
    {
        return false;
    }
    DMAC->SWTRIGCTRL.reg = 1 << channel;
}

// IRQ ISR triggerd by DMAC
void DMAC_Handler()
{
    uint8_t irqChannel = DMAC->INTPEND.bit.ID;

    //Clear Interrupt-Flag of the channel
    DMAC->INTPEND.bit.TCMPL = 1;

    //Disable Corresponding DMA Channel
    DMAC->CHID.bit.ID = irqChannel;
    DMAC->CHCTRLA.bit.ENABLE = 0;

    //Mark Descriptor as invalid 
    s_dmacDescriptor.channel[irqChannel].BTCTRL.bit.VALID = 0;

    if(s_channelDoneCallbacks[irqChannel])
    {
        //Exec Callback Function (if there is one)
        (*s_channelDoneCallbacks[irqChannel])();
    }
}