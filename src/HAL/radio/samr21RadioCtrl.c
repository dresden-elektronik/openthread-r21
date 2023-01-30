/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#include "samr21RadioCtrl.h"

static Samr21RadioState s_radioState = SAMR21_RADIO_STATE_DISABLED;

void samr21RadioInitIrq()
{
    if(s_radioState){
        return;
    }
    // Get TRX into IDLE (STATUS_TRX_OFF) State
    samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_FORCE_TRX_OFF);
    while (g_trxStatus.bit.trxStatus != TRX_STATUS_TRX_OFF){
        samr21TrxUpdateStatus();
    }

    samr21RadioRxResetAllBuffer();
    

    //Enable EIC in Power Manager
    PM->APBAMASK.bit.EIC_ = 1;

    // Enable IRQ via EIC
    //Use GCLKGEN0 as core Clock for EIC (At86rf233, IRQ_Detect)
    GCLK->CLKCTRL.reg =
        //GCLK_CLKCTRL_WRTLOCK
        GCLK_CLKCTRL_CLKEN
        |GCLK_CLKCTRL_GEN(0) // GCLKGEN1
        |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EIC_Val)
    ;
    //Wait for synchronization 
    while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );  
    // Reset first and wait for reset to finish
    EIC->CTRL.bit.SWRST = 1;
    while (EIC->STATUS.bit.SYNCBUSY);
    while (EIC->CTRL.bit.SWRST);

    // Enable EXINT[0]
    EIC->CONFIG[0].bit.FILTEN0 = 0;
    EIC->CONFIG[0].bit.SENSE0 = EIC_CONFIG_SENSE0_RISE_Val;
    EIC->INTENSET.bit.EXTINT0 = 1;

    // Enable Module
    EIC->CTRL.bit.ENABLE = 1;
    while (EIC->STATUS.bit.SYNCBUSY);
    while (!(EIC->CTRL.bit.ENABLE));

    //cLEAR irq BEFORE ENABLING
    EIC->INTFLAG.bit.EXTINT0 = 1;

    //Read Once to clear all pending Interrupts
    samr21TrxReadRegister(IRQ_STATUS_REG);
    // Enable IRQ in NVIC
    __NVIC_EnableIRQ(EIC_IRQn);

    s_radioState = SAMR21_RADIO_STATE_SLEEP;
}
void samr21RadioDeinitIrq()
{
    if(!s_radioState){
        return;
    }

    samr21RadioRemoveEventHandler();
    __NVIC_DisableIRQ(EIC_IRQn);

    // Get TRX into IDLE (STATUS_TRX_OFF) State
    samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_FORCE_TRX_OFF);
    while (g_trxStatus.bit.trxStatus != TRX_STATUS_TRX_OFF){
        samr21TrxUpdateStatus();
    }

    // Reset EIC and wait for reset to finish
    EIC->CTRL.bit.SWRST = 1;
    while (EIC->STATUS.bit.SYNCBUSY);
    while (EIC->CTRL.bit.SWRST);

    // Disable EIC-Clock
    GCLK->CLKCTRL.reg =
        //GCLK_CLKCTRL_WRTLOCK
        //GCLK_CLKCTRL_CLKEN
        GCLK_CLKCTRL_GEN(0) // GCLKGEN1
        |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EIC_Val)
    ;
    //Wait for synchronization 
    while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );  

    //Disable EIC in Power Manager
    PM->APBAMASK.bit.EIC_ = 0;

    s_radioState = SAMR21_RADIO_STATE_DISABLED;
}

void samr21RadioCtrlSetState(Samr21RadioState State){
    s_radioState = State;
}

Samr21RadioState samr21RadioCtrlGetState(){
    return s_radioState;
}

int8_t samr21RadioCtrlGetRssi(){
    if(
        g_trxStatus.bit.trxStatus != TRX_STATUS_RX_ON
        && g_trxStatus.bit.trxStatus != TRX_STATUS_RX_ON 
    ){
        return INT8_MAX;
    }

    return AT86RF233_RSSI_BASE_VAL + (int8_t)samr21TrxReadRegister(PHY_ED_LEVEL_REG);
}

//TX Power 
#define SIZE_AT86RF233_TX_POWER_TABLE 16
static const int8_t s_txPowerTable[SIZE_AT86RF233_TX_POWER_TABLE]={16,15,14,12,10,8,4,0,-4,-8,-12,-16,-24,-32,-48,-68};
void samr21RadioCtrlSetTxPower(int8_t txPower)
{
    txPower *= 4; // multiply by 4 (for resolution in txPowerTable)

    for(uint8_t i = 0; i < SIZE_AT86RF233_TX_POWER_TABLE; i++){
        if(
            (s_txPowerTable[i] >= txPower)
            && (s_txPowerTable[i+1] <= txPower)
        ){
            g_phyTxPwr.bit.txPwr = i;
            goto writeReg;
        }
    }

    //Default
    g_phyTxPwr.bit.txPwr = 0x7;

writeReg:
    samr21TrxWriteRegister(PHY_TX_PWR_REG, g_phyTxPwr.reg);
}
int8_t samr21RadioCtrlGetTxPower(){
    return s_txPowerTable[g_phyTxPwr.bit.txPwr] >> 2; // devide by 4
}

void samr21RadioCtrlSetCCAMode(uint8_t newCcaMode)
{
    g_phyCcCcaReg.bit.ccaMode = newCcaMode;
    samr21TrxWriteRegister(PHY_CC_CCA_REG, g_phyCcCcaReg.reg);
}
uint8_t samr21RadioCtrlGetCCAMode()
{
    return g_phyCcCcaReg.bit.ccaMode;
}

void samr21RadioCtrlSetCcaThreshold(int8_t threshold)
{
    int8_t diff = (AT86RF233_RSSI_BASE_VAL - threshold);
    g_ccaThres.bit.ccaEdThres = abs(diff) >> 1; //Devide by 2

    samr21TrxWriteRegister(CCA_THRES_REG, g_ccaThres.reg);
}
int8_t samr21RadioCtrlGetCurrentCcaThreshold()
{
    return ( AT86RF233_RSSI_BASE_VAL + ( g_ccaThres.bit.ccaEdThres << 1 ) ); //Multiply by 2
}

void samr21RadioCtrlSetPromiscuousMode(bool enable){
    g_promiscuousMode = enable;
}
bool samr21RadioCtrlGetPromiscuousMode(){
    return g_promiscuousMode;
}

void samr21RadioCtrlSetMacFrameCounter(uint32_t macFrameCounter){
    g_macFrameCounter = macFrameCounter;
}
uint32_t samr21RadioCtrlGetMacFrameCounter(){
    return g_macFrameCounter;
}

void samr21RadioCtrlUpdateCslSampleTime(uint32_t cslSampleTime){
    g_cslSampleTime = cslSampleTime;
}

uint16_t samr21RadioCtrlCslGetPhase()
{
    uint32_t curTime       = samr21RtcGetTimestamp();
    uint32_t cslPeriodInUs = g_cslPeriod * OT_US_PER_TEN_SYMBOLS;
    uint32_t diff = (cslPeriodInUs - (curTime % cslPeriodInUs) + (g_cslSampleTime % cslPeriodInUs)) % cslPeriodInUs;
    return (uint16_t)(diff / OT_US_PER_TEN_SYMBOLS + 1);
}

Samr21RadioState samr21RadioCtrlReturnToLastHandler()
{

    if (
        s_radioState == SAMR21_RADIO_STATE_DISABLED || s_radioState == SAMR21_RADIO_STATE_SLEEP || s_radioState == SAMR21_RADIO_STATE_INVALID)
    {
        if (s_radioState == SAMR21_RADIO_STATE_INVALID)
        {
            s_radioState = SAMR21_RADIO_STATE_DISABLED;
        }
        samr21RadioRemoveEventHandler();
        g_irqMask = (AT86RF233_REG_IRQ_MASK_t){
            .bit.pllLock = 0,
            .bit.pllUnlock = 0,
            .bit.rxStart = 0,
            .bit.trxEnd = 0,
            .bit.ccaEdDone = 0,
            .bit.addressMatch = 0,
            .bit.bufferUnderRun = 0,
            .bit.batteryLow = 0};
        samr21TrxWriteRegister(IRQ_MASK_REG, g_irqMask.reg);
        samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_TRX_OFF);
        return s_radioState;
    }

    g_irqMask = (AT86RF233_REG_IRQ_MASK_t){
        .bit.pllLock = 0,
        .bit.pllUnlock = 0,
        .bit.rxStart = 1,
        .bit.trxEnd = 1,
        .bit.ccaEdDone = 0,
        .bit.addressMatch = 0,
        .bit.bufferUnderRun = 1,
        .bit.batteryLow = 0};
    samr21TrxWriteRegister(IRQ_MASK_REG, g_irqMask.reg);

    samr21RadioRxPrepareBuffer();
    
    samr21RadioSetEventHandler(&samr21RadioRxEventHandler);
    
    samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);
    
    if(s_radioState == SAMR21_RADIO_STATE_TRANSMIT){
        s_radioState = SAMR21_RADIO_STATE_RECEIVE;
    }

    return true;
}

void samr21RadioCtrlSetMacKeys(

    uint8_t        aKeyId,
    const uint8_t  *aPrevKey,
    const uint8_t  *aCurrKey,
    const uint8_t  *aNextKey
){
    __disable_irq();

    g_currKeyId = aKeyId;

    memcpy(g_prevKey,aPrevKey,OT_AES_KEY_SIZE);
    memcpy(g_currKey,aCurrKey,OT_AES_KEY_SIZE);
    memcpy(g_nextKey,aNextKey,OT_AES_KEY_SIZE);

    __enable_irq();
}
