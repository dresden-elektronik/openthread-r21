/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#include "samr21Timer.h"

//TC3
void samr21Timer_initDmaPaceMaker()
{
    // Enable In Power Manger
    PM->APBCMASK.bit.TC3_ = 1;

    // Disable Modules First
    TC3->COUNT16.CTRLA.bit.ENABLE = 0;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
        ;

    // Reset after
    TC3->COUNT16.CTRLA.bit.SWRST = 1;
    while (TC3->COUNT16.CTRLA.bit.SWRST || TC3->COUNT16.STATUS.bit.SYNCBUSY)
        ;

    // Setup TC Modules
    TC3->COUNT16.INTENSET.bit.OVF = 1;

    TC3->COUNT16.CTRLA.reg =
        TC_CTRLA_ENABLE | TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT16_Val) | TC_CTRLA_WAVEGEN(TC_CTRLA_WAVEGEN_MFRQ_Val) | TC_CTRLA_PRESCALER(0) // 1Mhz
        | TC_CTRLA_RUNSTDBY | TC_CTRLA_PRESCSYNC(TC_CTRLA_PRESCSYNC_GCLK);
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
        ;
    
    TC3->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_STOP 
        | TC_CTRLBSET_DIR
    ;

    while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
        ;

    // clear pending interrupt in TC Module
    TC3->COUNT16.INTFLAG.bit.OVF = 1;
    NVIC_ClearPendingIRQ(TC3_IRQn);
}

void samr21Timer_startDmaPaceMaker(uint16_t period)
{
    TC3->COUNT16.CC[0].reg = period;
    TC3->COUNT16.COUNT.reg = period;

    TC3->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_RETRIGGER;
}

void samr21Timer_stopDmaPaceMaker()
{
    TC3->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_STOP;
}




#define LINKED_LIST_BUFFER_ENTRY_DATA_TYPE timerJob_t
#define LINKED_LIST_STATIC_IMPLEMENTATION
#define LINKED_LIST_BUFFER_SIZE_AWARE //Speeds ups seeks
#include "utils/linkedListBuffer_t.h"

static timestamp_t s_now;
static bool s_timerFired = false;

#define NUM_MAX_ACTIVE_TIMER_JOBS 4

//Low Urgency Tasks
//Handled in next timer_tick() routine
static timerJob_t s_timerTasksBuffer[NUM_MAX_ACTIVE_TIMER_JOBS];
static linkedListBuffer_t s_timerTaskLinkedListBuffer =
{
    .entryPool = (linkedListBufferEntry_t * ) s_timerTasksBuffer,
    .entryPoolSize = sizeof(s_timerTasksBuffer) / sizeof(s_timerTasksBuffer[0]),
    .rootEntry = NULL,
    .numActiveEntries = 0
};

//High Urgency Tasks
//Handled from within the IRQ routine
static timerJob_t s_timerActionBuffer[NUM_MAX_ACTIVE_TIMER_JOBS];
static linkedListBuffer_t s_timerActionsLinkedListBuffer =
{
    .entryPool = (linkedListBufferEntry_t * ) s_timerActionBuffer,
    .entryPoolSize = sizeof(s_timerActionBuffer) / sizeof(s_timerActionBuffer[0]),
    .rootEntry = NULL,
    .numActiveEntries = 0
};



static void updateNowTimestamp(void)
{
    //The internal Timer is only 32Bits wide, so we use a static variable that counts the upper halfword for the application
    if(TC4->COUNT32.INTFLAG.bit.OVF)
    {
        //Reset Flag
        TC4->COUNT32.INTFLAG.bit.OVF = 1;

        //overflow of the 32 bit counter
        //increment upper halfword of timestamp
        s_now.u32Value.upper++;
    }

     //Update Lower Half-word of Timestamp
    s_now.u32Value.lower = TC4->COUNT32.COUNT.reg;
}


void samr21Timer_init()
{
    s_now.u64Value = 0x00;

    linkedList_reset(&s_timerTaskLinkedListBuffer);
    linkedList_reset(&s_timerActionsLinkedListBuffer);

    // Enable In Power Manger
    PM->APBCMASK.bit.TC4_ = 1;
    PM->APBCMASK.bit.TC5_ = 1;

    // Disable Modules First
    TC4->COUNT32.CTRLA.bit.ENABLE = 0;
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY)
        ;

    // Reset after
    TC4->COUNT32.CTRLA.bit.SWRST = 1;
    while (TC4->COUNT32.CTRLA.bit.SWRST || TC4->COUNT32.STATUS.bit.SYNCBUSY)
        ;

    //Setup as free running 32 Bit Counter with 1us resolution
    TC4->COUNT32.CTRLA.reg =
        TC_CTRLA_ENABLE 
        | TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT32_Val) 
        | TC_CTRLA_WAVEGEN(TC_CTRLA_WAVEGEN_NFRQ_Val) 
        | TC_CTRLA_PRESCALER(0) 
        | TC_CTRLA_PRESCSYNC(TC_CTRLA_PRESCSYNC_GCLK)
    ;

    //Enable Continuous Sync with Count Register
    TC4->COUNT32.READREQ.reg =
        TC_READREQ_ADDR((uint32_t)TC4 - (uint32_t)&TC4->COUNT32.COUNT.reg)
        | TC_READREQ_RCONT
    ;

    //Reset the Counter Value
    TC4->COUNT32.CC[0].reg = UINT32_MAX;
    TC4->COUNT32.CC[1].reg = UINT32_MAX;

    while (TC4->COUNT32.STATUS.bit.SYNCBUSY)
        ;


    //Start the timer
    TC4->COUNT32.CTRLBSET.reg =
        TC_CTRLBSET_CMD_RETRIGGER 
    ;

    TC4->COUNT32.COUNT.reg = 0;

     //Clear Interrupts
    TC4->COUNT32.INTFLAG.bit.MC0 = 1;
    TC4->COUNT32.INTFLAG.bit.MC1 = 1;
    TC4->COUNT32.INTFLAG.bit.OVF  = 1;

    NVIC_ClearPendingIRQ(TC4_IRQn);
    NVIC_EnableIRQ(TC4_IRQn);
}

uint32_t samr21Timer_getNowU32()
{
    updateNowTimestamp();
    return s_now.u32Value.lower;
}

uint64_t samr21Timer_getNowU64()
{
    updateNowTimestamp();
    return s_now.u64Value;
}

timestamp_t samr21Timer_getCurrentTimestamp()
{
    updateNowTimestamp();
    return s_now;
}

bool samr21Timer_addScheduledTask(timestamp_t desiredTriggerTime, samr21Timer_fired_cb triggerCallback)
{
     __disable_irq();
    if(!linkedList_getNumEntriesFree(&s_timerTaskLinkedListBuffer))
    {
        //No Job Slot left
        __enable_irq();
        return false;
    }

    linkedListBufferEntry_t * timerJob = linkedList_getFirstEntry(&s_timerTaskLinkedListBuffer);

    while (timerJob != NULL)
    {
        if(timerJob->data.triggerTime.u64Value > desiredTriggerTime.u64Value)
        {
            timerJob = linkedList_allocEntryBefore(&s_timerTaskLinkedListBuffer, timerJob);
            break;
        }
        
        timerJob = linkedList_getNext(&s_timerTaskLinkedListBuffer, timerJob);
    }
    
    if (timerJob == NULL)
    {
        //Trigger is not before any other Trigger
        timerJob = linkedList_allocEntryAtEnd(&s_timerTaskLinkedListBuffer);
    }
    
    timerJob->data.triggerTime = desiredTriggerTime;
    timerJob->data.callback = triggerCallback;

    __enable_irq();
    return true;
}

bool samr21Timer_removeScheduledTask(samr21Timer_fired_cb triggerCallback)
{
    __disable_irq();
    linkedListBufferEntry_t * timerJob = linkedList_getFirstEntry(&s_timerTaskLinkedListBuffer);

    while (timerJob != NULL)
    {
        if(timerJob->data.callback == triggerCallback)
        {
            break;
        }
        
        timerJob = linkedList_getNext(&s_timerTaskLinkedListBuffer, timerJob);
    }
    
    if (timerJob == NULL)
    {
        __enable_irq();
        return false;
    }

    linkedList_freeEntry(&s_timerTaskLinkedListBuffer,timerJob);

    __enable_irq();
    return true;
}

bool samr21Timer_addDelayedAction(uint32_t delay, samr21Timer_fired_cb triggerCallback)
{
    __disable_irq();
    timestamp_t desiredTriggerTime =
    {
        .u64Value = samr21Timer_getNowU64() + delay
    };

    if(!linkedList_getNumEntriesFree(&s_timerActionsLinkedListBuffer))
    {
        //No Job Slot left
        __enable_irq();
        return false;
    }

    bool newActionIsNextPending = true;

    linkedListBufferEntry_t * timerActionEntry = linkedList_getFirstEntry(&s_timerActionsLinkedListBuffer);

    while (timerActionEntry != NULL)
    {
        if(timerActionEntry->data.triggerTime.u64Value > desiredTriggerTime.u64Value)
        {
            timerActionEntry = linkedList_allocEntryBefore(&s_timerActionsLinkedListBuffer, timerActionEntry);
            break;
        }
        
        timerActionEntry = linkedList_getNext(&s_timerActionsLinkedListBuffer, timerActionEntry);
        newActionIsNextPending = false;
    }
    
    if (timerActionEntry == NULL)
    {
        //Trigger is not before any other Trigger
        timerActionEntry = linkedList_allocEntryAtEnd(&s_timerActionsLinkedListBuffer);
    }
    
    timerActionEntry->data.triggerTime = desiredTriggerTime;
    timerActionEntry->data.callback = triggerCallback;

    if(newActionIsNextPending)
    {
        // Modify Compare Register in Hardware Timer
        TC4->COUNT32.CC[0].reg = desiredTriggerTime.u32Value.lower;
        while(TC4->COUNT32.STATUS.bit.SYNCBUSY);

        //Enable IRQ
        TC4->COUNT32.INTENSET.bit.MC0 = 1;
    }

    __enable_irq();
    return true;
}


bool samr21Timer_removeDelayedAction(samr21Timer_fired_cb triggerCallback)
{
    __disable_irq();
    linkedListBufferEntry_t * pendingActionEntry = linkedList_getFirstEntry(&s_timerActionsLinkedListBuffer);

    while (pendingActionEntry != NULL)
    {
        if(pendingActionEntry->data.callback == triggerCallback)
        {
            break;
        }
        
        pendingActionEntry = linkedList_getNext(&s_timerActionsLinkedListBuffer, pendingActionEntry);
    }
    
    if (pendingActionEntry == NULL)
    {
        __enable_irq();
        return false;
    }

    linkedList_freeEntry(&s_timerActionsLinkedListBuffer,pendingActionEntry);
    __enable_irq();
    return true;
}

void samr21Timer_tick()
{
         __disable_irq();

        //Check for pending Timer Jobs
        linkedListBufferEntry_t * pendingTaskEntry = linkedList_getFirstEntry(&s_timerTaskLinkedListBuffer);
        samr21Timer_fired_cb pendingTask = NULL;

        while (pendingTaskEntry->data.triggerTime.u64Value < samr21Timer_getNowU64())
        {
            pendingTask = pendingTaskEntry->data.callback;

            //Remove Entry
            linkedList_freeEntry(&s_timerTaskLinkedListBuffer, pendingTaskEntry);

            if(pendingTask)
            {
                 __enable_irq();
                pendingTask();
                 __disable_irq();
            }


            //Check if there is more to do
            pendingTaskEntry = linkedList_getFirstEntry(&s_timerTaskLinkedListBuffer);
        }
        __enable_irq();
}

void TC4_Handler()
{
    //Check if this Interrupt must exec a timing critical Callback
    if(TC4->COUNT32.INTFLAG.bit.MC0)
    {
        //Disable IRQ for now
        TC4->COUNT32.INTENCLR.bit.MC0 = 1;
        //Clear Interrupt
        TC4->COUNT32.INTFLAG.bit.MC0 = 1;

        //Get the current Action Entry
        linkedListBufferEntry_t * pendingActionEntry =  linkedList_getFirstEntry(&s_timerActionsLinkedListBuffer); 

        while(pendingActionEntry)
        {
            samr21Timer_fired_cb pendingAction = pendingActionEntry->data.callback;

            if (pendingActionEntry->data.triggerTime.u64Value <= samr21Timer_getNowU64())
            {
                //Remove Entry
                linkedList_freeEntry(&s_timerActionsLinkedListBuffer, pendingActionEntry);

                if(pendingAction)
                {
                    pendingAction();
                }
            }
            else
            {
                //Clear Interrupt
                TC4->COUNT32.INTFLAG.bit.MC0 = 1;

                //Modify Compare Register in Hardware Timer
                TC4->COUNT32.CC[0].reg = pendingActionEntry->data.triggerTime.u32Value.lower;

                //Arm IRQ again
                TC4->COUNT32.INTENSET.bit.MC0 = 1;
                return;
            }

            pendingActionEntry =  linkedList_getFirstEntry(&s_timerActionsLinkedListBuffer); 
        }
    }
}