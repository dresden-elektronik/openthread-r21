//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#ifndef _SAMR21_RADIO_ED_HANDLER_H_
#define _SAMR21_RADIO_ED_HANDLER_H_

#include <stdint.h>
#include <stdbool.h>
#include "samr21.h"

#include "802_15_4_Helper.h"
#include "samr21RadioIrqHandler.h"
#include "samr21Trx.h"
#include "samr21RadioTrxRegCopy.h"
#include "samr21RadioCtrl.h"
#include "samr21Timer.h"

#ifndef NUM_ENERGY_DETECTION_SCANS_PER_MS
    #define NUM_ENERGY_DETECTION_SCANS_PER_MS 4
#endif

#define TIME_UNTIL_NEXT_ENERGY_DETECTION_SCAN_us (1000 / NUM_ENERGY_DETECTION_SCANS_PER_MS)

typedef enum 
{
    ED_STATUS_IDLE                  = 0x00,
    ED_STATUS_SETUP                 = 0x01,
    ED_STATUS_WAIT_FOR_RESULT       = 0x02,
    ED_STATUS_WAIT_FOR_NEXT_SCAN    = 0x03,
    ED_STATUS_DONE                  = 0xFF
} EdStatus;




//Callback
 void cb_samr21RadioEdDone(int8_t);

#endif //_SAMR21_RADIO_ED_HANDLER_H_