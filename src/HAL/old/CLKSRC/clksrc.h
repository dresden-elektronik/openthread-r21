// Author ERH for dresden elektronik ingenieurtechnik gmbh © 2022
#include "compiler.h"

#define SYSCTRL_BASE_ADDR        0x40000800

#define SYSCTRL_XOSC_OFFSET          0x10
#define SYSCTRL_XOSC32K_OFFSET       0x14
#define SYSCTRL_OSC32K_OFFSET        0x18
#define SYSCTRL_OSCULP32K_OFFSET     0x1C
#define SYSCTRL_OSC8M_OFFSET         0x20
#define SYSCTRL_DFLLCTRL_OFFSET      0x24
#define SYSCTRL_DFLLVAL_OFFSET       0x28
#define SYSCTRL_DFLLMUL_OFFSET       0x2C
#define SYSCTRL_DFLLSYNC_OFFSET      0x30

void setupXOSC(
    bool enable,
    bool onDemand,
    bool runInStandby,

    uint8_t startUpTime,
    uint8_t gain,
    bool autoAmplitudeGainControl,
    bool isCrystal
);

void setupXOSC32K(
    bool enable,
    bool onDemand,
    bool runInStandby,

    bool autoAmplitudeControl,
    bool enableOutput, 
    uint8_t startUpTime,
    bool isCrystal,
    bool lockConfig
);

void setupOSC32K(
    bool enable,
    bool onDemand,
    bool runInStandby,

    bool enableOutput, 
    uint8_t startUpTime,
    uint8_t calibrationVal,
    bool lockConfig
);

void setupOSCULP32K(
    uint8_t calibrationVal,
    bool lockConfig
);


void setupOSC8M(
    bool enable,
    bool onDemand,
    bool runInStandby,

    uint8_t frequencyRange,
    uint16_t calibrationVal,
    uint8_t preScaler
);


void setupDFLL(
    bool enable,
    bool onDemand,
    bool runInStandby,

    bool waitLock,
    bool bypassCoarseLock,
    bool quickLock,
    bool chillCycle,
    bool usbClockRecovery,
    bool loseLockAfterWake,
    bool stableDFLLFrequency,
    bool closedLoop

    uint8_t coarseValue,
    uint16_t fineValue,

    uint8_t coarseStep,
    uint16_t fineStep,

    uint16_t multiplyFactor
);

