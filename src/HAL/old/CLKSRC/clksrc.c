#include "clksrc.h"

void setupXOSC(
    bool enable,
    bool onDemand,
    bool runInStandby,

    uint8_t startUpTime,
    uint8_t gain,
    bool autoAmplitudeGainControl,
    bool isCrystal
){
    uint16_t* reg = SYSCTRL_BASE_ADDR+SYSCTRL_XOSC_OFFSET;
    *(reg) = ((uint16_t)enable               << 1)
    | ((uint16_t)isCrystal                   << 2)
    | ((uint16_t)runInStandby                << 6)
    | ((uint16_t)onDemand                    << 7)
    | ((uint16_t)(gain & 0x7)                << 8)
    | ((uint16_t)autoAmplitudeGainControl    << 11)
    | ((uint16_t)(startUpTime & 0xF)         << 12);
}

void setupXOSC32K(
    bool enable,
    bool onDemand,
    bool runInStandby,

    bool autoAmplitudeControl,
    bool enableOutput, 
    uint8_t startUpTime,
    bool isCrystal,
    bool lockConfig
){
    uint16_t* reg = SYSCTRL_BASE_ADDR+SYSCTRL_XOSC32K_OFFSET;
    *(reg) = ((uint16_t)enable               << 1)
    | ((uint16_t)isCrystal                   << 2)
    | ((uint16_t)enableOutput                << 3)
    | ((uint16_t)autoAmplitudeControl        << 5)
    | ((uint16_t)runInStandby                << 6)
    | ((uint16_t)onDemand                    << 7)
    | ((uint16_t)(startUpTime & 0x7)         << 8)
    | ((uint16_t)lockConfig                  << 12);
}

void setupXOSC32K(
    bool enable,
    bool onDemand,
    bool runInStandby,

    bool autoAmplitudeControl,
    bool enableOutput, 
    uint8_t startUpTime,
    bool isCrystal,
    bool lockConfig
){
    uint16_t* reg = SYSCTRL_BASE_ADDR+SYSCTRL_XOSC32K_OFFSET;
    *(reg) = ((uint16_t)enable               << 1)
    | ((uint16_t)isCrystal                   << 2)
    | ((uint16_t)enableOutput                << 3)
    | ((uint16_t)autoAmplitudeControl        << 5)
    | ((uint16_t)runInStandby                << 6)
    | ((uint16_t)onDemand                    << 7)
    | ((uint16_t)(startUpTime & 0x7)         << 8)
    | ((uint16_t)lockConfig                  << 12);
}

void setupOSC32K(
    bool enable,
    bool onDemand,
    bool runInStandby,

    bool enableOutput, 
    uint8_t startUpTime,
    uint8_t calibrationVal,
    bool lockConfig
){
    uint32_t* reg = SYSCTRL_BASE_ADDR+SYSCTRL_OSC32K_OFFSET;
    *(reg) = ((uint32_t)enable               << 1)
    | ((uint32_t)enableOutput                << 2)
    | ((uint32_t)runInStandby                << 6)
    | ((uint32_t)onDemand                    << 7)
    | ((uint32_t)(startUpTime & 0x7)         << 8)
    | ((uint32_t)lockConfig                  << 12)
    | ((uint32_t)(calibrationVal & 0x7F)     << 16);
}

void setupOSCULP32K(
    uint8_t calibrationVal,
    bool lockConfig
){
    uint8_t* reg = SYSCTRL_BASE_ADDR+SYSCTRL_OSC32K_OFFSET;
    *(reg) = (uint8_t)calibrationVal
    | ((uint8_t)lockConfig                << 7);
}

void setupOSC8M(
    bool enable,
    bool onDemand,
    bool runInStandby,

    uint8_t frequencyRange,
    uint16_t calibrationVal,
    uint8_t preScaler
){
    uint32_t* reg = SYSCTRL_BASE_ADDR+SYSCTRL_OSC32K_OFFSET;
    *(reg) = ((uint32_t)enable               << 1)
    | ((uint32_t)runInStandby                << 6)
    | ((uint32_t)onDemand                    << 7)
    | ((uint32_t)(preScaler & 0x3)           << 8)
    | ((uint32_t)(calibrationVal & 0xFFF)    << 16)
    | ((uint32_t)(frequencyRange & 0x3)      << 30);
}

