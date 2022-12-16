//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#ifndef _SAMR21_TRX_H_
#define _SAMR21_TRX_H_

#include "samr21.h"
#include "samr21NopDelay.h"
#include <stdbool.h>

#include "at86rf233_Bitfield.h"
#include "at86rf233.h"

#define CPU_WAIT_CYCLES_AFTER_SSEL_LOW              9   // 187.5ns
#define CPU_WAIT_CYCLES_BETWEEN_BYTES               12  // 250ns
#define CPU_WAIT_CYCLES_BETWEEN_BYTES_FAST_ACCESS   24  // 500ns
#define CPU_WAIT_CYCLES_SLP_TR_PULSE                4   // 84ns
#define CPU_WAIT_CYCLES_BEFORE_SSEL_HIGH            12  // 250ns
#define CPU_WAIT_CYCLES_FOR_FRAME_BUFFER_EMPTY_FLAG 36  // 750ns

#define AT86RF233_RSSI_BASE_VAL                     -94

#define SPI_DUMMY_BYTE                              0x00

#define AT86RF233_CMD_REG_READ_MASK                 0b10000000   
#define AT86RF233_CMD_REG_WRITE_MASK                0b11000000

#define AT86RF233_CMD_FRAMEBUFFER_READ              0b00100000   
#define AT86RF233_CMD_FRAMEBUFFER_WRITE             0b01100000 
  
#define AT86RF233_CMD_SRAM_READ                     0b00000000
#define AT86RF233_CMD_SRAM_WRITE                    0b01000000

//Inits SPI Interface and Trigger Ports 
void samr21TrxInterfaceInit();
//Writes a clk Value into the TRX_CTRL_0_REG[CLKM] register to get a 16MHz Output on CLKM
void samr21TrxSetupMClk(uint8_t clk);
//Updated Local Copy of TRX STATE Register
void samr21TrxUpdateStatus();

uint8_t samr21TrxReadRegister(uint8_t addr);
void samr21TrxWriteRegister(uint8_t addr, uint8_t data);

void samr21TrxWriteToSRam(uint8_t addr, uint8_t* writeBuffer, uint8_t lenght);
void samr21TrxReadFromSRam(uint8_t addr, uint8_t* readBuffer, uint8_t lenght);

void samr21TrxSetSSel(bool enabled);
void samr21TrxSetRSTN(bool enabled);
void samr21TrxSetSLP_TR(bool enabled);

//Sends a Byte via SPI and simultaneously Recives one
uint8_t samr21TrxSpiTransceiveByteRaw(uint8_t data);

#endif // _SAMR21_TRX_H_
