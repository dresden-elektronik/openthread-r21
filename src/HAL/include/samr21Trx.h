/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */

#ifndef _SAMR21_TRX_H_
#define _SAMR21_TRX_H_

#include "samr21.h"
#include "samr21NopDelay.h"
#include "samr21Timer.h"
#include "samr21At86rf233.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define SAMR21_SPI_TIME_PER_BYTE_us 10
#define SAMR21_SPI_INIT_TIME_FRAMEBUFFER_us 3
#define SAMR21_SPI_INIT_TIME_REG_us 3
#define SAMR21_SPI_INIT_TIME_SRAM_us 5

#define SPI_DUMMY_BYTE 0x00

#define NUM_TRX_IRQS 8

enum
{
    TRX_IRQ_PLL_LOCK = 0x0,
    TRX_IRQ_PLL_UNLOCK = 0x1,
    TRX_IRQ_RX_START = 0x2,
    TRX_IRQ_TRX_END = 0x3,
    TRX_IRQ_CCA_ED_DONE = 0x4,
    TRX_IRQ_AMI = 0x5,
    TRX_IRQ_TRX_UR = 0x6,
    TRX_IRQ_BAT_LOW = 0x7
};
/**
 * Inits SPI and Pin Interface to At86rf233
 * Involves Changes to CLK-Domain
 */
void samr21TrxInterfaceInit();

/**
 * Setups External IRQ Controller for Interrupts From Trx
 *
 */
void samr21TrxInterruptInit();

/**
 * Deinit the IRQ Controller for Interrupts From Trx
 *
 */
void samr21TrxInterruptDeinit();

/**
 * Changes the CLKm Output of AT86RF233 (SAMR21 Datasheet 38.6.4 Master Clock Signal Output)
 *
 * @param[in]  clk  mClk Output Setting Value
 */
void samr21TrxSetupMClk(uint8_t clk);

/**
 * 
 * Updates Local Copy of TRX STATE Register
 *
 */
void samr21TrxUpdateStatusRegister();

/**
 * Gets a Register Value of the AT86rf233
 * via SPI-Bus
 * @param[in]  addr  Address of AT86rf233-Register to be read (See samr21 Datasheet 41. AT86RF233 Register Reference)
 *
 * @returns     Value of Register in question
 */
uint8_t samr21TrxReadRegister(uint8_t addr);

/**
 * Sets a Register Value to the AT86rf233
 * via SPI-Bus
 * @param[in]  addr  Address of AT86rf233-Register to be written to (See samr21 Datasheet 41. AT86RF233 Register Reference)
 *
 * @param[in]  data  Value the Register will be set to
 *
 */
void samr21TrxWriteRegister(uint8_t addr, uint8_t data);

/**
 * Opens a SPI-Transaction with the AT86RF233
 * and executes the given Transaction-Init-Command
 * (See samr21 Datasheet 35.3 SPI Protocol)
 *
 * @param[in]  command  Init Command for SPI-Transaction
 *
 * @param[in]  data     Optional Address-Value for for InitCommand
 *
 */
void samr21TrxSpiStartAccess(uint8_t command, uint8_t addr);

/**
 * Closes (and Ends) a SPI-Transaction
 *
 */
void samr21TrxSpiCloseAccess();

/**
 * Sets The Output of the "Reset" Pin to the AT86RF233
 * (See samr21 Datasheet 35. AT86RF233 Microcontroller Interface)
 *
 * @param[in]  enable  True = Active, False = Inactive
 */
void samr21TrxSetRSTN(bool enable);

/**
 * Sets The Output of the "Sleep" / "Start Transmission" Pin to the AT86RF233
 * (See samr21 Datasheet 35. AT86RF233 Microcontroller Interface)
 *
 * @param[in]  enable  True = Active, False = Inactive
 */
void samr21TrxSetSLP_TR(bool enable);

/**
 * Sends and simultaneously retrieves a raw Byte via the SPI-BUS
 *
 * @param[in]  data  Byte to be Transmitted
 *
 * @returns     retrieved Byte
 */
uint8_t samr21TrxSpiTransceiveByteRaw(uint8_t data);

/**
 * Sends and simultaneously retrieves a raw Bytes via the SPI-BUS
 *
 * @param[in]  inData  pointer to Buffer with the data to be transmitted
 *
 * @param[out]  outData  pointer to Buffer where the received data is stored
 *
 * @param[in]   len length of data to be transmitted
 */
void samr21TrxSpiTransceiveBytesRaw(uint8_t *inData, uint8_t *outData, uint8_t len);

/**
 * Sends a DummyByte to the SPI-Bus an retrieves a Byte from the Bus
 *
 * @returns     retrieved Byte
 */
uint8_t samr21TrxSpiReadByteRaw();

/**
 * Queues a State Change to RX on the AT86RF233. Takes Place when current action is completed
 *
 * @param[in]  blocking  if true the function only returns when State change took place
 */
void samr21TrxQueueMoveToRx(bool blocking);

/**
 * Queues a State Change to TX on the AT86RF233. Takes Place when current action is completed
 *
 * @param[in]  blocking  if true the function only returns when State change took place
 */
void samr21TrxQueueMoveToTx(bool blocking);

/**
 * Forces a State Change to TX on the AT86RF233
 *
 * @param[in]  blocking  if true the function only returns when State change took place
 */
void samr21TrxForceMoveToTx(bool blocking);

/**
 * Queues a State Change to TRX_OFF on the AT86RF233
 *
 * @param[in]  blocking  if true the function only returns when State change took place
 */
void samr21TrxQueueMoveToIdle(bool blocking);

/**
 * Forces a State Change to TRX_OFF on the AT86RF233
 *
 * @param[in]  blocking  if true the function only returns when State change took place
 */
void samr21TrxForceMoveToIdle(bool blocking);

/**
 * Sets operational Channel for the AT86rf233 Receive- and Transmit-Actions
 *
 * @param[in]  channel  desired channel ID (11 <= channel <= 26)
 */
void samr21TrxSetChannel(uint8_t channel);

/**
 * Gets the Channel the AT86rf233 is currently operating on
 *
 * @returns  current Channel ID
 */
uint8_t samr21TrxGetChannel();

/**
 * Sets the transmit Power of the AT86rf233
 *
 * @param[in]  power  desired Tx Power in dBm (See samr21 datasheet 38.2.4 TX Power Ramping)
 * 
 */
void samr21TrxSetTxPower(int8_t power);

/**
 * Sets the transmit Power of the AT86rf233
 *
 * @returns  current Tx Power in dBm (See samr21 datasheet 38.2.4 TX Power Ramping)
 */
int8_t samr21TrxGetTxPower();

/**
 * True Random engine of the AT86rf233
 *
 * @returns  uint8 with the 2 lowest Bits containing a random Value
 */
uint8_t samr21TrxGetRandomCrumb();

/**
 * True Random engine of the AT86rf233
 *
 * @returns  uint8 with the 4 lowest Bits containing a random Value
 */
uint8_t samr21TrxGetRandomNibble();

/**
 * True Random engine of the AT86rf233
 *
 * @returns  uint8 containing a random Value
 */
uint8_t samr21TrxGetRandomByte();


/**
 *
 * Downloads the content of the Framebuffer to a specified length, while a Frame is simultaneously received.
 * This function is blocking till the FrameBuffer is filled to the desired length or a Timeout triggers
 *
 * @param[out]  data  buffer the Framebuffer is downloaded to
 * @param[in]   len   length of the Framebuffer Download
 *
 * @returns  true if the framebuffer was downloaded to the specified length, false if the action ran into a timeout
 */
bool samr21TrxLiveFramebufferDownload(uint8_t *data, uint8_t len);

/**
 *
 * Downloads the content of the Framebuffer to a specified length, while a Frame is simultaneously received.
 * This function is blocking till the FrameBuffer is filled to psdu-Length or a Timeout triggers
 *
 * @param[out]  psduLen  pointer to to a uint8 to write the psduLen to
 * @param[out]  psdu     pointer to buffer the psdu is written to
 * @param[out]  LQI      pointer to to a uint8 to write the LQI to
 * @param[out]  RSSI     pointer to to a uint8 to write the RSSI to
 * @param[in]  live         set True if frame reception is still ongoing
 *
 * @returns  true if the downloaded content of Framebuffer is valid, false if the content is invalid or a timeout triggered
 */
bool samr21TrxDownloadFramebuffer(uint8_t *psduLen, uint8_t *psdu, uint8_t *LQI, int8_t *RSSI, bool live);

/**
 *
 * Uploads data of a specified length to a given position in the Framebuffer
 *
 * @param[in]  data  pointer to buffer of the data to be uploaded
 * @param[in]   len   length of the data to be uploaded
 * @param[in]   pos   offset inside the Framebuffer where the data is uploaded to
 *
 */
void samr21TrxUploadToFramebuffer(uint8_t *data, uint8_t len, uint8_t pos);

/**
 *
 * Uploads data of a specified length to a given position in the Framebuffer 
 * Each byte is uploaded individually each 32us (802.15.4 Datarate) 
 * After This function Call the Content of the given Framebuffer in SRAM can still be modified
 *
 * @param[in]  data  pointer to buffer of the data to be uploaded
 * @param[in]   len   length of the data to be uploaded
 * @param[in]   pos   offset inside the Framebuffer where the data is uploaded to
 *
 */
void samr21TrxStartJustInTimeUploadToFramebuffer(uint8_t *data, uint8_t len, uint8_t pos);

/**
 *
 * Adds a IRQ-Handler for a certain Type of Interrupt from the AT86RF233
 *
 * @param[in]  irqType IRQ-Type for the new handler (see samr21 datasheet 35.7 Interrupt Logic)
 * @param[in]  handler pointer to new associated ISR (or NULL to remove a handler)
 */
void samr21TrxSetIrqHandler(uint8_t irqType, void (*handler)(void));

/**
 *
 * Removes All IRQ-Handler, but does not disable the Interrupt itself
 *
 */
void samr21TrxRemoveAllHandler();

/**
 *
 * Enables an Interrupt on the AT86Rf233 (see samr21 datasheet 35.7 Interrupt Logic)
 *
 * @param[in]  irqType IRQ-Type to be enabled
 */
void samr21TrxEnableIrq(uint8_t irqType);

/**
 *
 * Disables an Interrupt on the AT86Rf233 (see samr21 datasheet 35.7 Interrupt Logic)
 *
 * @param[in]  irqType IRQ-Type to be disabled
 */
void samr21TrxDisableIrq(uint8_t irqType);

/**
 *
 * Starts a Channel Clear Assessment on the at86rf233
 * TRX must be in a RX State
 *
 */
void samr21TrxStartCca();

/**
 *
 * Starts a Energy Detection on the at86rf233
 * TRX must be in a RX State
 */
void samr21TrxStartEd();

/**
 *
 * Get the last measured RSSI Value
 * @returns the current RSSI in dBm
 * 
 */
int8_t samr21TrxGetLastRssiValue();

/**
 *
 * Changes the way the at86rf233 considers a channel as busy
 * See samr21 datasheet Table 37-22. CCA_MODE
 *
 * @param[in]  newCcaMode nmode
 */
void samr21RadioCtrlSetCCAMode(uint8_t newCcaMode);

/**
 *
 * Changes the threshold for when the at86rf233 considers a channel as busy
 * See samr21 datasheet 37.6.6.3 CCA_THRES
 *
 * @param[in]  threshold    desired threshold in dBm
 */
void samr21RadioCtrlSetCcaThreshold(int8_t a_threshold);

/**
 *
 * Get the threshold in dBm for when the at86rf233 considers a channel as busy
 *
 * @returns  current CCA threshold in dBm
 */
int8_t samr21RadioCtrlGetCcaThreshold(); 

#endif // _SAMR21_TRX_H_
