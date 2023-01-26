/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#include "samr21RadioTxHandler.h"

static TxStatus s_txStatus = TX_STATUS_IDLE;

static uint8_t s_txFramePsdu[IEEE_802_15_4_FRAME_SIZE];
static uint8_t s_txAckPsdu[IEEE_802_15_4_FRAME_SIZE];

static otRadioFrame s_txFrame = {.mPsdu = &s_txFramePsdu[1]};
static otRadioFrame s_txAckFrame = {.mPsdu = &s_txAckPsdu[1]};

static struct 
{
    uint8_t numCsmaBackoffs;
    uint8_t numTransmissionRetrys;
    uint8_t numPsduBytesUploaded;

}s_currentTransmission;

static struct 
{
    bool        securityHeaderPresent;
    bool        needsEncryption;

    uint8_t*    header;
    uint8_t     headerLen;

    uint8_t*    payload;
    uint8_t     payloadLen;

    uint8_t*    mic;
    uint8_t     micSize;

    uint8_t*     pendingAesResultBuffer;

    nonce[AES_BLOCK_SIZE];
    encryptionBlocks[(IEEE_802_15_4_FRAME_SIZE / AES_BLOCK_SIZE)][AES_BLOCK_SIZE];

    struct
    {
        uint8_t numEncryptionBlocks;
        uint8_t numProcessedEncryptionBlocks;
    } ctr;
    
    cbcBlock[AES_BLOCK_SIZE];

    struct
    {
        uint8_t currentBlockLen;
        uint8_t numProcessedHeaderBytes;
        uint8_t numProcessedPayloadBytes;
    } cbc;
    
}s_currentTransmissionSecurity;


volatile bool s_txHandlerActive = false;

bool samr21RadioTxBusy()
{
    return s_txHandlerActive;
}

bool samr21RadioTxAbort()
{
    if (samr21RadioTxBusy())
    {
        s_currentTransmission.numTransmissionRetrys = 0xFF;
        s_currentTransmission.numCsmaBackoffs = 0xFF;
    }
    return false;
}

otRadioFrame *samr21RadioTxGetOtBuffer()
{
    return &s_txFrame;
}

otRadioFrame *samr21RadioTxGetAckOtBuffer()
{
    return &s_txAckFrame;
}

void samr21RadioTxCleanup(bool success)
{
    samr21Timer4Stop();
    samr21RadioRemoveEventHandler();

    if (!success)
    {
        cb_samr21RadioTxFailed(&s_txFrame, s_txStatus);
    }
    else
    {
        cb_samr21RadioTxDone(&s_txFrame, &s_txAckFrame);
    }

    s_txHandlerActive = false;
}

static void samr21RadioTxStartCCA()
{
    s_txStatus = TX_STATUS_CCA;

    // Check if Transciver is in recive state
    if (g_trxStatus.bit.trxStatus != TRX_STATUS_RX_ON)
    {
        samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);
        while (g_trxStatus.bit.trxStatus != TRX_STATUS_RX_ON)
        {
            samr21TrxUpdateStatus();
        }
    }

    // Prepare CCA Measurment
    g_phyCcCcaReg.bit.ccaRequest = 1;

    // Start CCA Measurment
    samr21TrxWriteRegister(PHY_CC_CCA_REG, g_phyCcCcaReg.reg);

    // Reset local copy of ccaRequest Bit
    g_phyCcCcaReg.bit.ccaRequest = 0;

    // Start CCA TimeoutTimer
    samr21Timer4Set(TIMEOUT_CCA_us);
}

static void samr21RadioTxStart()
{
    s_currentTransmission.numCsmaBackoffs = 0;
    s_currentTransmission.numTransmissionRetrys = 0;

    samr21RadioTxStartCCA();
}

bool samr21RadioTxSetup()
{
    if (s_txHandlerActive)
    {
        return false;
    }

    samr21Timer4Stop();

    s_currentTransmission.numPsduBytesUploaded = 0;
    s_txFrame.mPsdu[-1] = s_txFrame.mLength;


    if(!s_txFrame.mInfo.mTxInfo.mIsARetx && !s_txFrame.mInfo.mTxInfo.mIsHeaderUpdated && otMacFrameIsSecurityEnabled(&s_txFrame)){
        __disable_irq();
        otMacFrameSetFrameCounter(&s_txFrame, g_macFrameCounter++);
        
        if(otMacFrameIsKeyIdMode1(&s_txFrame)){
            otMacFrameSetKeyId(&s_txFrame, g_currKeyId);
            s_txFrame.mInfo.mTxInfo.mAesKey = (const otMacKeyMaterial *)&g_currKey;
        }
        s_currentTransmissionSecurity.securityHeaderPresent = 1;
        __enable_irq();
    }
    
    s_currentTransmissionSecurity.securityHeaderPresent = 0;

    samr21RadioSetEventHandler(&samr21RadioTxEventHandler);
    s_txHandlerActive = true;

    // Set relevant IRQ Mask
    g_irqMask = (AT86RF233_REG_IRQ_MASK_t){
        .bit.pllLock = 0,
        .bit.pllUnlock = 0,
        .bit.rxStart = 1,
        .bit.trxEnd = 1,
        .bit.ccaEdDone = 1,
        .bit.addressMatch = 0,
        .bit.bufferUnderRun = 0,
        .bit.batteryLow = 0};
    samr21TrxWriteRegister(IRQ_MASK_REG, g_irqMask.reg);

    g_phyCcCcaReg.bit.channel = s_txFrame.mChannel;
    samr21TrxWriteRegister(PHY_CC_CCA_REG, g_phyCcCcaReg.reg);

    if (s_txFrame.mInfo.mTxInfo.mTxDelay)
    {
        uint32_t diffTime = samr21RtcGetTimestamp() - s_txFrame.mInfo.mTxInfo.mTxDelayBaseTime;

        if (diffTime > s_txFrame.mInfo.mTxInfo.mTxDelay)
        {
            goto txStart;
        }
        else
        {
            s_txStatus = TX_STATUS_WAIT_FOR_DELAYED_START;
            samr21Timer4Set(s_txFrame.mInfo.mTxInfo.mTxDelay - diffTime);
            return true;
        }
    }

txStart:
    samr21RadioTxStart();
    return true;
}

static void samr21RadioTxStartTransmission()
{

    samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_FORCE_PLL_ON);

    while (g_trxStatus.bit.trxStatus != TRX_STATUS_PLL_ON)
    {
        samr21TrxUpdateStatus();
    }

    // Start Trasmission in advance, there is some spare time while the Preamble and SFD is transmitted
    samr21TrxSetSLP_TR(true);


    uint8_t securityLevel = (s_txFrame.mInfo.mTxInfo.mIsSecurityProcessed ? 0b000 : otMacFrameGetSecurityLevel(&s_txFrame));

    s_currentTransmissionSecurity.needsEncryption = (securityLevel & 0b100 ? 1 : 0);

    switch (securityLevel & 0b11)
    {
    case 0b00:
        s_currentTransmissionSecurity.micSize = 0;
        break;
    
    case 0b01:
        s_currentTransmissionSecurity.micSize = 2;
        break;
    
    case 0b10:
        s_currentTransmissionSecurity.micSize = 4;
        break;
    
    case 0b11:
        s_currentTransmissionSecurity.micSize = 16;
        break;
    }

    //Generate Nonce and first EncryptionBlock if needed (TRX is busy with sending Preamble and SFD anyways)
    if (securityLevel)
    {
        otMacGenerateNonce(&s_txFrame, (uint8_t *)(&g_extAddr), s_currentTransmissionSecurity.nonce);
        samr21RadioAesKeySetup((uint8_t *)s_txFrame.mInfo.mTxInfo.mAesKey);

        if(s_currentTransmissionSecurity.needsEncryption){

            s_currentTransmissionSecurity.nonce[(AES_BLOCK_SIZE - 2)] = 0; // Always 0 cause max nonce-ctr == 8 (FRAME_SIZE / BLOCKSIZE)
            s_currentTransmissionSecurity.nonce[(AES_BLOCK_SIZE - 1)] = 1; // The first Nonce for Payload encryption has CTR value 1

            // Start generating the encrytion mask for the first block
            // The AES-Engine of the at86rf233 takes 21us per Block
            samr21RadioAesEcbEncrypt(s_currentTransmissionSecurity.nonce, NULL);

            s_currentTransmissionSecurity.ctr.numProcessedEncryptionBlocks = 0;
            s_currentTransmissionSecurity.pendingAesResultBuffer = 
                s_currentTransmissionSecurity.encryptionBlocks[
                    s_currentTransmissionSecurity.nonce[(AES_BLOCK_SIZE - 1)]
                ]
            ;
        }
    }

    if(!s_txFrame.mInfo.mTxInfo.mIsHeaderUpdated){

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
        // Add CSL-IE
        if ((g_cslPeriod > 0) && !s_txFrame.mInfo.mTxInfo.mIsARetx)
        {
            otMacFrameSetCslIe(&s_txFrame, (uint16_t)g_cslPeriod, samr21RadioCtrlCslGetPhase());
        }
#endif

#if OPENTHREAD_CONFIG_TIME_SYNC_ENABLE
        // Add Time-IE
        if (s_txFrame.mInfo.mTxInfo.mIeInfo->mTimeIeOffset != 0)
        {
            uint8_t *timeIe = s_txFrame.mPsdu + s_txFrame.mInfo.mTxInfo.mIeInfo->mTimeIeOffset;
            uint64_t time = samr21RtcGetTimestamp() + s_txFrame.mInfo.mTxInfo.mIeInfo->mNetworkTimeOffset;

            *timeIe = s_txFrame.mInfo.mTxInfo.mIeInfo->mTimeSyncSeq;

            *(++timeIe) = (uint8_t)(time & 0xff);
            for (uint8_t i = 1; i < sizeof(uint64_t); i++)
            {
                time = time >> 8;
                *(++timeIe) = (uint8_t)(time & 0xff);
            }
        }
#endif // OPENTHREAD_CONFIG_TIME_SYNC_ENABLE
    }

    // Transmission should have started here allrdy so trigger can be pulled low again
    samr21TrxSetSLP_TR(false);


    if (!s_currentTransmissionSecurity.needsEncryption && s_currentTransmissionSecurity.micSize == 0)
    {
        //If there is no Security processing needed, the frame can just be uploaded
        samr21RadioTxUploadAllRaw();
        return;
    }

    //Get some Markers for AES 
    s_currentTransmissionSecurity.header = otMacFrameGetHeader(&s_txFrame);
    s_currentTransmissionSecurity.payload = otMacFrameGetPayload(&s_txFrame);
    s_currentTransmissionSecurity.mic = otMacFrameGetFooter(&s_txFrame);

    s_currentTransmissionSecurity.headerLen =  s_currentTransmissionSecurity.header - s_txFrame.mPsdu;
    s_currentTransmissionSecurity.payloadLen = s_currentTransmissionSecurity.mic - s_currentTransmissionSecurity.payload;

    // Upload only the unencrypted Header Part first
    // This gives some Headroom to perform AES-CCM*
    samr21RadioTxUploadHeader();
    
    // Upload the Payload next. This takes time ~100us per 16Byte Block ( ~32us without Encryption ) 
    // The Trx ist still busy transmitting the so far uploaded Framebuffer
    samr21RadioTxUploadPayload();

    payloadLenght = otMacFrameGetPayloadLength(&s_txFrame);
    footerLenght = otMacFrameGetFooterLength(&s_txFrame);

    if(((securityLevel & 0b100) == 0x0)){
        goto calcCbc;
    }

    uint8_t numRequiredEncryptionBlocks = payloadLenght / AES_BLOCK_SIZE;

    if (payloadLenght % AES_BLOCK_SIZE)
    {
        numRequiredEncryptionBlocks++;
    }

    while (numRequiredEncryptionBlocks > numProcessedEncryptionBlocks)
    {

        // increase Nonce Counter
        s_txNonce[(AES_BLOCK_SIZE - 1)]++;
        // start uploading next AES Block, while reading the result of the last Block
        samr21RadioAesEcbEncrypt(s_txNonce, s_txEncryptionMask[1 + numProcessedEncryptionBlocks]);

        // While AES is busy, an encrypted payload-block can be uploaded to the FrameBuffer
        // This gives headroom for longer AES Operation
        samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, (uint8_t)(s_numBytesPdsuUploaded + 1)); // phyLen not in pdsuLen

        for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
        {
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
            samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
            // Upload (encrypted) payload to Framebuffer
            if (securityLevel >= 0b100)
            {
                samr21TrxSpiTransceiveByteRaw(
                    s_txFrame.mPsdu[s_numBytesPdsuUploaded++] ^ s_txEncryptionMask[1 + numProcessedEncryptionBlocks][i]);
            }
            else
            {
                samr21TrxSpiTransceiveByteRaw(
                    s_txFrame.mPsdu[s_numBytesPdsuUploaded++]);
            }
        }

        samr21TrxSpiCloseAccess();

        numProcessedEncryptionBlocks++;
    }

    // get Last Payload encryptionMaskBlock and start MIC encryptionMaskBlock
    s_txNonce[(AES_BLOCK_SIZE - 1)] = 0; ////The Nonce for MIC encryption has the CTR value 0
    samr21RadioAesEcbEncrypt(s_txNonce, s_txEncryptionMask[1 + numProcessedEncryptionBlocks]);

    // Upload Last Payload-Chunk to be encrypted befor MIC
    if (s_numBytesPdsuUploaded < (headerLength + payloadLenght))
    {

        samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, (uint8_t)(s_numBytesPdsuUploaded + 1));
        for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
        {

            // Upload remaining encrypted payload to Framebuffer
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
            samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
            if (securityLevel >= 0b100)
            {
                samr21TrxSpiTransceiveByteRaw(
                    s_txFrame.mPsdu[s_numBytesPdsuUploaded++] ^ s_txEncryptionMask[1 + numProcessedEncryptionBlocks][i]);
            }
            else
            {
                samr21TrxSpiTransceiveByteRaw(
                    s_txFrame.mPsdu[s_numBytesPdsuUploaded++]);
            }

            // Break if entire payload is uploaded
            if (s_numBytesPdsuUploaded >= payloadLenght + headerLength)
            {
                break;
            }
        }
        samr21TrxSpiCloseAccess();
    }

    /******************************Authenicate Payload and upload (encrypted) MIC (CBC-MAC)************************************/

calcCbc:
    if (securityLevel & 0b011)
    { // MIC requested
        pHeader = otMacFrameGetHeader(&s_txFrame);
        pFooter = otMacFrameGetFooter(&s_txFrame);

        uint8_t cbcBlockLength;
        uint8_t micSize = footerLenght - 2;

        // Form init Vektor
        s_cbcBlock[0] = (headerLength != 0 ? 0x01000000 : 0x0) | (((footerLenght - 2) >> 1) << 3) | 1; // L always 2
        s_cbcBlock[14] = 0;                                                                            // always 0 cause maxlenght is 127
        s_cbcBlock[15] = headerLength;
        memcpy(&s_cbcBlock[1], s_txNonce, 13);

        // start the first CBC-Round  (plain ECB, r21 datasheet 40.1.4.2 Cipher Block Chaining (CBC))
        // simultaneously retive the MIC encryptionMaskBlock (Nonce-ctr = 0)) started last in the AES-CTR section
        samr21RadioAesEcbEncrypt(s_cbcBlock, s_txEncryptionMask[0]);

        s_cbcBlock[0] ^= headerLength >> 8;
        s_cbcBlock[1] ^= headerLength >> 0;

        cbcBlockLength = 2;

        // process header bulk
        for (uint8_t i = 0; i < headerLength; i++)
        {
            if (cbcBlockLength == sizeof(s_cbcBlock))
            {
                while (samr21RadioAesBusy())
                    ;
                samr21RadioAesCbcEncrypt(s_cbcBlock, AES_BLOCK_SIZE, NULL);
                cbcBlockLength = 0;
            }

            s_cbcBlock[cbcBlockLength++] = pHeader[i];
        }

        // process header remainder
        if (cbcBlockLength != 0)
        {
            while (samr21RadioAesBusy())
                ;
            samr21RadioAesCbcEncrypt(s_cbcBlock, cbcBlockLength, NULL);
            cbcBlockLength = 0;
        }

        // process Payload bulk
        for (uint8_t i = 0; i < payloadLenght; i++)
        {
            if (cbcBlockLength == sizeof(s_cbcBlock))
            {
                while (samr21RadioAesBusy())
                    ;
                samr21RadioAesCbcEncrypt(s_cbcBlock, AES_BLOCK_SIZE, NULL);
                cbcBlockLength = 0;
            }

            s_cbcBlock[cbcBlockLength++] = pPayload[i];
        }

        // process Payload remainder
        if (cbcBlockLength != 0)
        {
            while (samr21RadioAesBusy())
                ;
            samr21RadioAesCbcEncrypt(s_cbcBlock, cbcBlockLength, NULL);
        }

        // Wait for the last CBC-Block
        while (samr21RadioAesBusy())
            ;
        samr21RadioAesReadBuffer(pFooter, 0, micSize);

        samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, (uint8_t)(s_numBytesPdsuUploaded + 1));

        for (uint8_t i = 0; micSize; i++)
        {

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
            samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
            if (securityLevel >= 0b100)
            {
                samr21TrxSpiTransceiveByteRaw(
                    s_txFrame.mPsdu[s_numBytesPdsuUploaded++] ^ s_txEncryptionMask[0][i]);
            }
            else
            {
                samr21TrxSpiTransceiveByteRaw(
                    s_txFrame.mPsdu[s_numBytesPdsuUploaded++]);
            }
        }
        samr21TrxSpiCloseAccess();
    }
    s_txStatus = TX_STATUS_SENDING_WAIT_TRX_END;
}


static void samr21RadioTxUploadAllRaw(){
        samr21TrxSpiStartAccess(AT86RF233_CMD_FRAMEBUFFER_WRITE, NULL);
        samr21TrxSpiTransceiveBytesRaw(&s_txFrame.mPsdu[-1], NULL, s_txFrame.mLength - 1); // -2 FCS +1 PhyLen
        samr21TrxSpiCloseAccess();

        s_currentTransmission.numPsduBytesUploaded = s_txFrame.mLength;
        s_txStatus = TX_STATUS_SENDING_WAIT_TRX_END;
}

static void samr21RadioTxUploadHeader(){
    samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, 0x00);
    samr21TrxSpiTransceiveBytesRaw(&s_txFrame.mPsdu[-1], NULL, s_currentTransmissionSecurity.headerLen + 1); // phyLen not in pdsuLen
    samr21TrxSpiCloseAccess();

    s_currentTransmission.numPsduBytesUploaded = s_currentTransmissionSecurity.headerLen;
    s_txStatus = TX_STATUS_SENDING_UPLOADED_HEADER;
}

static void samr21RadioTxUploadPayload(){
    s_currentTransmissionSecurity.ctr.numEncryptionBlocks = s_currentTransmissionSecurity.payloadLen / AES_BLOCK_SIZE ;
    s_currentTransmissionSecurity.ctr.numEncryptionBlocks += (s_currentTransmissionSecurity.payloadLen % AES_BLOCK_SIZE ? 1 : 0);

    while (s_currentTransmissionSecurity.ctr.numEncryptionBlocks > s_currentTransmissionSecurity.ctr.numProcessedEncryptionBlocks)
    {
        //Save some Time if no encryption is needed
        if(s_currentTransmissionSecurity.needsEncryption){
            // increase Nonce Counter
            s_currentTransmissionSecurity.nonce[(AES_BLOCK_SIZE - 1)]++;
            // start uploading next AES Block, while reading the result of the last Block
            samr21RadioAesEcbEncrypt(s_currentTransmissionSecurity.nonce, s_currentTransmissionSecurity.pendingAesResultBuffer);
            
            s_currentTransmissionSecurity.pendingAesResultBuffer = 
                s_currentTransmissionSecurity.encryptionBlocks[
                    s_currentTransmissionSecurity.nonce[(AES_BLOCK_SIZE - 1)]
                ]
            ;
        } 
        s_currentTransmissionSecurity.ctr.numProcessedEncryptionBlocks++;

        // While AES is busy, an encrypted payload-block can be uploaded 
        // This gives headroom for longer AES Operation, while simultaniusly topping of the FrameBuffer
        samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, (uint8_t)(s_currentTransmission.numPsduBytesUploaded + 1)); // phyLen not in pdsuLen

        for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
        {
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
            samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
            // Upload (encrypted) payload to Framebuffer
            if (s_currentTransmissionSecurity.needsEncryption)
            {
                samr21TrxSpiTransceiveByteRaw(
                    s_txFrame.mPsdu[s_currentTransmission.numPsduBytesUploaded++] 
                    ^ s_currentTransmissionSecurity.encryptionBlocks[s_currentTransmissionSecurity.ctr.numProcessedEncryptionBlocks][i]);
            }
            else
            {
                samr21TrxSpiTransceiveByteRaw(
                    s_txFrame.mPsdu[s_currentTransmission.numPsduBytesUploaded++]);
            }
        }

        samr21TrxSpiCloseAccess();
    }
    if(s_currentTransmissionSecurity.needsEncryption){
        // get Last Payload encryptionBlock and start MIC encryptionBlock
        s_currentTransmissionSecurity.nonce[(AES_BLOCK_SIZE - 1)] = 0; ////The Nonce for MIC encryption has the CTR value 0
        samr21RadioAesEcbEncrypt(s_currentTransmissionSecurity.nonce, s_currentTransmissionSecurity.pendingAesResultBuffer);
        s_currentTransmissionSecurity.pendingAesResultBuffer = 
            s_currentTransmissionSecurity.encryptionBlocks[
                s_currentTransmissionSecurity.nonce[(AES_BLOCK_SIZE - 1)]
            ]
        ;
    }
    s_currentTransmissionSecurity.ctr.numProcessedEncryptionBlocks++;

    // Upload Last Payload-Chunk to be encrypted befor MIC
    if (s_currentTransmission.numPsduBytesUploaded < (s_currentTransmissionSecurity.headerLen + s_currentTransmissionSecurity.payloadLen))
    {
        samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, (uint8_t)(s_currentTransmission.numPsduBytesUploaded + 1));

        for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
        {

            // Upload remaining to Framebuffer
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
            samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
            if (s_currentTransmissionSecurity.needsEncryption)
            {
                samr21TrxSpiTransceiveByteRaw(
                    s_txFrame.mPsdu[s_currentTransmission.numPsduBytesUploaded++] 
                    ^ s_currentTransmissionSecurity.encryptionBlocks[s_currentTransmissionSecurity.ctr.numProcessedEncryptionBlocks][i]);
            }
            else
            {
                samr21TrxSpiTransceiveByteRaw(
                    s_txFrame.mPsdu[s_currentTransmission.numPsduBytesUploaded++]);
            }

            // Break if entire payload is uploaded
            if (s_currentTransmission.numPsduBytesUploaded >= (s_currentTransmissionSecurity.headerLen + s_currentTransmissionSecurity.payloadLen))
            {
                break;
            }
        }
        samr21TrxSpiCloseAccess();
    }
    s_txStatus = TX_STATUS_SENDING_UPLOADED_HEADER;
}

//this function gets triggered once by the main handler and after by a Timer.
//this is because there is nothing to do while the AES-Engine is busy
static void samr21RadioTxCalcMic(){
    
    //Only executed on first Round
    if(s_txStatus != TX_STATUS_SENDING_BUSY_MIC){
        // Form init Vektor
        s_currentTransmissionSecurity.cbcBlock[0] = 
            (s_currentTransmissionSecurity.headerLen != 0 ? 0x01000000 : 0x0) 
            | (((s_currentTransmissionSecurity.micSize) >> 1) << 3) | 1; // L always 2
        s_currentTransmissionSecurity.cbcBlock[14] = 0; // always 0 cause maxlenght is 127
        s_currentTransmissionSecurity.cbcBlock[15] = s_currentTransmissionSecurity.headerLen;
        memcpy(&s_currentTransmissionSecurity.cbcBlock[1], s_currentTransmissionSecurity.nonce, 13);

        // start the first CBC-Round  (plain ECB, r21 datasheet 40.1.4.2 Cipher Block Chaining (CBC))
        // simultaneously retive the MIC encryptionMaskBlock (Nonce-ctr = 0)) started last in the AES-CTR section
        samr21RadioAesEcbEncrypt(s_currentTransmissionSecurity.cbcBlock, s_currentTransmissionSecurity.pendingAesResultBuffer);
        samr21Timer4Set(21);

        // Only the result of the last CBC Round is of interest
        s_currentTransmissionSecurity.pendingAesResultBuffer = NULL;

        s_currentTransmissionSecurity.cbcBlock[0]^= s_currentTransmissionSecurity.headerLen >> 8;
        s_currentTransmissionSecurity.cbcBlock[1]^= s_currentTransmissionSecurity.headerLen >> 0;

        s_currentTransmissionSecurity.cbc.currentBlockLen = 2;
        s_currentTransmissionSecurity.cbc.numProcessedHeaderBytes = 0;
        s_currentTransmissionSecurity.cbc.numProcessedPayloadBytes = 0;

        goto fillNextBlock;
    }

    if (s_currentTransmissionSecurity.cbc.numProcessedHeaderBytes < s_currentTransmissionSecurity.headerLen)


fillNextBlock:

    if(s_currentTransmissionSecurity.cbc.numProcessedHeaderBytes < s_currentTransmissionSecurity.headerLen){
        while (s_currentTransmissionSecurity.cbc.currentBlockLen < AES_BLOCK_SIZE)
        {
            s_currentTransmissionSecurity.cbcBlock[s_currentTransmissionSecurity.cbc.currentBlockLen++] =
                (s_currentTransmissionSecurity.cbc.numProcessedHeaderBytes < s_currentTransmissionSecurity.headerLen ?
                    s_currentTransmissionSecurity.header[s_currentTransmissionSecurity.cbc.numProcessedHeaderBytes++]
                    : 0x00
                ) 
            ;
        }
        return;
    }

    if(s_currentTransmissionSecurity.cbc.numProcessedPayloadBytes < s_currentTransmissionSecurity.payloadLen){
        while (s_currentTransmissionSecurity.cbc.currentBlockLen < AES_BLOCK_SIZE)
        {
            s_currentTransmissionSecurity.cbcBlock[s_currentTransmissionSecurity.cbc.currentBlockLen++] =
                (s_currentTransmissionSecurity.cbc.numProcessedPayloadBytes < s_currentTransmissionSecurity.payloadLen ?
                    s_currentTransmissionSecurity.payload[s_currentTransmissionSecurity.cbc.numProcessedPayloadBytes++]
                    : 0x00
                ) 
            ;
        }
        return;
    }
}

static void samr21RadioTxEvalCCA()
{
    // Clear CCA TimeoutTimer
    samr21Timer4Stop();

    if (g_trxStatus.bit.ccaStatus)
    {
        samr21RadioTxStartTransmission();
        return;
    }

    if (
        s_numCsmaBackoffs < s_txFrame.mInfo.mTxInfo.mMaxCsmaBackoffs && s_txFrame.mInfo.mTxInfo.mCsmaCaEnabled)
    {
        uint32_t backoffTime_us =
            ((samr21TrxGetRandomByte() >> (8 - (g_csmaBackoffExponentMin + s_numCsmaBackoffs > g_csmaBackoffExponentMax ? g_csmaBackoffExponentMax : g_csmaBackoffExponentMin + s_numCsmaBackoffs))) - 1) * (20 * IEEE_802_15_4_24GHZ_TIME_PER_OCTET_us);
        s_numCsmaBackoffs++;

        s_txStatus = TX_STATUS_CSMA_BACKOFF;
        samr21Timer4Set(backoffTime_us);
        return;
    }

    samr21RadioTxCleanup(false);
}

static void samr21RadioTxPrepareAckReception()
{
    samr21Timer4Stop();
    if (otMacFrameIsAckRequested(&s_txFrame))
    {
        s_txStatus = TX_STATUS_WAIT_FOR_ACK;
        samr21Timer4Set(g_txAckTimeout_us);
        return;
    }
    s_txStatus = TX_STATUS_DONE;
    samr21RadioTxCleanup(true);
}

static void samr21RadioTxAckReceptionStarted()
{
    samr21Timer4Stop();
    s_txAckFrame.mInfo.mRxInfo.mTimestamp = samr21RtcGetTimestamp();
    s_txStatus = TX_STATUS_RECIVING_ACK;

    // Timeout for Ack Reception
    samr21Timer4Set(IEEE_802_15_4_24GHZ_TIME_PER_OCTET_us * IEEE_802_15_4_FRAME_SIZE);
}

static void samr21RadioTxAckReceptionDone()
{
    // Clear Timeout for Ack Reception
    samr21Timer4Stop();

    samr21TrxSpiStartAccess(AT86RF233_CMD_FRAMEBUFFER_READ, 0);

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
    s_txAckFrame.mLength = samr21TrxSpiReadByteRaw();

    if (s_txAckFrame.mLength > IEEE_802_15_4_PDSU_SIZE)
    {
        samr21TrxSpiCloseAccess();
        goto ackInvalid;
    }

    // Download PDSU
    samr21TrxSpiTransceiveBytesRaw(NULL, s_txAckFrame.mPsdu, s_txAckFrame.mLength);

// Dowload LQI, RSSI and CRC Check (r21 Datasheet 35.3.2 -  Frame Buffer Access Mode)
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
    s_txAckFrame.mInfo.mRxInfo.mLqi = samr21TrxSpiReadByteRaw();

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
    s_txAckFrame.mInfo.mRxInfo.mRssi = AT86RF233_RSSI_BASE_VAL + samr21TrxSpiReadByteRaw();

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
    AT86RF233_REG_RX_STATUS_t rxStatus = (AT86RF233_REG_RX_STATUS_t)samr21TrxSpiReadByteRaw();

    samr21TrxSpiCloseAccess();

    if (
        otMacFrameIsAck(&s_txAckFrame) && rxStatus.bit.crcValid && otMacFrameGetSequence(&s_txFrame) == otMacFrameGetSequence(&s_txAckFrame))
    {
        samr21RadioTxCleanup(true);
        return;
    }

ackInvalid:

    if (s_numTransmissionRetrys < s_txFrame.mInfo.mTxInfo.mMaxFrameRetries)
    {
        s_numTransmissionRetrys++;
        s_numCsmaBackoffs = 0;
        samr21RadioTxStartCCA();
        return;
    }

    samr21RadioTxCleanup(false);
}

void samr21RadioTxEventHandler(IrqEvent event)
{
    switch (event)
    {
    case TIMER_EVENT_4_TRIGGER:

        if (s_txStatus == TX_STATUS_WAIT_FOR_DELAYED_START)
        {
            samr21RadioTxStart();
            return;
        }

        if (s_txStatus == TX_STATUS_CSMA_BACKOFF)
        {
            samr21RadioTxStartCCA();
            return;
        }

        // Timeout Something went wrong
        samr21RadioTxCleanup(false);
        return;

    case TRX_EVENT_CCA_ED_DONE:
        // Scan Done
        samr21RadioTxEvalCCA();
        return;

    case TRX_EVENT_TRX_END:
        if (s_txStatus == TX_STATUS_SENDING_WAIT_TRX_END)
        {
            samr21RadioTxPrepareAckReception();
            return;
        }
        else if (s_txStatus == TX_STATUS_RECIVING_ACK)
        {
            samr21RadioTxAckReceptionDone();
            return;
        }

    case TRX_EVENT_RX_START:
        if (s_txStatus == TX_STATUS_WAIT_FOR_ACK)
        {
            samr21RadioTxAckReceptionStarted();
            return;
        }

    default:
        return;
    }
}
