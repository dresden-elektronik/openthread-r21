// Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21RadioRxHandler.h"

static RxBuffer s_rxBuffer[NUM_SAMR21_RX_BUFFER];
static uint8_t s_activeRxBuffer = 0;

volatile bool s_rxSlottedActive = false;
uint32_t s_slottedListeningDuration_us;
uint8_t s_slottedListeningChannel;

volatile bool s_rxAbort = false;
volatile bool s_rxHandlerActive = false;

static 

static uint8_t s_ackEncryptionMask[(IEEE_802_15_4_FRAME_SIZE / AES_BLOCK_SIZE)][AES_BLOCK_SIZE];

bool samr21RadioRxBusy()
{
    return s_rxHandlerActive;
}

bool samr21RadioRxAbort()
{
    if (samr21RadioRxBusy())
    {
        s_rxAbort = true;

        while (s_rxHandlerActive)
            ;
        return true;
    }
    return false;
}

void samr21RadioRxResetBuffer()
{
    for (uint16_t i = 0; i < NUM_SAMR21_RX_BUFFER; i++)
    {
        s_rxBuffer[i].status = RX_STATUS_IDLE;
        s_rxBuffer[i].otFrame.mPsdu = &s_rxBuffer[i].rxFramePsdu[1];
    }

    s_activeRxBuffer = 0;
}

RxBuffer *samr21RadioRxGetPendingRxBuffer()
{
    for (uint16_t i = 0; i < NUM_SAMR21_RX_BUFFER; i++)
    {
        if (s_rxBuffer[i].status >= RX_STATUS_DONE)
        {
            return &s_rxBuffer[i];
        }
    }
    return NULL;
}

bool samr21RadioRxSetup(uint8_t channel, uint32_t duration, uint32_t startTime)
{
    if (duration)
    {
        s_slottedListeningChannel = channel;
        s_slottedListeningDuration_us = duration;
        samr21RtcSetWakeUpTimestamp(startTime);
        return;
    }

    samr21RadioRxStart(channel);
    return true;
}

void samr21RadioRxStart(uint8_t channel)
{

    // Set Timer for listening duration
    if (s_slottedListeningDuration_us)
    {
        s_rxSlottedActive = true;
        samr21RtcSetWakeUpTimestamp(samr21RtcGetTimestamp() + s_slottedListeningDuration_us);
    }

    // Abort the current Transmisson (prevent retrys)
    samr21RadioTxAbort();

    // Check if theres already an ongoing Reception
    if (s_rxHandlerActive)
    {
        if (channel == g_phyCcCcaReg.bit.channel)
        {
            return; // TRX is allrdy setup for reception on the desired channel
        }
        samr21RadioRxAbort();
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

    // Set Channel of TRX
    g_phyCcCcaReg.bit.channel = channel;
    samr21TrxWriteRegister(PHY_CC_CCA_REG, g_phyCcCcaReg.reg);

    // Prepare RX-Buffer
    if (s_rxBuffer[s_activeRxBuffer].status != RX_STATUS_IDLE)
    {
        __disable_irq();
        uint16_t nextBuffer = (s_activeRxBuffer + 1) % NUM_SAMR21_RX_BUFFER;
        s_rxBuffer[nextBuffer].status = RX_STATUS_IDLE;
        s_activeRxBuffer = nextBuffer;
        __enable_irq();
    }

    // Check and enforce that the Transciver is in recive state
    if (g_trxStatus.bit.trxStatus != TRX_STATUS_RX_ON)
    {
        samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);
        while (g_trxStatus.bit.trxStatus != TRX_STATUS_RX_ON)
        {
            samr21TrxUpdateStatus();
        }
    }

    void samr21RadioSetEventHandler(&samr21RadioRxEventHandler);

    s_rxHandlerActive = true;
    s_rxAbort = false;
}

void samr21RadioRxReceptionStarted()
{
    s_rxBuffer[s_activeRxBuffer].status = RX_STATUS_RECIVING_FCF;
    s_rxBuffer[s_activeRxBuffer].otFrame.mInfo.mRxInfo.mTimestamp =
        samr21RtcGesamr21RtcGetTimestamp();
    s_rxBuffer[s_activeRxBuffer].otFrame.mChannel = g_phyCcCcaReg.bit.channel;
    samr21Timer4Set(RX_BACKOFF_BEFORE_FIRST_FRAMEBUFFER_ACCESS);
}

void samr21RadioRxDownloadFCF()
{
    samr21TrxSpiStartAccess(AT86RF233_CMD_FRAMEBUFFER_READ, 0);

    RxBuffer *buffer = &s_rxBuffer[s_activeRxBuffer];

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
    buffer->otFrame.mLength = samr21TrxSpiReadByteRaw();
    uint8_t numDownloadedPdsuBytes = 0;

    if (buffer->otFrame.mLength > IEEE_802_15_4_PDSU_SIZE)
    {
        samr21TrxSpiStopAccess();
        samr21RadioRxCleanup(false);
        return;
    }

    // Dowload FCS + DSN Field First
    while (numDownloadedPdsuBytes < IEEE_802_15_4_STATIC_HEADER_SIZE)
    {
        uint32_t timeout = 0xFFFF;
        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
        samr21delaySysTick(CPU_WAIT_CYCLES_FOR_FRAME_BUFFER_EMPTY_FLAG);
        while ((PORT->Group[1].IN.reg & PORT_PB00) && timeout)
        {
            timeout--;
        }

        if (timeout)
        {
            buffer->otFrame.mPsdu[numDownloadedPdsuBytes++] =
                samr21TrxSpiReadByteRaw();
            ;
        }
        else
        {
            samr21RadioRxCleanup(false);
            return;
        }
    }
    samr21TrxSpiCloseAccess();

    uint8_t relevantOctets = 0;

    otMacAddress srcAddr;
    otMacAddress dstAddr;

    otMacFrameGetSrcAddr(&(buffer->otFrame), &srcAddr);
    otMacFrameGetDstAddr(&(buffer->otFrame), &dstAddr);

    if (dstAddr.mType != OT_MAC_ADDRESS_TYPE_NONE)
    {
        relevantOctets +=
            (dstAddr.mType == OT_MAC_ADDRESS_TYPE_EXTENDED ? sizeof(uint64_t)
                                                           : sizeof(uint16_t));
    }

    if (srcAddr.mType == OT_MAC_ADDRESS_TYPE_EXTENDED)
    {
        relevantOctets += sizeof(uint64_t);
        if (utilsSoftSrcMatchExtFindEntry(srcAddr.mAddress.mExtAddress.m8) >= 0)
        {
            buffer->framePending = true;
        }
        else
        {
            buffer->framePending = false;
        }
    }

    if (srcAddr.mType == OT_MAC_ADDRESS_TYPE_SHORT)
    {
        relevantOctets += sizeof(uint16_t);
        if (utilsSoftSrcMatchShortFindEntry(srcAddr.mAddress.mShortAddress) >= 0)
        {
            buffer->framePending = true;
        }
        else
        {
            buffer->framePending = false;
        }
    }

    relevantOctets += 
        (otMacFrameIsSrcPanIdPresent(&(buffer->otFrame)) ? sizeof(uint16_t) : 0) 
    ;

    relevantOctets += 
        (otMacFrameIsDstPanIdPresent(&(buffer->otFrame)) ? sizeof(uint16_t) : 0)
    ;

    if (relevantOctets)
    {

        uint32_t nextAction_us =
            (relevantOctets * IEEE_802_15_4_24GHZ_TIME_PER_OCTET_us) - (AT86RF233_SPI_INIT_TIME_FRAMEBUFFER_us + RX_FRAMEBUFFER_ACCESS_HEADROOM_us) - ((relevantOctets + IEEE_802_15_4_STATIC_HEADER_SIZE) * AT86RF233_SPI_TIME_PER_BYTE_us);

        buffer->neededPdsuSizeForNextAction = relevantOctets + IEEE_802_15_4_STATIC_HEADER_SIZE;
        buffer->status = RX_STATUS_RECIVING_ADDR_FIELD;

        if (nextAction_us > 0xFFFF || nextAction_us < RX_FRAMEBUFFER_ACCESS_HEADROOM_us)
        {
            samr21RadioRxDownloadAddrField();
            return;
        }

        samr21Timer4Set(nextAction_us);
        return;
    }

    uint32_t nextAction_us =
        ((buffer->otFrame.mLength - numDownloadedPdsuBytes) * IEEE_802_15_4_24GHZ_TIME_PER_OCTET_us) - (AT86RF233_SPI_INIT_TIME_FRAMEBUFFER_us + RX_FRAMEBUFFER_ACCESS_HEADROOM_us) - ((buffer->otFrame.mLength + AT86RF233_FRAMEBUFFER_MISC_SIZE) * AT86RF233_SPI_TIME_PER_BYTE_us);

    buffer->neededPdsuSizeForNextAction = buffer->otFrame.mLength;
    buffer->status = RX_STATUS_RECIVING_REMAINING;

    if (nextAction_us > 0xFFFF || nextAction_us < RX_FRAMEBUFFER_ACCESS_HEADROOM_us)
    {
        samr21RadioRxDownloadRemaining();
        return;
    }

    samr21Timer4Set(nextAction_us);
}

void samr21RadioRxDownloadAddrField()
{
    samr21TrxSpiStartAccess(AT86RF233_CMD_FRAMEBUFFER_READ, 0);

    RxBuffer *buffer = &s_rxBuffer[s_activeRxBuffer];

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
    buffer->otFrame.mLength = samr21TrxSpiReadByteRaw();
    uint8_t numDownloadedPdsuBytes = 0;

    if (buffer->otFrame.mLength > IEEE_802_15_4_PDSU_SIZE)
    {
        samr21TrxSpiStopAccess();
        samr21RadioRxCleanup(false);
        return;
    }

    // Dowload FCF+ADDR Field First
    while (numDownloadedPdsuBytes < buffer->neededPdsuSizeForNextAction)
    {
        uint32_t timeout = 0xFFFF;
        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
        samr21delaySysTick(CPU_WAIT_CYCLES_FOR_FRAME_BUFFER_EMPTY_FLAG);
        while ((PORT->Group[1].IN.reg & PORT_PB00) && timeout)
        {
            timeout--;
        }

        if (timeout)
        {
            buffer->otFrame.mPsdu[numDownloadedPdsuBytes++] =
                samr21TrxSpiReadByteRaw();
            ;
        }
        else
        {
            samr21RadioRxCleanup(false);
            return;
        }
    }
    samr21TrxSpiCloseAccess();

    if (!otMacFrameDoesAddrMatch(
            &(buffer->otFrame),
            g_panId,
            g_shortAddr,
            (const otExtAddress *)(&g_extAddr)))
    {
        samr21RadioRxCleanup(false);
        return;
    }

    uint32_t nextAction_us =
        ((buffer->otFrame.mLength - numDownloadedPdsuBytes) * IEEE_802_15_4_24GHZ_TIME_PER_OCTET_us) - (AT86RF233_SPI_INIT_TIME_FRAMEBUFFER_us + RX_FRAMEBUFFER_ACCESS_HEADROOM_us) - ((buffer->otFrame.mLength + AT86RF233_FRAMEBUFFER_MISC_SIZE) * AT86RF233_SPI_TIME_PER_BYTE_us);

    buffer->neededPdsuSizeForNextAction = buffer->otFrame.mLength;
    buffer->status = RX_STATUS_RECIVING_REMAINING;

    if (nextAction_us > 0xFFFF || nextAction_us < RX_FRAMEBUFFER_ACCESS_HEADROOM_us)
    {
        samr21RadioRxDownloadRemaining();
        return;
    }

    samr21Timer4Set(nextAction_us);
}

void samr21RadioRxDownloadRemaining()
{
    samr21TrxSpiStartAccess(AT86RF233_CMD_FRAMEBUFFER_READ, 0);

    RxBuffer *buffer = &s_rxBuffer[s_activeRxBuffer];

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
    buffer->otFrame.mLength = samr21TrxSpiReadByteRaw();
    uint8_t numDownloadedPdsuBytes = 0;

    if (buffer->otFrame.mLength > IEEE_802_15_4_PDSU_SIZE)
    {
        samr21TrxSpiStopAccess();
        samr21RadioRxCleanup(false);
        return;
    }

    // Dowload FCF+ADDR Field First
    while (numDownloadedPdsuBytes < buffer->otFrame.mLength)
    {
        uint32_t timeout = 0xFFFF;
        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
        samr21delaySysTick(CPU_WAIT_CYCLES_FOR_FRAME_BUFFER_EMPTY_FLAG);
        while ((PORT->Group[1].IN.reg & PORT_PB00) && timeout)
        {
            timeout--;
        }

        if (timeout)
        {
            buffer->otFrame.mPsdu[numDownloadedPdsuBytes++] =
                samr21TrxSpiReadByteRaw();
            ;
        }
        else
        {
            samr21RadioRxCleanup(false);
            return;
        }
    }
    // Dowload LQI, RSSI and CRC Check (r21 Datasheet 35.3.2 -  Frame Buffer Access Mode)
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
    buffer->otFrame.mInfo.mRxInfo.mLqi = samr21TrxSpiReadByteRaw();

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
    buffer->otFrame.mInfo.mRxInfo.mRssi = AT86RF233_RSSI_BASE_VAL + samr21TrxSpiReadByteRaw();

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
    AT86RF233_REG_RX_STATUS_t rxStatus = (AT86RF233_REG_RX_STATUS_t)samr21TrxSpiReadByteRaw();

    samr21TrxSpiCloseAccess();

    if (!rxStatus.bit.crcValid)
    {
        samr21RadioRxCleanup(false);
        return;
    }

    if (otMacFrameIsAckRequested(&(buffer->otFrame)))
    {

        // Prep TRX
        samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_FORCE_PLL_ON);

        if (otMacFrameIsVersion2015(&(buffer->otFrame)))
        {
            samr21RadioRxSendEnhAck();
            return;
        }

        samr21RadioRxSendAck();
    }
}

void samr21RadioRxSendEnhAck()
{
    while (g_trxStatus.bit.trxStatus != TRX_STATUS_PLL_ON)
    {
        samr21TrxUpdateStatus();
    }

    // Start Trasmission in advance, there is some spare time while the Preamble and SFD is transmitted
    samr21TrxSetSLP_TR(true);
    
    RxBuffer *buffer = &s_rxBuffer[s_activeRxBuffer];
    buffer->status = RX_STATUS_SENDING_ENH_ACK;
    // Timout-Timer
    samr21Timer4Set((IEEE_802_15_4_FRAME_SIZE * 2) * IEEE_802_15_4_24GHZ_TIME_PER_OCTET_us);


    otRadioFrame ackFrame = {.mPsdu = &(buffer->rxAckPsdu[1])};

    uint8_t ieData[IEEE_802_15_4_PDSU_SIZE];
    uint8_t ieDataLen = 0;

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
    if (g_cslPeriod > 0)
    {
        ieData[ieDataLen++] = CSL_IE_HEADER_BYTES_LO;
        ieData[ieDataLen++] = CSL_IE_HEADER_BYTES_HI;

        uint16_t cslPhase = samr21RadioCtrlCslGetPhase();
        memcpy(&ieData[ieDataLen], &cslPhase, sizeof(uint16_t));
        ieDataLen += sizeof(uint16_t);

        memcpy(&ieData[ieDataLen], &g_cslPeriod, sizeof(uint16_t));
        ieDataLen += sizeof(uint16_t);
    }
#endif

#ifdef OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
    otMacAddress macAddress;
    otMacFrameGetSrcAddr(&(buffer->otFrame), &macAddress);
    uint8_t linkMetricsDataLen = otLinkMetricsEnhAckGenData(
        &macAddress, 
        buffer->otFrame.mInfo.mRxInfo.mLqi,
        buffer->otFrame.mInfo.mRxInfo.mRssi
        &ieData[ieDataLen+4] //IE VENDOR HEADER LENGTH
    );

    if (linkMetricsDataLen)
    {    
        ieData[ieDataLen++] =  ENH_ACK_PROBING_IE; 
        
        ieData[ieDataLen++] =  ( VENDOR_OUI_THREAD_COMPANY_ID & 0x0000FF ) >> 0; 
        ieData[ieDataLen++] =  ( VENDOR_OUI_THREAD_COMPANY_ID & 0x00FF00 ) >> 8; 
        ieData[ieDataLen++] =  ( VENDOR_OUI_THREAD_COMPANY_ID & 0xFF0000 ) >> 16; 

        ieDataLen += linkMetricsDataLen;
    }
#endif

    otMacFrameGenerateEnhAck(
        &(buffer->otFrame),
        buffer->framePending,
        ieData,
        ieDataLen,
        &ackFrame);

    ackFrame.mPsdu[-1] = ackFrame.mLength;

    uint8_t securityLevel = otMacFrameGetSecurityLevel(&ackFrame);

    if(!securityLevel){
        samr21TrxSpiStartAccess(AT86RF233_CMD_FRAMEBUFFER_WRITE, NULL);
        samr21TrxSetSLP_TR(false);
        samr21TrxSpiTransceiveBytesRaw(ackFrame.mPsdu[-1], NULL,ackFrame.mLength-1); // -2 FCS +1 PhyLen
        samr21TrxSpiCloseSpiAccess();
        return;
    }
  
    uint16_t payloadLenght;
    uint16_t headerLength;
    uint16_t footerLenght;

    uint8_t *pHeader;
    uint8_t *pPayload;
    uint8_t *pFooter;

    uint16_t numProcessedEncryptionBlocks = 0;
    uint16_t numBytesPdsuUploaded; 
    uint16_t processedAuthenicationHeaderBlocks = 0;
    uint16_t processedAuthenicationPayloadBlocks = 0;
    
    uint8_t ackNonce[AES_BLOCK_SIZE];
    uint8_t cbcBlock[AES_BLOCK_SIZE];
    uint8_t ackEncryptionMask[IEEE_802_15_4_FRAME_SIZE];

    uint8_t keyId = otMacFrameGetKeyId(&ackFrame);

    otMacGenerateNonce(&ackFrame, (uint8_t *)(&g_extAddr), ackNonce);

    if (keyId == g_currKeyId)
    {
        samr21RadioAesKeySetup(g_currKey);
    }
    else if (keyId == (g_currKeyId - 1))
    {
        samr21RadioAesKeySetup(g_prevKey);
    }
    else if (keyId == (g_currKeyId + 1))
    {
        samr21RadioAesKeySetup(g_nextKey);
    }
    else
    {
        samr21RadioRxCleanup(false);
        return;
    }

    ackNonce[(AES_BLOCK_SIZE - 2)] = 0; // Always 0 cause max nonce-ctr == 8 (FRAME_SIZE / BLOCKSIZE)
    ackNonce[(AES_BLOCK_SIZE - 1)] = 1; // The first Nonce for Payload encryption has CTR value 1

    // Start generating the encrytion mask for the first block
    // The AES-Engine of the at86rf233 takes 21us per Block
    samr21RadioAesEcbEncrypt(ackNonce, NULL);
    numProcessedEncryptionBlocks = 0;

    // Needed for Calculation of Unsecured Header size
    pPayload = otMacFrameGetPayload(&ackFrame);
    headerLength = (uint32_t)pPayload - (uint32_t)ackFrame.mPsdu;
    otMacFrameSetFrameCounter(&ackFrame, g_macFrameCounter++);


    samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, 0x00);
    samr21TrxSpiTransceiveBytesRaw(&ackFrame.mPsdu[-1], NULL, headerLength + 1); // phyLen not in pdsuLen
    numBytesPdsuUploaded = headerLength;
    samr21TrxSpiCloseSpiAccess();


    /******************************Encrypt and Upload Payload (AES CTR)************************************/

    payloadLenght = otMacFrameGetPayloadLength(&ackFrame);
    footerLenght = otMacFrameGetFooterLength(&ackFrame);

    uint8_t numRequiredEncryptionBlocks = payloadLenght / AES_BLOCK_SIZE;

    if (payloadLenght % AES_BLOCK_SIZE)
    {
        numRequiredEncryptionBlocks++;
    }

    while (numRequiredEncryptionBlocks > numProcessedEncryptionBlocks)
    {

        // increase Nonce Counter
        ackNonce[(AES_BLOCK_SIZE - 1)]++;
        // start uploading next AES Block, while reading the result of the last Block
        samr21RadioAesEcbEncrypt(ackNonce, ackEncryptionMask[1 + numProcessedEncryptionBlocks]);

        // While AES is busy, an encrypted payload-block can be uploaded to the FrameBuffer
        // This gives headroom for longer AES Operation
        samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, (uint8_t)(numBytesPdsuUploaded + 1)); // phyLen not in pdsuLen

        for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
        {
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
            samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
            // Upload (encrypted) payload to Framebuffer
            if (securityLevel >= 0b100)
            {
                samr21TrxSpiTransceiveByteRaw(
                    ackFrame.mPsdu[numBytesPdsuUploaded++] ^ ackEncryptionMask[1 + numProcessedEncryptionBlocks][i]);
            }
            else
            {
                samr21TrxSpiTransceiveByteRaw(
                    ackFrame.mPsdu[numBytesPdsuUploaded++]);
            }
        }

        samr21TrxSpiCloseAccess();

        numProcessedEncryptionBlocks++;
    }

    // get Last Payload encryptionMaskBlock and start MIC encryptionMaskBlock
    ackNonce[(AES_BLOCK_SIZE - 1)] = 0; ////The Nonce for MIC encryption has the CTR value 0
    samr21RadioAesEcbEncrypt(ackNonce, ackEncryptionMask[1 + numProcessedEncryptionBlocks]);

    // Upload Last Payload-Chunk to be encrypted befor MIC
    if (numBytesPdsuUploaded < (headerLength + payloadLenght))
    {

        samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, (uint8_t)(numBytesPdsuUploaded + 1));
        for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
        {

            // Upload remaining encrypted payload to Framebuffer
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
            samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
            if (securityLevel >= 0b100)
            {
                samr21TrxSpiTransceiveByteRaw(
                    ackFrame.mPsdu[numBytesPdsuUploaded++] ^ ackEncryptionMask[1 + numProcessedEncryptionBlocks][i]);
            }
            else
            {
                samr21TrxSpiTransceiveByteRaw(
                    ackFrame.mPsdu[numBytesPdsuUploaded++]);
            }

            // Break if entire payload is uploaded
            if (numBytesPdsuUploaded >= payloadLenght + headerLength)
            {
                break;
            }
        }
        samr21TrxSpiCloseAccess();
    }

    /******************************Authenicate Payload and upload (encrypted) MIC (CBC-MAC)************************************/
    if (footerLenght > 2)
    { // MIC requested
        pHeader = otMacFrameGetHeader(&ackFrame);
        pFooter = otMacFrameGetFooter(&ackFrame);

        uint8_t cbcBlockLength;
        uint8_t micSize = footerLenght - 2;

        // Form init Vektor
        cbcBlock[0] = (headerLength != 0 ? 0x01000000 : 0x0) | (((footerLenght - 2) >> 1) << 3) | 1; // L always 2
        cbcBlock[14] = 0;                                                                            // always 0 cause maxlenght is 127
        cbcBlock[15] = headerLength;
        memcpy(&cbcBlock[1], ackNonce, 13);

        // start the first CBC-Round  (plain ECB, r21 datasheet 40.1.4.2 Cipher Block Chaining (CBC))
        // simultaneously retive the MIC encryptionMaskBlock (Nonce-ctr = 0)) started last in the AES-CTR section
        samr21RadioAesEcbEncrypt(cbcBlock, ackEncryptionMask[0]);

        cbcBlock[0] ^= headerLength >> 8;
        cbcBlock[1] ^= headerLength >> 0;

        cbcBlockLength = 2;

        // process header bulk
        for (uint8_t i = 0; i < headerLength; i++)
        {
            if (cbcBlockLength == sizeof(cbcBlock))
            {
                while (samr21RadioAesBusy())
                    ;
                samr21RadioAesCbcEncrypt(cbcBlock, AES_BLOCK_SIZE, NULL);
                cbcBlockLength = 0;
            }

            cbcBlock[cbcBlockLength++] = pHeader[i];
        }

        // process header remainder
        if (cbcBlockLength != 0)
        {
            while (samr21RadioAesBusy())
                ;
            samr21RadioAesCbcEncrypt(cbcBlock, cbcBlockLength, NULL);
            cbcBlockLength = 0;
        }

        // process Payload bulk
        for (uint8_t i = 0; i < payloadLenght; i++)
        {
            if (cbcBlockLength == sizeof(cbcBlock))
            {
                while (samr21RadioAesBusy())
                    ;
                samr21RadioAesCbcEncrypt(s_cbcBlock, AES_BLOCK_SIZE, NULL);
                cbcBlockLength = 0;
            }

            cbcBlock[cbcBlockLength++] = pPayload[i];
        }

        // process Payload remainder
        if (cbcBlockLength != 0)
        {
            while (samr21RadioAesBusy())
                ;
            samr21RadioAesCbcEncrypt(cbcBlock, cbcBlockLength, NULL);
        }

        // Wait for the last CBC-Block
        while (samr21RadioAesBusy())
            ;
        samr21RadioAesReadBuffer(pFooter, 0, micSize);

        samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, (uint8_t)(numBytesPdsuUploaded + 1));

        for (uint8_t i = 0; micSize; i++)
        {

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
            samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
            if (securityLevel >= 0b100)
            {
                samr21TrxSpiTransceiveByteRaw(
                    ackFrame.mPsdu[numBytesPdsuUploaded++] ^ ackEncryptionMask[0][i]);
            }
            else
            {
                samr21TrxSpiTransceiveByteRaw(
                    ackFrame.mPsdu[numBytesPdsuUploaded++]);
            }
        }
        samr21TrxSpiCloseAccess();
    }

    buffer->status = RX_STATUS_SENDING_ACK_WAIT_TRX_END;
}



void samr21RadioRxSendAck()
{
    RxBuffer *buffer = &s_rxBuffer[s_activeRxBuffer];
    buffer->status = RX_STATUS_SENDING_ACK;

    otRadioFrame ackFrame = {.mPsdu = &(buffer->rxAckPsdu[1])};

    otMacFrameGenerateImmAck(
        &(buffer->otFrame),
        buffer->framePending,
        &ackFrame);
    buffer->rxAckPsdu[0] = ackFrame.mLength;

    while (g_trxStatus.bit.trxStatus != TRX_STATUS_PLL_ON)
    {
        samr21TrxUpdateStatus();
    }

    // Start Trasmission in advance, there is some spare time while the Preamble and SFD is transmitted
    samr21TrxSetSLP_TR(true);

    // Timout-Timer
    samr21Timer4Set((IEEE_802_15_4_FRAME_SIZE * 2) * IEEE_802_15_4_24GHZ_TIME_PER_OCTET_us);

    samr21TrxSpiStartAccess(AT86RF233_CMD_FRAMEBUFFER_WRITE, NULL);
    samr21TrxSetSLP_TR(false);
    samr21TrxSpiTransceiveBytesRaw(ackFrame.mPsdu[-1], NULL, ackFrame.mLength - 1); // -2 FCS +1 PhyLen
    samr21TrxSpiCloseSpiAccess();
}

void RTC_Handler()
{
    RTC->MODE0.INTENFLAG.bit.CMP0 = 1;
    RTC->MODE0.INTENCLR.bit.CMP0 = 1;
    NVIC_DisableIRQ(RTC_IRQn);

    if (s_rxBuffer[(s_activeRxBuffer + 1) % NUM_SAMR21_RX_BUFFER].status == RX_STATUS_WAIT_FOR_DELAYED_START)
    {
    }
}