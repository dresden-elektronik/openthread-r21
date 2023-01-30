/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#include "samr21RadioRxHandler.h"

static RxBuffer s_rxBuffer[NUM_SAMR21_RX_BUFFER];
static uint8_t s_activeRxBuffer = 0;

volatile bool s_rxSlottedActive = false;
uint32_t s_recivedFramesWhileSlotActive;
uint32_t s_slottedListeningDuration_us;
uint8_t s_slottedListeningChannel;

volatile bool s_rxAbort = false;
volatile bool s_rxHandlerActive = false;

static struct 
{
    otRadioFrame otAck;
    uint8_t psdu[IEEE_802_15_4_FRAME_SIZE];
    uint8_t numBytesUploaded;
}s_currentAckTransmission;



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

    uint8_t nonce[AES_BLOCK_SIZE];
    uint8_t encryptionBlocks[(IEEE_802_15_4_FRAME_SIZE / AES_BLOCK_SIZE)][AES_BLOCK_SIZE];

    struct
    {
        uint8_t numEncryptionBlocks;
        uint8_t numProcessedEncryptionBlocks;
    } ctr;
    
    uint8_t cbcBlock[AES_BLOCK_SIZE];

    struct
    {
        uint8_t currentBlockLen;
        uint8_t numProcessedHeaderBytes;
        uint8_t numProcessedPayloadBytes;
    } cbc;
    
}s_currentAckTransmissionSecurity;

static void samr21RadioRxCleanup(bool success){
    samr21Timer4Stop();

    __disable_irq();
    RxBuffer *bufferDone = &s_rxBuffer[s_activeRxBuffer];
  
    if (success)
    {


        bufferDone->status = RX_STATUS_DONE;
        s_activeRxBuffer = ( s_activeRxBuffer + 1 ) % NUM_SAMR21_RX_BUFFER;
        if (s_rxSlottedActive){
            s_recivedFramesWhileSlotActive++;
        }
    }

exit:
    samr21RadioRxResetBuffer(&s_rxBuffer[s_activeRxBuffer]);
    
    if(s_rxAbort){
        samr21Timer4Stop();
        samr21RadioRemoveEventHandler();
        s_rxHandlerActive = false;
        __enable_irq();
        return;
    }
    
    __enable_irq();

    if(success){
        cb_samr21RadioRxDone(bufferDone);
    }
}

static void samr21RadioRxSendImmAck()
{
    // Timout-Timer
    samr21Timer4Set((IEEE_802_15_4_FRAME_SIZE * 2) * IEEE_802_15_4_24GHZ_TIME_PER_OCTET_us);

    RxBuffer *buffer = &s_rxBuffer[s_activeRxBuffer];
    buffer->status = RX_STATUS_SENDING_ACK;

    otMacFrameGenerateImmAck(
        &(buffer->otFrame),
        buffer->framePending,
        &s_currentAckTransmission.otAck);
    s_currentAckTransmission.psdu[0] = s_currentAckTransmission.otAck.mLength;

    while (g_trxStatus.bit.trxStatus != TRX_STATUS_PLL_ON)
    {
        samr21TrxUpdateStatus();
    }

    // Start Trasmission in advance, there is some spare time while the Preamble and SFD is transmitted
    samr21TrxSetSLP_TR(true);

    samr21TrxSpiStartAccess(AT86RF233_CMD_FRAMEBUFFER_WRITE, NULL);
    samr21TrxSetSLP_TR(false);
    samr21TrxSpiTransceiveBytesRaw(&s_currentAckTransmission.otAck.mPsdu[-1], NULL, s_currentAckTransmission.otAck.mLength - 1); // -2 FCS +1 PhyLen
    samr21TrxSpiCloseAccess();

    buffer->otFrame.mInfo.mRxInfo.mAckedWithFramePending = buffer->framePending;
    buffer->otFrame.mInfo.mRxInfo.mAckedWithSecEnhAck = false;
    buffer->otFrame.mInfo.mRxInfo.mAckFrameCounter = 0;
    buffer->otFrame.mInfo.mRxInfo.mAckKeyId = 0;

    buffer->status = RX_STATUS_SENDING_ACK_WAIT_TRX_END;
}

static void samr21RadioRxAckUploadAllRaw(){
        samr21TrxSpiStartAccess(AT86RF233_CMD_FRAMEBUFFER_WRITE, NULL);
        samr21TrxSpiTransceiveBytesRaw(&s_currentAckTransmission.psdu[0], NULL, s_currentAckTransmission.psdu[0] - 1); // -2 FCS +1 PhyLen
        samr21TrxSpiCloseAccess();

        s_currentAckTransmission.numBytesUploaded = s_currentAckTransmission.psdu[-1];

        //Queue Move to RX in advance, this will only executes after TX is done
        samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);

        s_rxBuffer[s_activeRxBuffer].status = RX_STATUS_SENDING_ACK_WAIT_TRX_END;
}

static void samr21RadioRxAckUploadHeader(){

    //Write via SRAM starting from Addr 0x00 of Framebuffer (->phyLen)
    samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, 0x00);
    samr21TrxSpiTransceiveBytesRaw(&s_currentAckTransmission.psdu[0], NULL, s_currentAckTransmissionSecurity.headerLen + 1); // phyLen not in psduLen
    samr21TrxSpiCloseAccess();

    s_currentAckTransmission.numBytesUploaded = s_currentAckTransmissionSecurity.headerLen;
    s_rxBuffer[s_activeRxBuffer].status = RX_STATUS_SENDING_ENH_ACK_UPLOADED_HEADER;
}

static void samr21RadioRxAckUploadPayload(){
    s_currentAckTransmissionSecurity.ctr.numEncryptionBlocks = s_currentAckTransmissionSecurity.payloadLen / AES_BLOCK_SIZE ;
    s_currentAckTransmissionSecurity.ctr.numEncryptionBlocks += (s_currentAckTransmissionSecurity.payloadLen % AES_BLOCK_SIZE ? 1 : 0);

    while (s_currentAckTransmissionSecurity.ctr.numEncryptionBlocks > s_currentAckTransmissionSecurity.ctr.numProcessedEncryptionBlocks)
    {
        //Save some Time if no encryption is needed
        if(s_currentAckTransmissionSecurity.needsEncryption){
            // increase Nonce Counter
            s_currentAckTransmissionSecurity.nonce[(AES_BLOCK_SIZE - 1)]++;
            // start uploading next AES Block, while reading the result of the last Block
            samr21RadioAesEcbEncrypt(s_currentAckTransmissionSecurity.nonce, s_currentAckTransmissionSecurity.pendingAesResultBuffer);
            
            s_currentAckTransmissionSecurity.pendingAesResultBuffer = 
                s_currentAckTransmissionSecurity.encryptionBlocks[
                    s_currentAckTransmissionSecurity.nonce[(AES_BLOCK_SIZE - 1)]
                ]
            ;
        } 
        s_currentAckTransmissionSecurity.ctr.numProcessedEncryptionBlocks++;

        // While AES is busy, an encrypted payload-block can be uploaded 
        // This gives headroom for longer AES Operation, while simultaniusly topping of the FrameBuffer

        //Write via SRAM starting from Addr of the current Byte to the Framebuffer
        samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, (uint8_t)(s_currentAckTransmission.numBytesUploaded + 1)); // phyLen not in psduLen

        for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
        {
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
            samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
            // Upload (encrypted) payload to Framebuffer
            if (s_currentAckTransmissionSecurity.needsEncryption)
            {
                samr21TrxSpiTransceiveByteRaw(
                    s_currentAckTransmission.otAck.mPsdu[s_currentAckTransmission.numBytesUploaded++] 
                    ^ s_currentAckTransmissionSecurity.encryptionBlocks[s_currentAckTransmissionSecurity.ctr.numProcessedEncryptionBlocks][i]);
            }
            else
            {
                samr21TrxSpiTransceiveByteRaw(
                    s_currentAckTransmission.otAck.mPsdu[s_currentAckTransmission.numBytesUploaded++]);
            }
        }

        samr21TrxSpiCloseAccess();
    }
    if(s_currentAckTransmissionSecurity.needsEncryption){
        // get Last Payload encryptionBlock and start MIC encryptionBlock
        s_currentAckTransmissionSecurity.nonce[(AES_BLOCK_SIZE - 1)] = 0; ////The Nonce for MIC encryption has the CTR value 0
        samr21RadioAesEcbEncrypt(s_currentAckTransmissionSecurity.nonce, s_currentAckTransmissionSecurity.pendingAesResultBuffer);
        s_currentAckTransmissionSecurity.pendingAesResultBuffer = 
            s_currentAckTransmissionSecurity.encryptionBlocks[
                s_currentAckTransmissionSecurity.nonce[(AES_BLOCK_SIZE - 1)]
            ]
        ;
    }
    s_currentAckTransmissionSecurity.ctr.numProcessedEncryptionBlocks++;

    // Upload Last Payload-Chunk to be encrypted befor MIC
    if (s_currentAckTransmission.numBytesUploaded < (s_currentAckTransmissionSecurity.headerLen + s_currentAckTransmissionSecurity.payloadLen))
    {
        //Write via SRAM starting from Addr of the current Byte to the Framebuffer
        samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, (uint8_t)(s_currentAckTransmission.numBytesUploaded + 1));//phyLen not in psduLen

        for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
        {

            // Upload remaining to Framebuffer
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
            samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
            if (s_currentAckTransmissionSecurity.needsEncryption)
            {
                samr21TrxSpiTransceiveByteRaw(
                    s_currentAckTransmission.otAck.mPsdu[s_currentAckTransmission.numBytesUploaded++] 
                    ^ s_currentAckTransmissionSecurity.encryptionBlocks[s_currentAckTransmissionSecurity.ctr.numProcessedEncryptionBlocks][i]);
            }
            else
            {
                samr21TrxSpiTransceiveByteRaw(
                    s_currentAckTransmission.otAck.mPsdu[s_currentAckTransmission.numBytesUploaded++]);
            }

            // Break if entire payload is uploaded
            if (s_currentAckTransmission.numBytesUploaded >= (s_currentAckTransmissionSecurity.headerLen + s_currentAckTransmissionSecurity.payloadLen))
            {
                break;
            }
        }
        samr21TrxSpiCloseAccess();
    }
    s_rxBuffer[s_activeRxBuffer].status = RX_STATUS_SENDING_ENH_ACK_UPLOADED_PAYLOAD;
}
//this function gets triggered once by the main handler and after by a Timer.
//this is because there is nothing to do while the AES-Engine is busy
static void samr21RadioRxAckCalcMic(){
    
    //Only executed on first Round
    if(s_rxBuffer[s_activeRxBuffer].status != TX_STATUS_SENDING_BUSY_MIC){

        s_rxBuffer[s_activeRxBuffer].status  = TX_STATUS_SENDING_BUSY_MIC;
        // Form init Vektor (See IEEE 802.15.4-2015 Appendix B for good examples)
        s_currentAckTransmissionSecurity.cbcBlock[0] = 
            (s_currentAckTransmissionSecurity.headerLen != 0 ? 0x01000000 : 0x0) 
            | (((s_currentAckTransmissionSecurity.micSize) >> 1) << 3) | 1; // L always 2
        s_currentAckTransmissionSecurity.cbcBlock[14] = 0; // always 0 cause maxlenght is 127
        s_currentAckTransmissionSecurity.cbcBlock[15] = s_currentAckTransmissionSecurity.headerLen;
        memcpy(&s_currentAckTransmissionSecurity.cbcBlock[1], s_currentAckTransmissionSecurity.nonce, 13);

        // start the first CBC-Round  (plain ECB, r21 datasheet 40.1.4.2 Cipher Block Chaining (CBC))
        // simultaneously retive the MIC encryptionBlock from the CTR-Section ()
        samr21RadioAesEcbEncrypt(s_currentAckTransmissionSecurity.cbcBlock, s_currentAckTransmissionSecurity.pendingAesResultBuffer);
        samr21Timer4Set(21); //AES-Engine takes 21us

        // Only the result of the last CBC Round is of interest
        // The output of the rounds in between can be discarded
        s_currentAckTransmissionSecurity.pendingAesResultBuffer = NULL;

        s_currentAckTransmissionSecurity.cbcBlock[0]^= s_currentAckTransmissionSecurity.headerLen >> 8;
        s_currentAckTransmissionSecurity.cbcBlock[1]^= s_currentAckTransmissionSecurity.headerLen >> 0;

        s_currentAckTransmissionSecurity.cbc.currentBlockLen = 2;
        s_currentAckTransmissionSecurity.cbc.numProcessedHeaderBytes = 0;
        s_currentAckTransmissionSecurity.cbc.numProcessedPayloadBytes = 0;

        goto fillNextBlock;
    }

    //Still Rounds to go
    if (
        s_currentAckTransmissionSecurity.cbc.numProcessedHeaderBytes < s_currentAckTransmissionSecurity.headerLen
        ||s_currentAckTransmissionSecurity.cbc.numProcessedPayloadBytes < s_currentAckTransmissionSecurity.headerLen
    ){
        samr21RadioAesCbcEncrypt(s_currentAckTransmissionSecurity.cbcBlock, AES_BLOCK_SIZE, NULL);
        samr21Timer4Set(21);//AES takes 21us
        goto fillNextBlock;
    }

    //This is the last CBC Round the result of this is the (unencrypted!) MIC
    if (
        s_currentAckTransmissionSecurity.cbc.numProcessedHeaderBytes == s_currentAckTransmissionSecurity.headerLen
        ||s_currentAckTransmissionSecurity.cbc.numProcessedPayloadBytes == s_currentAckTransmissionSecurity.headerLen
    ){
        samr21RadioAesCbcEncrypt(s_currentAckTransmissionSecurity.cbcBlock, AES_BLOCK_SIZE, NULL);
        s_currentAckTransmissionSecurity.pendingAesResultBuffer = s_currentAckTransmissionSecurity.cbcBlock; //Put the result back into the cbcBlockBuffer
        samr21Timer4Set(21); //AES takes 21us

        s_rxBuffer[s_activeRxBuffer].status = RX_STATUS_SENDING_ENH_ACK_WAIT_FOR_MIC;
        return;
    }


fillNextBlock:

    //Fill CBC-Input-Block with Header, pad remainder with 0x00.
    if(s_currentAckTransmissionSecurity.cbc.numProcessedHeaderBytes < s_currentAckTransmissionSecurity.headerLen){
        while (s_currentAckTransmissionSecurity.cbc.currentBlockLen < AES_BLOCK_SIZE)
        {
            s_currentAckTransmissionSecurity.cbcBlock[s_currentAckTransmissionSecurity.cbc.currentBlockLen++] =
                (s_currentAckTransmissionSecurity.cbc.numProcessedHeaderBytes < s_currentAckTransmissionSecurity.headerLen ?
                    s_currentAckTransmissionSecurity.header[s_currentAckTransmissionSecurity.cbc.numProcessedHeaderBytes++]
                    : 0x00
                ) 
            ;
        }
        return;
    }
    //Fill CBC-Input-Block with Payload, pad remainder with 0x00.
    if(s_currentAckTransmissionSecurity.cbc.numProcessedPayloadBytes < s_currentAckTransmissionSecurity.payloadLen){
        while (s_currentAckTransmissionSecurity.cbc.currentBlockLen < AES_BLOCK_SIZE)
        {
            s_currentAckTransmissionSecurity.cbcBlock[s_currentAckTransmissionSecurity.cbc.currentBlockLen++] =
                (s_currentAckTransmissionSecurity.cbc.numProcessedPayloadBytes < s_currentAckTransmissionSecurity.payloadLen ?
                    s_currentAckTransmissionSecurity.payload[s_currentAckTransmissionSecurity.cbc.numProcessedPayloadBytes++]
                    : 0x00
                ) 
            ;
        }
        return;
    }
}

static void samr21RadioRxAckUploadMic(){
    
    samr21RadioAesReadBuffer(s_currentAckTransmissionSecurity.cbcBlock, 0, s_currentAckTransmissionSecurity.micSize); //AesResultBuffer == cbcBlock
    
    //Write via SRAM starting from Addr of the current Byte to the Framebuffer (->footer)
    samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, (uint8_t)(s_currentAckTransmission.numBytesUploaded + 1)); //phyLen not in psduLen 

        for (uint8_t i = 0; i < s_currentAckTransmissionSecurity.micSize; i++)
        {

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
            samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
            if (s_currentAckTransmissionSecurity.needsEncryption)
            {
                samr21TrxSpiTransceiveByteRaw(
                    s_currentAckTransmissionSecurity.cbcBlock[i] ^ s_currentAckTransmissionSecurity.encryptionBlocks[0][i]); //MIC is encrypted with CTR Block i=0
                s_currentAckTransmission.numBytesUploaded++; 
            }
            else
            {
                samr21TrxSpiTransceiveByteRaw(
                    s_currentAckTransmissionSecurity.cbcBlock[i]);
                s_currentAckTransmission.numBytesUploaded++;
            }
        }
        samr21TrxSpiCloseAccess();
        s_rxBuffer[s_activeRxBuffer].status = RX_STATUS_SENDING_ACK_WAIT_TRX_END;

    //Queue Move to RX (For ACK) in advance, this will only executes after TX is done
    samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);
}

static void samr21RadioRxSendEnhAck()
{
    while (g_trxStatus.bit.trxStatus != TRX_STATUS_PLL_ON)
    {
        samr21TrxUpdateStatus();
    }

    // Start Trasmission in advance, there is some spare time while the Preamble and SFD is transmitted
    samr21TrxSetSLP_TR(true);
    
    RxBuffer *buffer = &s_rxBuffer[s_activeRxBuffer];
    buffer->status = RX_STATUS_SENDING_ENH_ACK_UPLOADED_HEADER;
    // Timout-Timer
    samr21Timer4Set((IEEE_802_15_4_FRAME_SIZE * 2) * IEEE_802_15_4_24GHZ_TIME_PER_OCTET_us);

    otRadioFrame ack = {.mPsdu = &s_currentAckTransmission.psdu[1]};

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
        &s_currentAckTransmission.otAck);

    buffer->otFrame.mInfo.mRxInfo.mAckedWithFramePending = buffer->framePending;
    ack.mPsdu[-1] = ack.mLength;

    s_currentAckTransmissionSecurity.securityHeaderPresent = 0;

    if(otMacFrameIsSecurityEnabled(&s_currentAckTransmission.otAck)){
        __disable_irq();

        buffer->otFrame.mInfo.mRxInfo.mAckFrameCounter = g_macFrameCounter;
        otMacFrameSetFrameCounter(&s_currentAckTransmission.otAck, g_macFrameCounter++);
        
        if(otMacFrameIsKeyIdMode1(&s_currentAckTransmission.otAck)){
            otMacFrameSetKeyId(&ack, g_currKeyId);
            buffer->otFrame.mInfo.mRxInfo.mAckKeyId = g_currKeyId;
            s_currentAckTransmission.otAck.mInfo.mTxInfo.mAesKey = (const otMacKeyMaterial *)&g_currKey;
        }

        s_currentAckTransmissionSecurity.securityHeaderPresent = 1;
        __enable_irq();
    } else {
        //If there is no Security processing needed, the frame can just be uploaded
        samr21RadioRxAckUploadAllRaw();
        return;
    }

    //Get some Markers for AES 
    s_currentAckTransmissionSecurity.header = otMacFrameGetHeader(&ack);
    s_currentAckTransmissionSecurity.payload = otMacFrameGetPayload(&ack);
    s_currentAckTransmissionSecurity.mic = otMacFrameGetFooter(&ack);

    s_currentAckTransmissionSecurity.headerLen =  s_currentAckTransmissionSecurity.header - s_currentAckTransmission.psdu; //calc offset between pointers
    s_currentAckTransmissionSecurity.payloadLen = s_currentAckTransmissionSecurity.mic - s_currentAckTransmissionSecurity.payload;


    // Upload only the unencrypted Header Part first
    // This gives some Headroom to perform AES-CCM*
    samr21RadioRxAckUploadHeader();

    uint8_t securityLevel = otMacFrameGetSecurityLevel(&ack);

    s_currentAckTransmissionSecurity.needsEncryption = (securityLevel & 0b100 ? 1 : 0);

    switch (securityLevel & 0b11)
    {
    case 0b00:
        s_currentAckTransmissionSecurity.micSize = 0;
        break;
    
    case 0b01:
        s_currentAckTransmissionSecurity.micSize = 2;
        break;
    
    case 0b10:
        s_currentAckTransmissionSecurity.micSize = 4;
        break;
    
    case 0b11:
        s_currentAckTransmissionSecurity.micSize = 16;
        break;
    }

    //Generate Nonce and first EncryptionBlock if needed (TRX is busy with sending Preamble and SFD anyways)
    if (securityLevel)
    {
        otMacGenerateNonce(&ack, (uint8_t *)(&g_extAddr), s_currentAckTransmissionSecurity.nonce);
        samr21RadioAesKeySetup((uint8_t *)ack.mInfo.mTxInfo.mAesKey);

        if(s_currentAckTransmissionSecurity.needsEncryption){

            s_currentAckTransmissionSecurity.nonce[(AES_BLOCK_SIZE - 2)] = 0; // Always 0 cause max nonce-ctr == 8 (FRAME_SIZE / BLOCKSIZE)
            s_currentAckTransmissionSecurity.nonce[(AES_BLOCK_SIZE - 1)] = 1; // The first Nonce for Payload encryption has CTR value 1

            // Start generating the encrytion mask for the first block
            // The AES-Engine of the at86rf233 takes 21us per Block
            samr21RadioAesEcbEncrypt(s_currentAckTransmissionSecurity.nonce, NULL);

            s_currentAckTransmissionSecurity.ctr.numProcessedEncryptionBlocks = 0;
            s_currentAckTransmissionSecurity.pendingAesResultBuffer = 
                s_currentAckTransmissionSecurity.encryptionBlocks[
                    s_currentAckTransmissionSecurity.nonce[(AES_BLOCK_SIZE - 1)]
                ]
            ;
        }
    }
    
    // Upload the Payload next. This takes time ~100us per 16Byte Block ( ~40us without Encryption ) 
    // The Trx ist still busy transmitting the so far uploaded Framebuffer
    samr21RadioRxAckUploadPayload();


    if(s_currentAckTransmissionSecurity.micSize)
    {
        // Start the Calculation of the MIC. This take Time (~60us * ( 2 + psduLen/16 ) )
        // While the AES-Engine is Busy there is not a lot to do.
        // Because of that, this Secetion is relauched Multiple Times by a Timer-IRQ
        // We just start the first Round here and wait for the said Timer-IRQ (See TxEventHandler)
        // If Encryption is enabled, the AES-Buffer should hold the result for the CTR=0 Encrytion Block (needed for MIC encryption)
        samr21RadioRxAckCalcMic();
        return;
    }

    //FCS is appended by at86rf233

    //Queue Move to RX (For ACK) in advance, this will only executes after TX is done
    samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);

    buffer->status = RX_STATUS_SENDING_ACK_WAIT_TRX_END;
}

static void samr21RadioRxDownloadRemaining()
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
        samr21TrxSpiCloseAccess();
        samr21RadioRxCleanup(false);
        return;
    }

    // Dowload FCF+ADDR Field First
    while (numDownloadedPdsuBytes < buffer->otFrame.mLength)
    {
        uint32_t timeout = 0xFFFF;
        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
        samr21delaySysTick(CPU_WAIT_CYCLES_FOR_FRAME_BUFFER_EMPTY_FLAG);
        while ((PORT->Group[1].IN.reg & PORT_PB00) && timeout && !s_rxAbort)
        {
            timeout--;
        }

        if (timeout && !s_rxAbort)
        {
            buffer->otFrame.mPsdu[numDownloadedPdsuBytes++] =
                samr21TrxSpiReadByteRaw();
            ;
        }
        else
        {
            samr21TrxSpiCloseAccess();
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

    if(g_promiscuousMode){
        samr21RadioRxCleanup(true);
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

        samr21RadioRxSendImmAck();
    }

    samr21RadioRxCleanup(true);
}

static void samr21RadioRxDownloadAddrField()
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
        samr21TrxSpiCloseAccess();
        samr21RadioRxCleanup(false);
        return;
    }

    // Dowload FCF+ADDR Field First
    while (numDownloadedPdsuBytes < buffer->neededPdsuSizeForNextAction)
    {
        uint32_t timeout = 0xFFFF;
        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
        samr21delaySysTick(CPU_WAIT_CYCLES_FOR_FRAME_BUFFER_EMPTY_FLAG);
        while ((PORT->Group[1].IN.reg & PORT_PB00) && timeout && !s_rxAbort)
        {
            timeout--;
        }

        if (timeout && !s_rxAbort)
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

    if (!g_promiscuousMode && !otMacFrameDoesAddrMatch(
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




static void samr21RadioRxStart(uint8_t channel)
{

    // Set Timer for listening duration
    if (s_slottedListeningDuration_us)
    {
        s_rxSlottedActive = true;
        samr21RtcSetAlarm(samr21RtcGetTimestamp() + s_slottedListeningDuration_us);
    }


    // Check if theres already an ongoing Reception
    if (s_rxHandlerActive)
    {
        if (channel == g_phyCcCcaReg.bit.channel)
        {
            return; // TRX is allrdy setup for reception on the desired channel
        }
        samr21RadioRxCleanup(false);
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
        s_activeRxBuffer = ( s_activeRxBuffer + 1 ) % NUM_SAMR21_RX_BUFFER;
        samr21RadioRxResetBuffer(&s_rxBuffer[s_activeRxBuffer]);
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

}

static void samr21RadioRxReceptionStarted()
{
    s_rxBuffer[s_activeRxBuffer].status = RX_STATUS_RECIVING_FCF;
    s_rxBuffer[s_activeRxBuffer].otFrame.mInfo.mRxInfo.mTimestamp =
        samr21RtcGetTimestamp();
    s_rxBuffer[s_activeRxBuffer].otFrame.mChannel = g_phyCcCcaReg.bit.channel;
    samr21Timer4Set(RX_BACKOFF_BEFORE_FIRST_FRAMEBUFFER_ACCESS);
}

static void samr21RadioRxDownloadFCF()
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
        samr21TrxSpiCloseAccess();
        samr21RadioRxCleanup(false);
        return;
    }

    // Dowload FCS + DSN Field First
    while (numDownloadedPdsuBytes < IEEE_802_15_4_STATIC_HEADER_SIZE)
    {
        uint32_t timeout = 0xFFFF;
        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
        samr21delaySysTick(CPU_WAIT_CYCLES_FOR_FRAME_BUFFER_EMPTY_FLAG);
        while ((PORT->Group[1].IN.reg & PORT_PB00) && timeout && !s_rxAbort)
        {
            timeout--;
        }

        if (timeout && !s_rxAbort)
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


bool samr21RadioRxBusy()
{
    return s_rxBuffer[s_activeRxBuffer].status == RX_STATUS_IDLE;
}

bool samr21RadioRxIsReceiveSlotPlanned(){
    return s_slottedListeningDuration_us;
}


void samr21RadioRxResetAllBuffer()
{
    s_currentAckTransmission.otAck.mPsdu = &s_currentAckTransmission.psdu[1];

    for (uint16_t i = 0; i < NUM_SAMR21_RX_BUFFER; i++)
    {
        samr21RadioRxResetBuffer(&s_rxBuffer[i]);
    }

    s_activeRxBuffer = 0;
}

void samr21RadioRxResetBuffer(RxBuffer * buffer)
{
    __disable_irq();
    memset(buffer,0x00,sizeof(RxBuffer));
    buffer->status = RX_STATUS_IDLE;
    buffer->otFrame.mPsdu = &buffer->rxFramePsdu[1];
    __enable_irq();
}

void samr21RadioRxPrepareBuffer()
{
    s_activeRxBuffer = ( s_activeRxBuffer + 1 ) % NUM_SAMR21_RX_BUFFER;
    samr21RadioRxResetBuffer(&s_rxBuffer[s_activeRxBuffer]);
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

void samr21RadioRxSetup(uint8_t channel, uint32_t duration, uint32_t startTime)
{
    // Wait for current Transmisson (prevent retrys)
    if(samr21RadioTxBusy()){
        samr21RadioTxAbortRetrys();
        while(samr21RadioTxBusy());
    }
    

    samr21RadioSetEventHandler(&samr21RadioRxEventHandler);
    s_rxHandlerActive = true;
    s_rxAbort = false;

    if (duration || startTime)
    {
        s_slottedListeningChannel = channel;
        s_slottedListeningDuration_us = duration;

        if(startTime - samr21RtcGetTimestamp() > 0x000FFFFF) //overflow, cause time is negative
        {
            samr21RadioRxStart(channel);
            return;            
        }

        samr21RtcSetAlarm(startTime);
        return;
    }

    samr21RadioRxStart(channel);
    return;
}


void samr21RadioRxEventHandler(IrqEvent event)
{
    if(s_rxAbort){
        samr21RadioRxCleanup(false);
        return;
    }
    
    RxBuffer *buffer = &s_rxBuffer[s_activeRxBuffer];

    switch (event)
    {
    case TIMER_EVENT_4_TRIGGER:

        if (buffer->status == RX_STATUS_RECIVING_FCF)
        {
            samr21RadioRxDownloadFCF();
            return;
        }

        if (buffer->status == RX_STATUS_RECIVING_ADDR_FIELD)
        {
            samr21RadioRxDownloadAddrField();
            return;
        }

        if (buffer->status == RX_STATUS_RECIVING_REMAINING)
        {
            samr21RadioRxDownloadRemaining();
            return;
        }

        if (buffer->status == RX_STATUS_SENDING_ENH_ACK_BUSY_MIC)
        {
            samr21RadioRxAckCalcMic();
            return;
        }

        if (buffer->status == RX_STATUS_SENDING_ENH_ACK_WAIT_FOR_MIC)
        {
            samr21RadioRxAckUploadMic();
            return;
        }

        // Timeout Something went wrong
        samr21RadioRxCleanup(false);
        return;

    case TRX_EVENT_RX_START:
        if (buffer->status == RX_STATUS_IDLE)
        {
            samr21RadioRxReceptionStarted();
        }
        return;

    case TRX_EVENT_TRX_END:
        if (buffer->status == RX_STATUS_SENDING_ACK_WAIT_TRX_END)
        {
            samr21RadioRxCleanup(true);
        }
        return;

    case RTC_EVENT_ALARM_TRIGGER:
        if (!s_rxSlottedActive)
        {
            samr21RadioRxStart(s_slottedListeningChannel);
            return;
        }

        //Wait for current Reception to finish
        while (
            s_rxBuffer[s_activeRxBuffer].status != RX_STATUS_IDLE && s_rxBuffer[s_activeRxBuffer].status != RX_STATUS_DONE)
        ;
        
        s_rxSlottedActive = false;
        s_slottedListeningChannel = 0;
        s_slottedListeningDuration_us = 0;
    
        
        s_rxHandlerActive = ( samr21RadioCtrlReturnToLastHandler() == SAMR21_RADIO_STATE_RECEIVE ? true : false );

        if (!s_recivedFramesWhileSlotActive)
        {
            cb_samr21RadioRxRecivedNothing();
        }

        s_recivedFramesWhileSlotActive = 0;
    default:
        return;
    }
}