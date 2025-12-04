/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "vra_pseudo_sram.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
status_t mixspi_psram_write_register(FLEXSPI_Type *base, psram_reg_access_t *regAccess)
{
    flexspi_transfer_t flashXfer;
    status_t status;
    uint32_t writeValue = regAccess->regValue.U;

    /* Write data */
    flashXfer.deviceAddress = regAccess->regAddr;
    flashXfer.port          = EXAMPLE_MIXSPI_PORT;
    flashXfer.cmdType       = kFLEXSPI_Write;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = PSRAM_CMD_LUT_SEQ_IDX_WRITEREG;
    flashXfer.data          = &writeValue;
    flashXfer.dataSize      = regAccess->regNum;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);

    return status;
}

status_t mixspi_psram_read_register(FLEXSPI_Type *base, psram_reg_access_t *regAccess)
{
    flexspi_transfer_t flashXfer;
    status_t status;
    uint32_t regVal = 0;

    /* Read data */
    flashXfer.deviceAddress = regAccess->regAddr;
    flashXfer.port          = EXAMPLE_MIXSPI_PORT;
    flashXfer.cmdType       = kFLEXSPI_Read;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = PSRAM_CMD_LUT_SEQ_IDX_READREG;
    flashXfer.data          = &regVal;
    flashXfer.dataSize      = regAccess->regNum;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);

    regAccess->regValue.U = regVal;

    return status;
}

status_t mixspi_psram_read_id(FLEXSPI_Type *base, uint32_t idAddress, uint32_t *buffer)
{
    flexspi_transfer_t flashXfer;
    status_t status;

    /* Write data */
    flashXfer.deviceAddress = idAddress;
    flashXfer.port          = EXAMPLE_MIXSPI_PORT;
    flashXfer.cmdType       = kFLEXSPI_Read;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = PSRAM_CMD_LUT_SEQ_IDX_READID;
    flashXfer.data          = buffer;
    flashXfer.dataSize      = 4;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);

    *buffer &= 0xffffU;

    return status;
}

status_t mixspi_psram_reset(FLEXSPI_Type *base)
{
    flexspi_transfer_t flashXfer;
    status_t status;

    /* Write data */
    flashXfer.deviceAddress = 0x0U;
    flashXfer.port          = EXAMPLE_MIXSPI_PORT;
    flashXfer.cmdType       = kFLEXSPI_Command;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = PSRAM_CMD_LUT_SEQ_IDX_RESET;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);

    if (status == kStatus_Success)
    {
        /* for loop of 50000 is about 1ms (@200 MHz CPU) */
        for (uint32_t i = 2000000U; i > 0; i--)
        {
            __NOP();
        }
    }
    return status;
}

/* Initialize psram. */
status_t mixspi_psram_init(FLEXSPI_Type *base, const uint32_t *customLUT, flexspi_read_sample_clock_t rxSampleClock)
{
    flexspi_device_config_t deviceconfig = {
        .flexspiRootClk       = 200000000, /* 200MHZ SPI serial clock, DDR serial clock 100M */
        .isSck2Enabled        = false,
        .flashSize            = 0x4000, /* 128Mb/KByte */
        .CSIntervalUnit       = kFLEXSPI_CsIntervalUnit1SckCycle,
        .CSInterval           = 5,
        .CSHoldTime           = 3,
        .CSSetupTime          = 3,
        .dataValidTime        = 1,
#if DEVICE_QPI
        .addressShift         = true,
        .columnspace          = 9 + 5, /* CA:9 + CA_SHIFT:5 */
        .enableWordAddress    = false,
#elif DEVICE_HYPERBUS
        .columnspace          = 3,
        .enableWordAddress    = true,
#elif DEVICE_XCCELA
        .columnspace          = 0,
        .enableWordAddress    = false,
#endif
        .AWRSeqIndex          = PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA,
        .AWRSeqNumber         = 1,
        .ARDSeqIndex          = PSRAM_CMD_LUT_SEQ_IDX_READDATA,
        .ARDSeqNumber         = 1,
        .AHBWriteWaitUnit     = kFLEXSPI_AhbWriteWaitUnit2AhbCycle,
        .AHBWriteWaitInterval = 0,
        .enableWriteMask      = true,
    };

    flexspi_config_t config;
    status_t status = kStatus_Success;

    bsp_mixspi_init();

    /* Get FLEXSPI default settings and configure the flexspi. */
    FLEXSPI_GetDefaultConfig(&config);

    /* Init FLEXSPI. */
    config.rxSampleClock = rxSampleClock;
#if defined(CPU_RW612ETA1I)
    config.rxSampleClockPortB = rxSampleClock;
    config.rxSampleClockDiff  = true;
#endif
    /*Set AHB buffer size for reading data through AHB bus. */
    config.ahbConfig.enableAHBPrefetch    = true;
    config.ahbConfig.enableAHBBufferable  = true;
    config.ahbConfig.enableAHBCachable    = true;
    config.ahbConfig.enableReadAddressOpt = true;
    for (uint8_t i = 1; i < FSL_FEATURE_FLEXSPI_AHB_BUFFER_COUNT - 1; i++)
    {
        config.ahbConfig.buffer[i].bufferSize = 0;
    }
#if defined(CPU_MIMXRT595SFFOC_cm33)
    /* FlexSPI1 has total 2KB RX buffer.
     * Set GPU/Display master to use AHB Rx Buffer0.
     */
    config.ahbConfig.buffer[0].masterIndex    = 11;   /* GPU/Display */
    config.ahbConfig.buffer[0].bufferSize     = 1024; /* Allocate 1KB bytes for GPU/Display */
    config.ahbConfig.buffer[0].enablePrefetch = true;
    config.ahbConfig.buffer[0].priority       = 7; /* Set GPU/Display to highest priority. */
    /* All other masters use last buffer with 1KB bytes. */
    config.ahbConfig.buffer[FSL_FEATURE_FLEXSPI_AHB_BUFFER_COUNT - 1].bufferSize = 1024;
#elif defined(CPU_MIMXRT685SFVKB_cm33) || defined(CPU_RW612ETA1I)
    /* FlexSPI has total 1KB RX buffer.
     * Set DMA0 master to use AHB Rx Buffer0.
     */
#if defined(CPU_RW612ETA1I)
    config.ahbConfig.buffer[0].masterIndex    = 10;  /* GDMA */
#elif defined(CPU_MIMXRT685SFVKB_cm33)
    config.ahbConfig.buffer[0].masterIndex    = 4;   /* DMA0 */
#endif
    config.ahbConfig.buffer[0].bufferSize     = 512; /* Allocate 512B bytes for DMA0 */
    config.ahbConfig.buffer[0].enablePrefetch = true;
    config.ahbConfig.buffer[0].priority       = 0;
    /* All other masters use last buffer with 512B bytes. */
    config.ahbConfig.buffer[FSL_FEATURE_FLEXSPI_AHB_BUFFER_COUNT - 1].bufferSize = 512;
#endif
#if !(defined(FSL_FEATURE_FLEXSPI_HAS_NO_MCR0_COMBINATIONEN) && FSL_FEATURE_FLEXSPI_HAS_NO_MCR0_COMBINATIONEN)
#if defined(CPU_RW612ETA1I)
    config.enableCombination = false;
#else
    config.enableCombination = true;
#endif
#endif
    FLEXSPI_Init(base, &config);

#if defined(FLEXSPI_AHBCR_ALIGNMENT_MASK)
    /* Set alignment, otherwise the prefetch burst may cross die boundary. */
    base->AHBCR &= ~FLEXSPI_AHBCR_ALIGNMENT_MASK;
    base->AHBCR |= FLEXSPI_AHBCR_ALIGNMENT(1);/* 1KB */
#endif

    /* Configure flash settings according to serial flash feature. */
    FLEXSPI_SetFlashConfig(base, &deviceconfig, EXAMPLE_MIXSPI_PORT);

    /* Update LUT table. */
    FLEXSPI_UpdateLUT(base, 0, customLUT, CUSTOM_LUT_LENGTH);

    /* Do software reset. */
    FLEXSPI_SoftwareReset(base);

    bsp_mixspi_cleanup();

    return status;
}

status_t mixspi_psram_ipcommand_write_data(FLEXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length)
{
    flexspi_transfer_t flashXfer;
    status_t status;

    /* Write data */
    flashXfer.deviceAddress = address;
    flashXfer.port          = EXAMPLE_MIXSPI_PORT;
    flashXfer.cmdType       = kFLEXSPI_Write;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA;
    flashXfer.data          = buffer;
    flashXfer.dataSize      = length;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);

    return status;
}

status_t mixspi_psram_ipcommand_read_data(FLEXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length)
{
    flexspi_transfer_t flashXfer;
    status_t status;

    /* Read data */
    flashXfer.deviceAddress = address;
    flashXfer.port          = EXAMPLE_MIXSPI_PORT;
    flashXfer.cmdType       = kFLEXSPI_Read;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = PSRAM_CMD_LUT_SEQ_IDX_READDATA;
    flashXfer.data          = buffer;
    flashXfer.dataSize      = length;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);

    return status;
}

void mixspi_psram_ahbcommand_write_data(FLEXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length)
{
    uint32_t *startAddr = (uint32_t *)(EXAMPLE_MIXSPI_AMBA_BASE + address);
    memcpy(startAddr, buffer, length);
}

void mixspi_psram_ahbcommand_read_data(FLEXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length)
{
    uint32_t *startAddr = (uint32_t *)(EXAMPLE_MIXSPI_AMBA_BASE + address);
    memcpy(buffer, startAddr, length);
}
