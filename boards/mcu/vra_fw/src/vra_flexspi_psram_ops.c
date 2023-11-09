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
status_t mixspi_psram_write_mcr(FLEXSPI_Type *base, uint8_t regAddr, uint32_t *mrVal)
{
    flexspi_transfer_t flashXfer;
    status_t status;

    /* Write data */
    flashXfer.deviceAddress = regAddr;
    flashXfer.port          = EXAMPLE_MIXSPI_PORT;
    flashXfer.cmdType       = kFLEXSPI_Write;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = PSRAM_CMD_LUT_SEQ_IDX_WRITEREG;
    flashXfer.data          = mrVal;
    flashXfer.dataSize      = 1;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);

    return status;
}

status_t mixspi_psram_get_mcr(FLEXSPI_Type *base, uint8_t regAddr, uint32_t *mrVal)
{
    flexspi_transfer_t flashXfer;
    status_t status;

    /* Read data */
    flashXfer.deviceAddress = regAddr;
    flashXfer.port          = EXAMPLE_MIXSPI_PORT;
    flashXfer.cmdType       = kFLEXSPI_Read;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = PSRAM_CMD_LUT_SEQ_IDX_READREG;
    flashXfer.data          = mrVal;
    flashXfer.dataSize      = 2;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);

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
status_t BOARD_InitPsRam(void)
{
    flexspi_device_config_t deviceconfig = {
        .flexspiRootClk       = 396000000, /* 396MHZ SPI serial clock, DDR serial clock 198M */
        .isSck2Enabled        = false,
        .flashSize            = 0x2000, /* 64Mb/KByte */
        .CSIntervalUnit       = kFLEXSPI_CsIntervalUnit1SckCycle,
        .CSInterval           = 5,
        .CSHoldTime           = 3,
        .CSSetupTime          = 3,
        .dataValidTime        = 1,
        .columnspace          = 0,
        .enableWordAddress    = false,
        .AWRSeqIndex          = PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA,
        .AWRSeqNumber         = 1,
        .ARDSeqIndex          = PSRAM_CMD_LUT_SEQ_IDX_READDATA,
        .ARDSeqNumber         = 1,
        .AHBWriteWaitUnit     = kFLEXSPI_AhbWriteWaitUnit2AhbCycle,
        .AHBWriteWaitInterval = 0,
        .enableWriteMask      = true,
    };

    uint32_t customLUT[64] = {
        /* Read Data */
        [4 * PSRAM_CMD_LUT_SEQ_IDX_READDATA + 0] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,            kFLEXSPI_8PAD, 0x20, kFLEXSPI_Command_RADDR_DDR,      kFLEXSPI_8PAD, 0x20),
        [4 * PSRAM_CMD_LUT_SEQ_IDX_READDATA + 1] = 
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DUMMY_RWDS_DDR, kFLEXSPI_8PAD, 0x07, kFLEXSPI_Command_READ_DDR,       kFLEXSPI_8PAD, 0x04),

        /* Write Data */
        [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA + 0] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,            kFLEXSPI_8PAD, 0xA0, kFLEXSPI_Command_RADDR_DDR,      kFLEXSPI_8PAD, 0x20),
        [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA + 1] = 
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DUMMY_RWDS_DDR, kFLEXSPI_8PAD, 0x07, kFLEXSPI_Command_WRITE_DDR,      kFLEXSPI_8PAD, 0x04),

        /* Read Register */
        [4 * PSRAM_CMD_LUT_SEQ_IDX_READREG + 0] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,            kFLEXSPI_8PAD, 0x40, kFLEXSPI_Command_RADDR_DDR,      kFLEXSPI_8PAD, 0x20),
        [4 * PSRAM_CMD_LUT_SEQ_IDX_READREG + 1] = 
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DUMMY_RWDS_DDR, kFLEXSPI_8PAD, 0x07, kFLEXSPI_Command_READ_DDR,       kFLEXSPI_8PAD, 0x04),

        /* Write Register */
        [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEREG + 0] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,            kFLEXSPI_8PAD, 0xC0, kFLEXSPI_Command_RADDR_DDR,      kFLEXSPI_8PAD, 0x20),
        [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEREG + 1] = 
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_DDR,      kFLEXSPI_8PAD, 0x08, kFLEXSPI_Command_STOP,           kFLEXSPI_1PAD, 0x00),

        /* reset */
        [4 * PSRAM_CMD_LUT_SEQ_IDX_RESET + 0] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,            kFLEXSPI_8PAD, 0xFF, kFLEXSPI_Command_DUMMY_SDR,      kFLEXSPI_8PAD, 0x03),

    };

    uint32_t mr0mr1[1];
    uint32_t mr4mr8[1];
    uint32_t mr0Val[1];
    uint32_t mr4Val[1];
    uint32_t mr8Val[1];
    flexspi_config_t config;
    status_t status = kStatus_Success;

    bsp_mixspi_init();

#if BOARD_ENABLE_PSRAM_CACHE
    CACHE64_EnableWriteBuffer(EXAMPLE_CACHE, true);
    CACHE64_EnableCache(EXAMPLE_CACHE);
#endif

    /* Get FLEXSPI default settings and configure the flexspi. */
    FLEXSPI_GetDefaultConfig(&config);

    /* Init FLEXSPI. */
    config.rxSampleClock = kFLEXSPI_ReadSampleClkExternalInputFromDqsPad;
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
#elif defined(CPU_MIMXRT685SFVKB_cm33)
    /* FlexSPI has total 1KB RX buffer.
     * Set DMA0 master to use AHB Rx Buffer0.
     */
    config.ahbConfig.buffer[0].masterIndex    = 4;   /* DMA0 */
    config.ahbConfig.buffer[0].bufferSize     = 512; /* Allocate 512B bytes for DMA0 */
    config.ahbConfig.buffer[0].enablePrefetch = true;
    config.ahbConfig.buffer[0].priority       = 0;
    /* All other masters use last buffer with 512B bytes. */
    config.ahbConfig.buffer[FSL_FEATURE_FLEXSPI_AHB_BUFFER_COUNT - 1].bufferSize = 512;
#endif
#if !(defined(FSL_FEATURE_FLEXSPI_HAS_NO_MCR0_COMBINATIONEN) && FSL_FEATURE_FLEXSPI_HAS_NO_MCR0_COMBINATIONEN)
    config.enableCombination = true;
#endif
    FLEXSPI_Init(EXAMPLE_MIXSPI, &config);

    /* Configure flash settings according to serial flash feature. */
    FLEXSPI_SetFlashConfig(EXAMPLE_MIXSPI, &deviceconfig, EXAMPLE_MIXSPI_PORT);

    /* Update LUT table. */
    FLEXSPI_UpdateLUT(EXAMPLE_MIXSPI, 0, customLUT, ARRAY_SIZE(customLUT));

    /* Do software reset. */
    FLEXSPI_SoftwareReset(EXAMPLE_MIXSPI);

    /* Reset hyper ram. */
    status = mixspi_psram_reset(EXAMPLE_MIXSPI);
    if (status != kStatus_Success)
    {
        return status;
    }

    status = mixspi_psram_get_mcr(EXAMPLE_MIXSPI, 0x0, mr0mr1);
    if (status != kStatus_Success)
    {
        return status;
    }

    status = mixspi_psram_get_mcr(EXAMPLE_MIXSPI, 0x4, mr4mr8);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* Enable RBX, burst length set to 1K. - MR8 */
    mr8Val[0] = (mr4mr8[0] & 0xFF00U) >> 8U;
    mr8Val[0] = mr8Val[0] | 0x0F;
    status    = mixspi_psram_write_mcr(EXAMPLE_MIXSPI, 0x8, mr8Val);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* Set LC code to 0x04(LC=7, maximum frequency 200M) - MR0. */
    mr0Val[0] = mr0mr1[0] & 0x00FFU;
    mr0Val[0] = (mr0Val[0] & ~0x3CU) | (4U << 2U);
    status    = mixspi_psram_write_mcr(EXAMPLE_MIXSPI, 0x0, mr0Val);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* Set WLC code to 0x01(WLC=7, maximum frequency 200M) - MR4. */
    mr4Val[0] = mr4mr8[0] & 0x00FFU;
    mr4Val[0] = (mr4Val[0] & ~0xE0U) | (1U << 5U);
    status    = mixspi_psram_write_mcr(EXAMPLE_MIXSPI, 0x4, mr4Val);
    if (status != kStatus_Success)
    {
        return status;
    }

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
