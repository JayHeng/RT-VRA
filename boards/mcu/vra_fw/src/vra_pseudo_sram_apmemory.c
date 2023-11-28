/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "vra_pseudo_sram_apmemory.h"
#if APMEMORY_DEVICE_SERIES
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/

#if APMEMORY_DEVICE_XCCELA
const uint32_t s_customLUT_APMEMORY_Xccela[CUSTOM_LUT_LENGTH] = {
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
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

void vra_psram_set_param_for_apmemory(void)
{
    vra_printf(" -- Apmemory Pseudo SRAM.\r\n");
#if APMEMORY_DEVICE_XCCELA
    {
        g_psramPropertyInfo.mixspiPad                 = kFLEXSPI_8PAD;
        g_psramPropertyInfo.mixspiRootClkFreq         = kMixspiRootClkFreq_400MHz;
        g_psramPropertyInfo.mixspiCustomLUTVendor     = s_customLUT_APMEMORY_Xccela;
        g_psramPropertyInfo.mixspiReadSampleClock     = kFLEXSPI_ReadSampleClkExternalInputFromDqsPad;
    }
#endif
}

status_t vra_psram_set_registers_for_apmemory(MIXSPI_Type *base)
{
    uint32_t mr0mr1[1];
    uint32_t mr4mr8[1];
    uint32_t mr0Val[1];
    uint32_t mr4Val[1];
    uint32_t mr8Val[1];
    psram_reg_access_t regAccess;
    status_t status = kStatus_Success;

    /* Reset hyper ram. */
    status = mixspi_psram_reset(base);
    if (status != kStatus_Success)
    {
        return status;
    }
    
    regAccess.regNum = 2;
    regAccess.regAddr = 0x0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    mr0mr1[0] = regAccess.regValue.U;

    regAccess.regNum = 2;
    regAccess.regAddr = 0x4;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    mr4mr8[0] = regAccess.regValue.U;

    /* Enable RBX, burst length set to 1K. - MR8 */
    mr8Val[0] = (mr4mr8[0] & 0xFF00U) >> 8U;
    mr8Val[0] = mr8Val[0] | 0x0F;
    regAccess.regNum = 1;
    regAccess.regAddr = 0x8;
    regAccess.regValue.U = mr8Val[0];
    status    = mixspi_psram_write_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* Set LC code to 0x04(LC=7, maximum frequency 200M) - MR0. */
    mr0Val[0] = mr0mr1[0] & 0x00FFU;
    mr0Val[0] = (mr0Val[0] & ~0x3CU) | (4U << 2U);
    regAccess.regNum = 1;
    regAccess.regAddr = 0x0;
    regAccess.regValue.U = mr0Val[0];
    status    = mixspi_psram_write_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* Set WLC code to 0x01(WLC=7, maximum frequency 200M) - MR4. */
    mr4Val[0] = mr4mr8[0] & 0x00FFU;
    mr4Val[0] = (mr4Val[0] & ~0xE0U) | (1U << 5U);
    regAccess.regNum = 1;
    regAccess.regAddr = 0x4;
    regAccess.regValue.U = mr4Val[0];
    status    = mixspi_psram_write_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }

    return status;
}

#endif // APMEMORY_DEVICE_SERIES

