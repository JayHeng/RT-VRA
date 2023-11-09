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

#endif // APMEMORY_DEVICE_SERIES

