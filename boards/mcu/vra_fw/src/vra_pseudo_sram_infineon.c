/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "vra_pseudo_sram_infineon.h"
#if INFINEON_DEVICE_SERIES
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/

#if INFINEON_DEVICE_HYPERBUS
const uint32_t s_customLUT_INFINEON_Hyperbus[CUSTOM_LUT_LENGTH] = {
    /* Read Data */
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READDATA] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR,            kFLEXSPI_8PAD, 0xA0, kFLEXSPI_Command_RADDR_DDR,      kFLEXSPI_8PAD, 0x18),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READDATA + 1] = 
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_CADDR_DDR,      kFLEXSPI_8PAD, 0x10, kFLEXSPI_Command_DUMMY_RWDS_DDR, kFLEXSPI_8PAD, 0x07),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READDATA + 2] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_DDR,       kFLEXSPI_8PAD, 0x04, kFLEXSPI_Command_STOP,           kFLEXSPI_1PAD, 0x00),

    /* Write Data */
    [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR,            kFLEXSPI_8PAD, 0x20, kFLEXSPI_Command_RADDR_DDR,      kFLEXSPI_8PAD, 0x18),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA + 1] = 
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_CADDR_DDR,      kFLEXSPI_8PAD, 0x10, kFLEXSPI_Command_DUMMY_RWDS_DDR, kFLEXSPI_8PAD, 0x07),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA + 2] = 
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_DDR,      kFLEXSPI_8PAD, 0x04, kFLEXSPI_Command_STOP,           kFLEXSPI_1PAD, 0x00),

    /* Read Register */
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READREG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR,            kFLEXSPI_8PAD, 0xE0, kFLEXSPI_Command_RADDR_DDR,      kFLEXSPI_8PAD, 0x18),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READREG + 1] = 
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_CADDR_DDR,      kFLEXSPI_8PAD, 0x10, kFLEXSPI_Command_DUMMY_RWDS_DDR, kFLEXSPI_8PAD, 0x07),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READREG + 2] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_DDR,       kFLEXSPI_8PAD, 0x04, kFLEXSPI_Command_STOP,           kFLEXSPI_1PAD, 0x00),

    /* Write Register */
    [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEREG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR,            kFLEXSPI_8PAD, 0x60, kFLEXSPI_Command_RADDR_DDR,      kFLEXSPI_8PAD, 0x18),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEREG + 1] = 
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_CADDR_DDR,      kFLEXSPI_8PAD, 0x10, kFLEXSPI_Command_WRITE_DDR,      kFLEXSPI_8PAD, 0x04),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEREG + 2] = 
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_STOP,           kFLEXSPI_1PAD, 0x00, kFLEXSPI_Command_STOP,           kFLEXSPI_1PAD, 0x00),
};
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

void vra_psram_set_param_for_infineon(void)
{
    vra_printf(" -- Infineon Pseudo SRAM.\r\n");
#if INFINEON_DEVICE_HYPERBUS
    {
        g_psramPropertyInfo.mixspiPad                 = kFLEXSPI_8PAD;
        g_psramPropertyInfo.mixspiRootClkFreq         = kMixspiRootClkFreq_200MHz;
        g_psramPropertyInfo.mixspiCustomLUTVendor     = s_customLUT_INFINEON_Hyperbus;
        g_psramPropertyInfo.mixspiReadSampleClock     = kFLEXSPI_ReadSampleClkExternalInputFromDqsPad;
    }
#endif
}

status_t vra_psram_set_registers_for_infineon(MIXSPI_Type *base)
{
    status_t status = kStatus_Success;
    psram_reg_access_t regAccess;
    uint32_t IR0 = 0;

    regAccess.regNum = 2;
    regAccess.regAddr = 0x0 << 1;
    regAccess.regValue.U = 0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    vra_printf(" Read Die0 ID0: 0x%x\r\n", regAccess.regValue.U);

    regAccess.regAddr = 0x400000 << 1;
    regAccess.regValue.U = 0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    vra_printf(" Read Die1 ID0: 0x%x\r\n", regAccess.regValue.U);
    IR0 = regAccess.regValue.U;
    
    ////////////////////////////////////////////////////////////////////////////
    uint32_t CR0 = 0;

    regAccess.regAddr = 0x800 << 1;
    regAccess.regValue.U = 0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    vra_printf(" Read Die0 CR0: 0x%x\r\n", regAccess.regValue.U);

    regAccess.regAddr = 0x400800 << 1;
    regAccess.regValue.U = 0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    vra_printf(" Read Die1 CR0: 0x%x\r\n", regAccess.regValue.U);

    CR0 = regAccess.regValue.U;
    
    regAccess.regAddr = 0x800 << 1;
    regAccess.regValue.U = (CR0 & ~0x70) | (7 << 4U);//19 ohms
    status = mixspi_psram_write_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }

    regAccess.regAddr = 0x400800 << 1;
    status = mixspi_psram_write_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }

    regAccess.regAddr = 0x800 << 1;
    regAccess.regValue.U = 0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    vra_printf(" Read Die0 CR0: 0x%x\r\n", regAccess.regValue.U);
    regAccess.regAddr = 0x400800 << 1;
    regAccess.regValue.U = 0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    vra_printf(" Read Die1 CR0: 0x%x\r\n", regAccess.regValue.U);

    return status;
}

#endif // INFINEON_DEVICE_SERIES

