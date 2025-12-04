/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "vra_pseudo_sram_winbond.h"
#if WINBOND_DEVICE_SERIES
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/

#if WINBOND_DEVICE_HYPERBUS
const uint32_t s_customLUT_WINBOND_Hyperbus[CUSTOM_LUT_LENGTH] = {
    /* Read Data */
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READDATA] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR,            kFLEXSPI_8PAD, 0xA0, kFLEXSPI_Command_RADDR_DDR,      kFLEXSPI_8PAD, 0x18),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READDATA + 1] = 
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_CADDR_DDR,      kFLEXSPI_8PAD, 0x10, kFLEXSPI_Command_DUMMY_RWDS_DDR, kFLEXSPI_8PAD, WINBOND_PSRAM_DUMMY_CYCLES),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READDATA + 2] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_DDR,       kFLEXSPI_8PAD, 0x04, kFLEXSPI_Command_STOP,           kFLEXSPI_1PAD, 0x00),

    /* Write Data */
    [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR,            kFLEXSPI_8PAD, 0x20, kFLEXSPI_Command_RADDR_DDR,      kFLEXSPI_8PAD, 0x18),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA + 1] = 
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_CADDR_DDR,      kFLEXSPI_8PAD, 0x10, kFLEXSPI_Command_DUMMY_RWDS_DDR, kFLEXSPI_8PAD, WINBOND_PSRAM_DUMMY_CYCLES),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA + 2] = 
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_DDR,      kFLEXSPI_8PAD, 0x04, kFLEXSPI_Command_STOP,           kFLEXSPI_1PAD, 0x00),

    /* Read Register */
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READREG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR,            kFLEXSPI_8PAD, 0xE0, kFLEXSPI_Command_RADDR_DDR,      kFLEXSPI_8PAD, 0x18),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READREG + 1] = 
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_CADDR_DDR,      kFLEXSPI_8PAD, 0x10, kFLEXSPI_Command_DUMMY_RWDS_DDR, kFLEXSPI_8PAD, WINBOND_PSRAM_DUMMY_CYCLES),
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

void vra_psram_set_param_for_winbond(void)
{
    vra_printf(" -- Winbond Pseudo SRAM.\r\n");
#if WINBOND_DEVICE_HYPERBUS
    {
        g_psramPropertyInfo.mixspiPad                 = kFLEXSPI_8PAD;
        g_psramPropertyInfo.mixspiRootClkFreq         = kMixspiRootClkFreq_332MHz;
        g_psramPropertyInfo.mixspiCustomLUTVendor     = s_customLUT_WINBOND_Hyperbus;
        g_psramPropertyInfo.mixspiReadSampleClock     = kFLEXSPI_ReadSampleClkExternalInputFromDqsPad;
    }
#endif
}

static uint32_t winbond_convert_reg_value(uint32_t reg)
{
    return ((reg & 0xFF) << 8) + ((reg >> 8) & 0xFF);
}

static uint32_t winbond_convert_reg_address(uint32_t addr)
{
    // If it is word-addressable, FlexSPI will ingore the last bit
    return (addr << 1);
}

status_t vra_psram_set_registers_for_winbond(MIXSPI_Type *base)
{
    status_t status = kStatus_Success;
    psram_reg_access_t regAccess;
    uint32_t IR0 = 0;

    // ID0 value for HYPERRAM™ is 0x0C86 if read from Die 0 or 0x4C86 if read from Die 1.
    regAccess.regNum = 2;
    regAccess.regAddr = winbond_convert_reg_address(0x0);
    regAccess.regValue.U = 0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    vra_printf(" Read Die0 ID0: 0x%x\r\n", winbond_convert_reg_value(regAccess.regValue.U));
    
#if (WINBOND_PSRAM_DIE_NUMBER == 2)
/*
    regAccess.regAddr = winbond_convert_reg_address(0x400000);
    regAccess.regValue.U = 0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    vra_printf(" Read Die1 ID0: 0x%x\r\n", winbond_convert_reg_value(regAccess.regValue.U));
*/
#endif
    IR0 = winbond_convert_reg_value(regAccess.regValue.U);
    
    ////////////////////////////////////////////////////////////////////////////
    uint32_t CR0 = 0;

    regAccess.regAddr = winbond_convert_reg_address(0x800);
    regAccess.regValue.U = 0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    vra_printf(" Read Die0 CR0: 0x%x\r\n", winbond_convert_reg_value(regAccess.regValue.U));

#if (WINBOND_PSRAM_DIE_NUMBER == 2)
    regAccess.regAddr = winbond_convert_reg_address(0x400800);
    regAccess.regValue.U = 0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    vra_printf(" Read Die1 CR0: 0x%x\r\n", winbond_convert_reg_value(regAccess.regValue.U));
#endif

    CR0 = winbond_convert_reg_value(regAccess.regValue.U);
#if 0
    regAccess.regAddr = winbond_convert_reg_address(0x800);
    regAccess.regValue.U = winbond_convert_reg_value((CR0 & 0x8FFF) | (7 << 12U)); //19 ohms
    status = mixspi_psram_write_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }

#if (WINBOND_PSRAM_DIE_NUMBER == 2)
    regAccess.regAddr = winbond_convert_reg_address(0x400800);
    status = mixspi_psram_write_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
#endif

    regAccess.regAddr = winbond_convert_reg_address(0x800);
    regAccess.regValue.U = 0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    vra_printf(" Read Die0 CR0: 0x%x\r\n", winbond_convert_reg_value(regAccess.regValue.U));

#if (WINBOND_PSRAM_DIE_NUMBER == 2)
    regAccess.regAddr = winbond_convert_reg_address(0x400800);
    regAccess.regValue.U = 0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    vra_printf(" Read Die1 CR0: 0x%x\r\n", winbond_convert_reg_value(regAccess.regValue.U));
#endif
#endif
    return status;
}

#endif // WINBOND_DEVICE_SERIES

