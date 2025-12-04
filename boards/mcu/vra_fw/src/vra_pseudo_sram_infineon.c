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
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_CADDR_DDR,      kFLEXSPI_8PAD, 0x10, kFLEXSPI_Command_DUMMY_RWDS_DDR, kFLEXSPI_8PAD, INFINEON_PSRAM_DUMMY_CYCLES),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READDATA + 2] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_DDR,       kFLEXSPI_8PAD, 0x04, kFLEXSPI_Command_STOP,           kFLEXSPI_1PAD, 0x00),

    /* Write Data */
    [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR,            kFLEXSPI_8PAD, 0x20, kFLEXSPI_Command_RADDR_DDR,      kFLEXSPI_8PAD, 0x18),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA + 1] = 
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_CADDR_DDR,      kFLEXSPI_8PAD, 0x10, kFLEXSPI_Command_DUMMY_RWDS_DDR, kFLEXSPI_8PAD, INFINEON_PSRAM_DUMMY_CYCLES),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA + 2] = 
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_DDR,      kFLEXSPI_8PAD, 0x04, kFLEXSPI_Command_STOP,           kFLEXSPI_1PAD, 0x00),

    /* Read Register */
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READREG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR,            kFLEXSPI_8PAD, 0xE0, kFLEXSPI_Command_RADDR_DDR,      kFLEXSPI_8PAD, 0x18),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READREG + 1] = 
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_CADDR_DDR,      kFLEXSPI_8PAD, 0x10, kFLEXSPI_Command_DUMMY_RWDS_DDR, kFLEXSPI_8PAD, INFINEON_PSRAM_DUMMY_CYCLES),
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
        g_psramPropertyInfo.mixspiRootClkFreq         = kMixspiRootClkFreq_332MHz;
        g_psramPropertyInfo.mixspiCustomLUTVendor     = s_customLUT_INFINEON_Hyperbus;
        g_psramPropertyInfo.mixspiReadSampleClock     = kFLEXSPI_ReadSampleClkExternalInputFromDqsPad;
    }
#endif
}

static uint32_t infineon_convert_reg_value(uint32_t reg)
{
    /* From S70KS_KL1282 DS
    When data is being accessed in register space:
    During a Read transaction on the HYPERBUS™ two bytes are transferred on each clock cycle.
    The upper order byte A (Word[15:8]) is transferred between the rising and falling edges
    of RWDS (edge-aligned). The lower order byte B (Word[7:0]) is transferred between the falling
    and rising edges of RWDS.
    During a write, the upper order byte A (Word[15:8]) is transferred on the CK rising edge
    and the lower order byte B (Word[7:0]) is transferred on the CK falling edge.

    So, register space is always read and written in Big-endian order because registers have device
    dependent fixed bit location and meaning definitions.
    */
    return ((reg & 0xFF) << 8) + ((reg >> 8) & 0xFF);
}

static uint32_t infineon_convert_reg_address(uint32_t addr)
{
    // If it is word-addressable, FlexSPI will ingore the last bit
    return (addr << 1);
}

status_t vra_psram_set_registers_for_infineon(MIXSPI_Type *base)
{
    status_t status = kStatus_Success;
    psram_reg_access_t regAccess;
    uint32_t IR0 = 0;

    // ID0 value for HYPERRAM™ is 0x0C81 if read from Die 0 or 0x4C81 if read from Die 1.
    regAccess.regNum = 2;
    regAccess.regAddr = infineon_convert_reg_address(0x0);
    regAccess.regValue.U = 0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    vra_printf(" Read Die0 ID0: 0x%x\r\n", infineon_convert_reg_value(regAccess.regValue.U));
/*
    regAccess.regAddr = infineon_convert_reg_address(0x400000);
    regAccess.regValue.U = 0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    vra_printf(" Read Die1 ID0: 0x%x\r\n", infineon_convert_reg_value(regAccess.regValue.U));
*/
    IR0 = infineon_convert_reg_value(regAccess.regValue.U);
    
    ////////////////////////////////////////////////////////////////////////////
    uint32_t CR0 = 0;

    regAccess.regAddr = infineon_convert_reg_address(0x800);
    regAccess.regValue.U = 0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    vra_printf(" Read Die0 CR0: 0x%x\r\n", infineon_convert_reg_value(regAccess.regValue.U));

    regAccess.regAddr = infineon_convert_reg_address(0x400800);
    regAccess.regValue.U = 0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    vra_printf(" Read Die1 CR0: 0x%x\r\n", infineon_convert_reg_value(regAccess.regValue.U));

    CR0 = infineon_convert_reg_value(regAccess.regValue.U);
#if 0
    regAccess.regAddr = infineon_convert_reg_address(0x800);
    regAccess.regValue.U = infineon_convert_reg_value((CR0 & 0x8FFF) | (7 << 12U)); //19 ohms
    status = mixspi_psram_write_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }

    regAccess.regAddr = infineon_convert_reg_address(0x400800);
    status = mixspi_psram_write_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }

    regAccess.regAddr = infineon_convert_reg_address(0x800);
    regAccess.regValue.U = 0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    vra_printf(" Read Die0 CR0: 0x%x\r\n", infineon_convert_reg_value(regAccess.regValue.U));

    regAccess.regAddr = infineon_convert_reg_address(0x400800);
    regAccess.regValue.U = 0;
    status = mixspi_psram_read_register(base, &regAccess);
    if (status != kStatus_Success)
    {
        return status;
    }
    vra_printf(" Read Die1 CR0: 0x%x\r\n", infineon_convert_reg_value(regAccess.regValue.U));
#endif
    return status;
}

#endif // INFINEON_DEVICE_SERIES

