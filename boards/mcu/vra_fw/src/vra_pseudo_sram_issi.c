/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "vra_pseudo_sram_issi.h"
#if ISSI_DEVICE_SERIES
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/

#if ISSI_DEVICE_QPI
const uint32_t s_customLUT_ISSI_Qpi[CUSTOM_LUT_LENGTH] = {
    /* Read Data */
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READDATA + 0] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR,            kFLEXSPI_4PAD, 0xAA, kFLEXSPI_Command_DDR,            kFLEXSPI_4PAD, 0x00),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READDATA + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_RADDR_DDR,      kFLEXSPI_4PAD, 16,   kFLEXSPI_Command_CADDR_DDR,      kFLEXSPI_4PAD, 16),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READDATA + 2] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DUMMY_DDR,      kFLEXSPI_4PAD, 28,   kFLEXSPI_Command_READ_DDR,       kFLEXSPI_4PAD, 0x01),

    /* Write Data */
    [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA + 0] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR,            kFLEXSPI_4PAD, 0x22, kFLEXSPI_Command_DDR,            kFLEXSPI_4PAD, 0x00),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_RADDR_DDR,      kFLEXSPI_4PAD, 16,   kFLEXSPI_Command_CADDR_DDR,      kFLEXSPI_4PAD, 16),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA + 2] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DUMMY_DDR,      kFLEXSPI_4PAD, 28,   kFLEXSPI_Command_WRITE_DDR,      kFLEXSPI_4PAD, 0x01),

    /* Read Register */
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READREG + 0] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR,            kFLEXSPI_4PAD, 0xCC, kFLEXSPI_Command_DDR,            kFLEXSPI_4PAD, 0x00),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READREG + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_RADDR_DDR,      kFLEXSPI_4PAD, 16,   kFLEXSPI_Command_CADDR_DDR,      kFLEXSPI_4PAD, 16),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READREG + 2] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DUMMY_DDR,      kFLEXSPI_4PAD, 12,   kFLEXSPI_Command_READ_DDR,       kFLEXSPI_4PAD, 0x01),

    /* Write Register */
    [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEREG + 0] = 
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR,            kFLEXSPI_4PAD, 0x66, kFLEXSPI_Command_DDR,            kFLEXSPI_4PAD, 0x00),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEREG + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_RADDR_DDR,      kFLEXSPI_4PAD, 16,   kFLEXSPI_Command_CADDR_DDR,      kFLEXSPI_4PAD, 16),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_WRITEREG + 2] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_DDR,      kFLEXSPI_4PAD, 0x01, kFLEXSPI_Command_STOP,           kFLEXSPI_1PAD, 0x00),

    /* Read ID */
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READID + 0] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,            kFLEXSPI_4PAD, 0xE0, kFLEXSPI_Command_RADDR_DDR,      kFLEXSPI_4PAD, 16),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READID + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_CADDR_DDR,      kFLEXSPI_4PAD, 16,   kFLEXSPI_Command_DUMMY_RWDS_DDR, kFLEXSPI_4PAD, 0x08),
    [4 * PSRAM_CMD_LUT_SEQ_IDX_READID + 2] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_DDR,       kFLEXSPI_4PAD, 0x01, kFLEXSPI_Command_STOP,           kFLEXSPI_1PAD, 0x00),
};
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

void vra_psram_set_param_for_issi(void)
{
    vra_printf(" -- ISSI Pseudo SRAM.\r\n");
#if ISSI_DEVICE_QPI
    {
        g_psramPropertyInfo.mixspiPad                 = kFLEXSPI_4PAD;
        g_psramPropertyInfo.mixspiRootClkFreq         = kMixspiRootClkFreq_332MHz;
        g_psramPropertyInfo.mixspiCustomLUTVendor     = s_customLUT_ISSI_Qpi;
        g_psramPropertyInfo.mixspiReadSampleClock     = kFLEXSPI_ReadSampleClkExternalInputFromDqsPad;
    }
#endif
}

status_t vra_psram_set_registers_for_issi(MIXSPI_Type *base)
{
    uint32_t identification = 0x00U;
    uint16_t registerVal    = 0x00U;
    psram_reg_access_t regAccess;
    status_t status = kStatus_Success;

    /* Read identification: the Manufacturer ID of ISSI's PSRAM(IS66/67WVQ8M4DALL) is 0x03U  */
    do
    {
        status = mixspi_psram_read_id(base, 0x0, &identification);
        if ((status != kStatus_Success) || (identification & 0x03U) != 0x03U)
        {
            status = kStatus_Fail;
            break;
        }

        /* Read configuration register: the default setting is 0xF052(see table 6.1 Configuration Register in
           PSRAM's(IS66/67WVQ8M4DALL) datasheet), which Latency code(CR[7:4]) is 0101b, which supported max frequency
           is 200MHz.*/
        regAccess.regNum = 2;
        regAccess.regAddr = 0x04UL << 9;
        status = mixspi_psram_read_register(base, &regAccess);
        registerVal = regAccess.regValue.U & 0xFFFF;
        if ((status != kStatus_Success) || registerVal != 0xF052U)
        {
            status = kStatus_Fail;
            break;
        }

        /* Initial access latency configuration, which is located in bit3 of CR. */
        registerVal |= (uint16_t)(0x01UL << 3);

        /* Write configuration register: */
        regAccess.regNum = 2;
        regAccess.regAddr = 0x04UL << 9;
        regAccess.regValue.U = registerVal & 0xFFFF;
        status = mixspi_psram_write_register(base, &regAccess);
        if ((status != kStatus_Success) || registerVal != 0xF05AU)
        {
            status = kStatus_Fail;
            break;
        }

        /* Reset */
        registerVal = 0x00U;

        /* Read configuration register: changes default Variable Latency into Fixed Latency: 0xF05A.
           Note: FlexSPI only supports fixed latency mode for ISSI's psram. */
        regAccess.regNum = 2;
        regAccess.regAddr = 0x04UL << 9;
        status = mixspi_psram_read_register(base, &regAccess);
        registerVal = regAccess.regValue.U & 0xFFFF;
        if ((status != kStatus_Success) || registerVal != 0xF05AU)
        {
            status = kStatus_Fail;
            break;
        }
    } while (false);

    return status;
}

#endif // ISSI_DEVICE_SERIES

