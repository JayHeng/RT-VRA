/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _VRA_PSEUDO_SRAM_H_
#define _VRA_PSEUDO_SRAM_H_

#include "vra.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

// Supported PSRAM protocol type
typedef enum _psram_protocol_type
{
    kPsamProtocolType_QSPI      = 0,
    kPsamProtocolType_QPI       = 1,
    kPsamProtocolType_XCCELA    = 2,
    kPsamProtocolType_HYPERBUS  = 3,
    kPsamProtocolType_xSPI      = 4,   // JESD251

    kPsamProtocolType_MAX       = 5,
} psram_protocol_type_t;

// PSRAM property info for operation
typedef struct _psram_property_info
{
    mixspi_pad_t                mixspiPad;
    mixspi_root_clk_freq_t      mixspiRootClkFreq;
    mixspi_read_sample_clock_t  mixspiReadSampleClock;
    const uint32_t             *mixspiCustomLUTVendor;

    psram_protocol_type_t       psramProtocolType;
} psram_property_info_t;

// PSRAM status/cfg register r/w access helper
typedef struct _psram_reg_access
{
    uint8_t regNum;
    uint8_t regSeqIdx;
    uint8_t reserved[2];
    uint32_t regAddr;
    union
    {
        struct
        {
            uint32_t reg1 : 8;
            uint32_t reg2 : 8;
            uint32_t reg3 : 8;
            uint32_t reg4 : 8;
        } B;
        uint32_t U;
    } regValue;
} psram_reg_access_t;

#define PSRAM_CMD_LUT_SEQ_IDX_READDATA   0
#define PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA  1
#define PSRAM_CMD_LUT_SEQ_IDX_READREG    2
#define PSRAM_CMD_LUT_SEQ_IDX_WRITEREG   3
#define PSRAM_CMD_LUT_SEQ_IDX_RESET      4
#define PSRAM_CMD_LUT_SEQ_IDX_READID     5

#define CUSTOM_LUT_LENGTH                64

////////////////////////////////////////////////////////////////////////////////
#define APMEMORY_DEVICE_SERIES      (1)
#define APMEMORY_DEVICE_QSPI        (0)
#define APMEMORY_DEVICE_APSxx04xSQ  (0)
#define APMEMORY_DEVICE_APSxx04xDQ  (0)
#define APMEMORY_DEVICE_XCCELA      (1)
#define APMEMORY_DEVICE_APSxx08xOB  (1)  // MIMXRT595-EVK, MIMXRT685-EVK (APS6408L-OBM)
#define APMEMORY_DEVICE_APSxx16xOB  (0)
////////////////////////////////////////////////////////////////////////////////
#define ISSI_DEVICE_SERIES          (1)
#define ISSI_DEVICE_QPI             (1)
#define ISSI_DEVICE_IS6xWVQ         (1)  // RD-RW612-BGA (IS66WVQ8M4DALL)

/*******************************************************************************
 * Variables
 ******************************************************************************/

extern psram_property_info_t g_psramPropertyInfo;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern status_t mixspi_psram_write_register(MIXSPI_Type *base, psram_reg_access_t *regAccess);
extern status_t mixspi_psram_read_register(MIXSPI_Type *base, psram_reg_access_t *regAccess);
extern status_t mixspi_psram_read_id(MIXSPI_Type *base, uint16_t *buffer);
extern status_t mixspi_psram_reset(MIXSPI_Type *base);
extern status_t mixspi_psram_init(MIXSPI_Type *base, const uint32_t *customLUT, flexspi_read_sample_clock_t rxSampleClock);

extern status_t mixspi_psram_ipcommand_write_data(MIXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length);
extern status_t mixspi_psram_ipcommand_read_data(MIXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length);
extern void mixspi_psram_ahbcommand_write_data(MIXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length);
extern void mixspi_psram_ahbcommand_read_data(MIXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length);

extern uint32_t decode_mixspi_root_clk_defn(mixspi_root_clk_freq_t mixspiRootClkFreq);

#if APMEMORY_DEVICE_SERIES
extern void vra_psram_set_param_for_apmemory(void);
extern status_t vra_psram_set_registers_for_apmemory(MIXSPI_Type *base);
#endif
#if ISSI_DEVICE_SERIES
extern void vra_psram_set_param_for_issi(void);
extern status_t vra_psram_set_registers_for_issi(MIXSPI_Type *base);
#endif

#endif /* _VRA_PSEUDO_SRAM_H_ */
