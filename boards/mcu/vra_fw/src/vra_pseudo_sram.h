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

#define PSRAM_CMD_LUT_SEQ_IDX_READDATA   0
#define PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA  1
#define PSRAM_CMD_LUT_SEQ_IDX_READREG    2
#define PSRAM_CMD_LUT_SEQ_IDX_WRITEREG   3
#define PSRAM_CMD_LUT_SEQ_IDX_RESET      4

#define CUSTOM_LUT_LENGTH                 64

////////////////////////////////////////////////////////////////////////////////
#define APMEMORY_DEVICE_SERIES      (1)
#define APMEMORY_DEVICE_QSPI        (0)
#define APMEMORY_DEVICE_APSxx04xSQ  (0)
#define APMEMORY_DEVICE_APSxx04xDQ  (0)
#define APMEMORY_DEVICE_XCCELA      (1)
#define APMEMORY_DEVICE_APSxx08xOB  (1)  // MIMXRT595-EVK, MIMXRT685-EVK (APS6408L-OBM)
#define APMEMORY_DEVICE_APSxx16xOB  (0)

/*******************************************************************************
 * Variables
 ******************************************************************************/

extern psram_property_info_t g_psramPropertyInfo;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern status_t mixspi_psram_write_mcr(MIXSPI_Type *base, uint8_t regAddr, uint32_t *mrVal);
extern status_t mixspi_psram_get_mcr(MIXSPI_Type *base, uint8_t regAddr, uint32_t *mrVal);
extern status_t mixspi_psram_reset(MIXSPI_Type *base);
extern status_t mixspi_psram_init(MIXSPI_Type *base, const uint32_t *customLUT, flexspi_read_sample_clock_t rxSampleClock);

extern status_t mixspi_psram_ipcommand_write_data(MIXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length);
extern status_t mixspi_psram_ipcommand_read_data(MIXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length);
extern void mixspi_psram_ahbcommand_write_data(MIXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length);
extern void mixspi_psram_ahbcommand_read_data(MIXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length);

#if APMEMORY_DEVICE_SERIES
extern void vra_psram_set_param_for_apmemory(void);
#endif

#endif /* _VRA_PSEUDO_SRAM_H_ */
