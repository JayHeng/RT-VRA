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

#define PSRAM_CMD_LUT_SEQ_IDX_READDATA   0
#define PSRAM_CMD_LUT_SEQ_IDX_WRITEDATA  1
#define PSRAM_CMD_LUT_SEQ_IDX_READREG    2
#define PSRAM_CMD_LUT_SEQ_IDX_WRITEREG   3
#define PSRAM_CMD_LUT_SEQ_IDX_RESET      4

/*******************************************************************************
 * Variables
 ******************************************************************************/



/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern status_t mixspi_psram_write_mcr(MIXSPI_Type *base, uint8_t regAddr, uint32_t *mrVal);
extern status_t mixspi_psram_get_mcr(MIXSPI_Type *base, uint8_t regAddr, uint32_t *mrVal);
extern status_t mixspi_psram_reset(MIXSPI_Type *base);
extern status_t BOARD_InitPsRam(void);

extern status_t mixspi_psram_ipcommand_write_data(MIXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length);
extern status_t mixspi_psram_ipcommand_read_data(MIXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length);
extern void mixspi_psram_ahbcommand_write_data(MIXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length);
extern void mixspi_psram_ahbcommand_read_data(MIXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length);

#endif /* _VRA_PSEUDO_SRAM_H_ */
