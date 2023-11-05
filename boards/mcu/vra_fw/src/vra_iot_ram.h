/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _VRA_IOT_RAM_H_
#define _VRA_IOT_RAM_H_

#include "vra.h"
#include "fsl_common.h"
#include "fsl_flexspi.h"
#include "port_flexspi_info.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define HYPERRAM_CMD_LUT_SEQ_IDX_READDATA  0
#define HYPERRAM_CMD_LUT_SEQ_IDX_WRITEDATA 1
#define HYPERRAM_CMD_LUT_SEQ_IDX_READREG   2
#define HYPERRAM_CMD_LUT_SEQ_IDX_WRITEREG  3
#define HYPERRAM_CMD_LUT_SEQ_IDX_RESET     4

/*******************************************************************************
 * Variables
 ******************************************************************************/



/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern status_t flexspi_hyper_ram_write_mcr(FLEXSPI_Type *base, uint8_t regAddr, uint32_t *mrVal);
extern status_t flexspi_hyper_ram_get_mcr(FLEXSPI_Type *base, uint8_t regAddr, uint32_t *mrVal);
extern status_t flexspi_hyper_ram_reset(FLEXSPI_Type *base);
extern status_t BOARD_InitPsRam(void);

extern status_t flexspi_hyper_ram_ipcommand_write_data(FLEXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length);
extern status_t flexspi_hyper_ram_ipcommand_read_data(FLEXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length);
extern void flexspi_hyper_ram_ahbcommand_write_data(FLEXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length);
extern void flexspi_hyper_ram_ahbcommand_read_data(FLEXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length);

#endif /* _VRA_IOT_RAM_H_ */