/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "vra_pseudo_sram.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/

const uint32_t g_mixspiRootClkFreqInMHz[] = {0, 30, 50, 60, 80, 100, 120, 133, 166, 200, 240, 266, 332, 400};

/*******************************************************************************
 * Code
 ******************************************************************************/

uint32_t decode_mixspi_root_clk_defn(mixspi_root_clk_freq_t mixspiRootClkFreq)
{ 
    return g_mixspiRootClkFreqInMHz[(uint32_t)mixspiRootClkFreq];
}


