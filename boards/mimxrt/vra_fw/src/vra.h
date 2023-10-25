/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _VRA_H_
#define _VRA_H_

#include <stdint.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
// Whether to show VRA log info via UART console (for development)
#define VRA_DEBUG_LOG_INFO_ENABLE        (1)

// Supported Flexspi clock defn
typedef enum _flexspi_root_clk_freq
{
    kFlexspiRootClkFreq_30MHz  = 1,
    kFlexspiRootClkFreq_50MHz  = 2,
    kFlexspiRootClkFreq_60MHz  = 3,
    kFlexspiRootClkFreq_80MHz  = 4,
    kFlexspiRootClkFreq_100MHz = 5,
    kFlexspiRootClkFreq_120MHz = 6,
    kFlexspiRootClkFreq_133MHz = 7,
    kFlexspiRootClkFreq_166MHz = 8,
    kFlexspiRootClkFreq_200MHz = 9,
    kFlexspiRootClkFreq_240MHz = 10,
    kFlexspiRootClkFreq_266MHz = 11,
    kFlexspiRootClkFreq_332MHz = 12,
    kFlexspiRootClkFreq_400MHz = 13,
} flexspi_root_clk_freq_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void vra_main(void);

int vra_printf(const char *fmt_s, ...);

#endif /* _VRA_H_ */
