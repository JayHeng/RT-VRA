/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _VRA_PSEUDO_SRAM_INFINEON_H_
#define _VRA_PSEUDO_SRAM_INFINEON_H_

#include "vra_pseudo_sram.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if INFINEON_DEVICE_S70KS0642

#define INFINEON_PSRAM_DUMMY_CYCLES (7)
#define INFINEON_PSRAM_DIE_NUMBER   (1)

#elif INFINEON_DEVICE_S70KS1282

#define INFINEON_PSRAM_DUMMY_CYCLES (7)
#define INFINEON_PSRAM_DIE_NUMBER   (2)

#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/


#endif /* _VRA_PSEUDO_SRAM_INFINEON_H_ */
