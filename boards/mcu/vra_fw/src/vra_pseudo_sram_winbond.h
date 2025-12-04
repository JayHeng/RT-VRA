/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _VRA_PSEUDO_SRAM_WINBOND_H_
#define _VRA_PSEUDO_SRAM_WINBOND_H_

#include "vra_pseudo_sram.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if WINBOND_DEVICE_W956_958A8

#define WINBOND_PSRAM_DUMMY_CYCLES (7)
#define WINBOND_PSRAM_DIE_NUMBER   (1)

#elif WINBOND_DEVICE_W957_959D8

#define WINBOND_PSRAM_DUMMY_CYCLES (7)
#define WINBOND_PSRAM_DIE_NUMBER   (2)

#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/


#endif /* _VRA_PSEUDO_SRAM_WINBOND_H_ */
