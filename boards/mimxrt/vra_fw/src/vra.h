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

/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void vra_main(void);

int vra_printf(const char *fmt_s, ...);

#endif /* _VRA_H_ */
