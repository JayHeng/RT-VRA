/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _VRA_ADAPTER_H_
#define _VRA_ADAPTER_H_

#include "port_mixspi_info.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

// Definition to select spi peripheral
#ifndef VRA_MIXSPI_MODULE
#define VRA_MIXSPI_MODULE VRA_MIXSPI_MODULE_IS_FLEXSPI
#endif

#if VRA_MIXSPI_MODULE == VRA_MIXSPI_MODULE_IS_FLEXSPI
#define mixspi_pad_t                 flexspi_pad_t                
#define mixspi_read_sample_clock_t   flexspi_read_sample_clock_t  
#define MIXSPI_Type                  FLEXSPI_Type
#elif VRA_MIXSPI_MODULE == VRA_MIXSPI_MODULE_IS_QUADSPI
#define mixspi_pad_t                 uint32_t  
#define mixspi_read_sample_clock_t   qspi_dqs_read_sample_clock_t
#define MIXSPI_Type                  QuadSPI_Type
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/


#endif /* _VRA_ADAPTER_H_ */
