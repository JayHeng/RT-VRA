/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/
/*
 * How to set up clock using clock driver functions:
 *
 * 1. Setup clock sources.
 *
 * 2. Set up all selectors to provide selected clocks.
 *
 * 3. Set up all dividers.
 */

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Clocks v7.0
processor: MIMXRT685S
mcu_data: ksdk2_0
processor_version: 0.10.4
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

#include "fsl_power.h"
#include "fsl_clock.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* System clock frequency. */
extern uint32_t SystemCoreClock;

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_FlexspiClockSafeConfig
 * Description   : FLEXSPI clock source safe configuration weak function. 
 *                 Called before clock source(Such as PLL, Main clock) configuration.
 * Note          : Users need override this function to change FLEXSPI clock source to stable source when executing
 *                 code on FLEXSPI memory(XIP). If XIP, the function should runs in RAM and move the FLEXSPI clock source
 *                 to an stable clock to avoid instruction/data fetch issue during clock updating.
 *END**************************************************************************/
__attribute__ ((weak)) void BOARD_FlexspiClockSafeConfig(void)
{
}

/*******************************************************************************
 ************************ BOARD_InitBootClocks function ************************
 ******************************************************************************/
void BOARD_InitBootClocks(void)
{
    BOARD_BootClockRUN();
}

/*******************************************************************************
 ********************** Configuration BOARD_BootClockRUN ***********************
 ******************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_BootClockRUN
called_from_default_init: true
outputs:
- {id: LPOSC1M_clock.outFreq, value: 1 MHz}
- {id: OSTIMER_clock.outFreq, value: 1 MHz}
- {id: System_clock.outFreq, value: 12 MHz}
- {id: WAKE_32K_clock.outFreq, value: 31.25 kHz}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/*******************************************************************************
 * Variables for BOARD_BootClockRUN configuration
 ******************************************************************************/
/*******************************************************************************
 * Code for BOARD_BootClockRUN configuration
 ******************************************************************************/
void BOARD_BootClockRUN(void)
{
    /* Configure LPOSC clock*/
    POWER_DisablePD(kPDRUNCFG_PD_LPOSC);                   /* Power on LPOSC (1MHz) */
    /* Configure FFRO clock */
    POWER_DisablePD(kPDRUNCFG_PD_FFRO);                    /* Power on FFRO (48/60MHz) */
    CLOCK_EnableFfroClk(kCLOCK_Ffro48M);                   /* Enable FFRO clock*/
    /* Configure SFRO clock */
    POWER_DisablePD(kPDRUNCFG_PD_SFRO);                    /* Power on SFRO (16MHz) */
    CLOCK_EnableSfroClk();                                 /* Wait until SFRO stable */

    /* Call function BOARD_FlexspiClockSafeConfig() to move FLEXSPI clock to a stable clock source to avoid
       instruction/data fetch issue when updating PLL and Main clock if XIP(execute code on FLEXSPI memory). */
    BOARD_FlexspiClockSafeConfig();

    /* Let CPU run on ffro for safe switching */
    CLOCK_AttachClk(kFFRO_to_MAIN_CLK);

    /* Set up clock selectors - Attach clocks to the peripheries */
    CLOCK_AttachClk(kFFRO_DIV4_to_MAIN_CLK);                 /* Switch MAIN_CLK to FFRO_DIV4 */

    /*!< Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKRUN_CORE_CLOCK;
}

