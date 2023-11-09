/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _PORT_MIXSPI_INFO_H_
#define _PORT_MIXSPI_INFO_H_

#include "vra_config.h"
#include "vra_define.h"

#include "fsl_cache.h"
#include "fsl_clock.h"
#include "fsl_cache.h"
#include "fsl_flexspi.h"
#include "fsl_io_mux.h"
#include "pin_mux.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define VRA_MIXSPI_MODULE VRA_MIXSPI_MODULE_IS_FLEXSPI

#define EXAMPLE_MIXSPI                  FLEXSPI
#define EXAMPLE_CACHE                   CACHE64_CTRL0
#define EXAMPLE_MIXSPI_AMBA_BASE        FlexSPI_AMBA_PC_CACHE_BASE
#define EXAMPLE_MIXSPI_PORT             kFLEXSPI_PortB1
#define DRAM_SIZE                       0x400000U

/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void bsp_mixspi_init(void)
{

}

static void cpu_show_clock_source(void)
{
#if VRA_DEBUG_LOG_INFO_ENABLE
    // Refer to CLOCK_GetMainClkFreq() in fsl_clock.c
    uint32_t mainClkSelB = (CLKCTL0->MAINCLKSELB) & CLKCTL0_MAINCLKSELB_SEL_MASK;
    uint32_t mainClkSelA = (CLKCTL0->MAINCLKSELA) & CLKCTL0_MAINCLKSELA_SEL_MASK;
    uint32_t clkDiv = 0;

    switch (mainClkSelB)
    {
        case CLKCTL0_MAINCLKSELB_SEL(0):
            switch (mainClkSelA)
            {
                case CLKCTL0_MAINCLKSELA_SEL(0):
                    vra_printf("VRA: CPU Clk Source from MAINCLKSELA 2'b00 - OSC clock %dHz.\r\n", CLOCK_GetSysOscFreq());
                    break;
                case CLKCTL0_MAINCLKSELA_SEL(1):
                    vra_printf("VRA: CPU Clk Source from MAINCLKSELA 2'b01 - FFRO clock (48/60m_irc) div4 Clock %dHz.\r\n", CLOCK_GetFFroFreq() / 4U);
                    break;
                case CLKCTL0_MAINCLKSELA_SEL(2):
                    vra_printf("VRA: CPU Clk Source from MAINCLKSELA 2'b10 - LPOSC Clock %dHz.\r\n", CLOCK_GetLpOscFreq());
                    break;
                case CLKCTL0_MAINCLKSELA_SEL(3):
                    vra_printf("VRA: CPU Clk Source from MAINCLKSELA 2'b11 - FFRO Clock %dHz.\r\n", CLOCK_GetFFroFreq());
                    break;
                default:
                    break;
            }
            break;

        case CLKCTL0_MAINCLKSELB_SEL(1):
            vra_printf("VRA: CPU Clk Source from MAINCLKSELB 2'b01 - SFRO Clock %dHz.\r\n", CLOCK_GetSFroFreq());
            break;

        case CLKCTL0_MAINCLKSELB_SEL(2):
            vra_printf("VRA: CPU Clk Source from MAINCLKSELB 2'b10 - Main System PLL Clock %dHz.\r\n", CLOCK_GetTcpuMciClkFreq() / ((CLKCTL0->MAINPLLCLKDIV & CLKCTL0_MAINPLLCLKDIV_DIV_MASK) + 1U));
            break;

        case CLKCTL0_MAINCLKSELB_SEL(3):
        default:
            vra_printf("VRA: CPU Clk Source from MAINCLKSELB 2'b11 - RTC 32KHz Clock %dHz.\r\n", CLOCK_GetClk32KFreq());
            break;
    }
    
    clkDiv = (CLKCTL0->SYSCPUAHBCLKDIV & CLKCTL0_SYSCPUAHBCLKDIV_DIV_MASK) >> CLKCTL0_SYSCPUAHBCLKDIV_DIV_SHIFT;
    vra_printf("VRA: CPU Clk Source Divider: %d.\r\n", (clkDiv + 1U));
    vra_printf("VRA: CPU Clk Frequency: %dHz.\r\n", CLOCK_GetCoreSysClkFreq());
#endif
}

static void mixspi_port_switch(FLEXSPI_Type *base, flexspi_port_t port, flexspi_pad_t pads)
{
}

static void mixspi_pin_init(FLEXSPI_Type *base, flexspi_port_t port, flexspi_pad_t pads)
{
    if (base == FLEXSPI)
    {
        IO_MUX_SetPinMux(IO_MUX_QUAD_SPI_FLASH);
    }
    else
    {
    }
}

static void mixspi_clock_init(FLEXSPI_Type *base, mixspi_root_clk_freq_t clkFreq)
{
    if (base == FLEXSPI)
    {
        CLOCK_EnableClock(kCLOCK_Flexspi);
        RESET_ClearPeripheralReset(kFLEXSPI_RST_SHIFT_RSTn);

        if (clkFreq == kMixspiRootClkFreq_30MHz)
        {
            /* Move FLEXSPI clock source to T3 256m / 8 to avoid instruction/data fetch issue in XIP when
             * updating PLL and main clock.
             */
            BOARD_SetFlexspiClock(FLEXSPI, 6U, 8U);
        }
        else if (clkFreq == kMixspiRootClkFreq_50MHz)
        {
            /* Set FlexSPI clock: source TddrMciFlexspi£¨320MHz£©, divide by 7 */
            BOARD_SetFlexspiClock(FLEXSPI, 5U, 7U);
        }
        else if (clkFreq == kMixspiRootClkFreq_60MHz)
        {
            /* Set FlexSPI clock: source TddrMciFlexspi£¨320MHz£©, divide by 6 */
            BOARD_SetFlexspiClock(FLEXSPI, 5U, 6U);
        }
        else if (clkFreq == kMixspiRootClkFreq_80MHz)
        {
            /* Set FlexSPI clock: source AUX0_PLL£¨260MHz£©, divide by 3 */
            BOARD_SetFlexspiClock(FLEXSPI, 2U, 3U);
        }
        else if (clkFreq == kMixspiRootClkFreq_120MHz)
        {
            /* Set FlexSPI clock: source TddrMciFlexspi£¨320MHz£©, divide by 3 */
            BOARD_SetFlexspiClock(FLEXSPI, 5U, 3U);
        }
        else if (clkFreq == kMixspiRootClkFreq_133MHz)
        {
            /* Set FlexSPI clock: source AUX0_PLL£¨260MHz£©, divide by 2 */
            BOARD_SetFlexspiClock(FLEXSPI, 2U, 2U);
        }
        else if (clkFreq == kMixspiRootClkFreq_166MHz)
        {
            /* Set FlexSPI clock: source TddrMciFlexspi£¨320MHz£©, divide by 2 */
            BOARD_SetFlexspiClock(FLEXSPI, 5U, 2U);
        }
        else if (clkFreq == kMixspiRootClkFreq_332MHz)
        {
            /* Set FlexSPI clock: source TddrMciFlexspi£¨320MHz£©, divide by 1 */
            BOARD_SetFlexspiClock(FLEXSPI, 5U, 1U);
        }
        else
        {
            vra_printf("VRA: This FlexSPI clock freq is not set.\r\n");
        }
    }
    else
    {
    }
}

static uint32_t mixspi_get_clock(FLEXSPI_Type *base)
{
    if (base == FLEXSPI)
    {
        return CLOCK_GetFlexspiClkFreq();
    }
    else
    {
        return 0;
    }
}

static void mixspi_show_clock_source(FLEXSPI_Type *base)
{
#if VRA_DEBUG_LOG_INFO_ENABLE
    uint32_t index = 0;
    uint32_t clkSel;
    uint32_t clkDiv;
    if (base == FLEXSPI)
    {
        index = 0;
        clkSel = CLKCTL0->FLEXSPIFCLKSEL & CLKCTL0_FLEXSPIFCLKSEL_SEL_MASK;
        clkDiv = CLKCTL0->FLEXSPIFCLKDIV & CLKCTL0_FLEXSPIFCLKDIV_DIV_MASK;
        switch (clkSel)
        {
            case CLKCTL0_FLEXSPIFCLKSEL_SEL(0):
                vra_printf("VRA: FLEXSPI0 Clk Source from 3'b000 - Main Clock %dHz.\r\n", CLOCK_GetMainClkFreq());
                break;

            case CLKCTL0_FLEXSPIFCLKSEL_SEL(1):
                vra_printf("VRA: FLEXSPI0 Clk Source from 3'b001 - T3 PLL MCI Flexspi Clock %dHz.\r\n", CLOCK_GetT3PllMciFlexspiClkFreq());
                break;

            case CLKCTL0_FLEXSPIFCLKSEL_SEL(2):
               {
                   uint32_t Aux0PllClkFreq = CLOCK_GetTcpuMciClkFreq() / ((CLKCTL0->AUX0PLLCLKDIV & CLKCTL0_AUX0PLLCLKDIV_DIV_MASK) + 1U);
                   vra_printf("VRA: FLEXSPI0 Clk Source from 3'b010 - SYSPLL0 AUX0 PLL Clock %dHz.\r\n", Aux0PllClkFreq);
                   break;
               }

            case CLKCTL0_FLEXSPIFCLKSEL_SEL(3):
                vra_printf("VRA: FLEXSPI0 Clk Source from 3'b011 - FFRO Clock %dHz.\r\n", CLOCK_GetFFroFreq());
                break;

            case CLKCTL0_FLEXSPIFCLKSEL_SEL(4):
               {
                   uint32_t Aux1PllClkFreq = CLOCK_GetT3PllMci213mClkFreq() / ((CLKCTL0->AUX1PLLCLKDIV & CLKCTL0_AUX1PLLCLKDIV_DIV_MASK) + 1U);
                   vra_printf("VRA: FLEXSPI0 Clk Source from 3'b100 - SYSPLL0 AUX1 PLL Clock %dHz.\r\n", Aux1PllClkFreq);
                   break;
               }

            case CLKCTL0_FLEXSPIFCLKSEL_SEL(5):
                vra_printf("VRA: FLEXSPI0 Clk Source from 3'b101 - TDDR MCI Flexspi Clock %dHz.\r\n", CLOCK_GetTddrMciFlexspiClkFreq());
                break;

            case CLKCTL0_FLEXSPIFCLKSEL_SEL(6):
                vra_printf("VRA: FLEXSPI0 Clk Source from 3'b110 - T3 PLL MCI 256MHz Clock %dHz.\r\n", CLOCK_GetT3PllMci256mClkFreq());
                break;

            default:
                break;
        }
    }
    else
    {}
    vra_printf("VRA: FLEXSPI%d Clk Source Divider: %d.\r\n", index, (clkDiv + 1U));
    vra_printf("VRA: FLEXSPI%d Clk Frequency: %dHz.\r\n", index, mixspi_get_clock(EXAMPLE_MIXSPI));
#endif
}

#endif /* _PORT_MIXSPI_INFO_H_ */
