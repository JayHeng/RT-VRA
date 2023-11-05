/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _PORT_FLEXSPI_INFO_H_
#define _PORT_FLEXSPI_INFO_H_

/*${header:start}*/
#include "fsl_cache.h"
#include "fsl_clock.h"
#include "fsl_cache.h"
#include "fsl_iopctl.h"
#include "fsl_power.h"
#include "fsl_flexspi.h"
#include "pin_mux.h"
#include "board.h"
#include "vra.h"
/*${header:end}*/
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define EXAMPLE_FLEXSPI                 FLEXSPI1
#define EXAMPLE_FLEXSPI_AMBA_BASE       FlexSPI1_AMBA_BASE
#define EXAMPLE_FLEXSPI_PORT            kFLEXSPI_PortA1
#define DRAM_SIZE                       0x800000U

/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

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
                    vra_printf("VRA: CPU Clk Source from MAINCLKSELA 2'b00 - LPOSC Clock %dHz.\r\n", CLOCK_GetLpOscFreq());
                    break;
                case CLKCTL0_MAINCLKSELA_SEL(1):
                   {
                       uint32_t freq = (CLK_FRO_CLK) / (2U << (((CLKCTL0->FRODIVSEL) & CLKCTL0_FRODIVSEL_SEL_MASK) >> CLKCTL0_FRODIVSEL_SEL_SHIFT));
                       vra_printf("VRA: CPU Clk Source from MAINCLKSELA 2'b01 - FRODIVSEL Clock %dHz.\r\n", freq);
                       break;
                   }
                case CLKCTL0_MAINCLKSELA_SEL(2):
                    vra_printf("VRA: CPU Clk Source from MAINCLKSELA 2'b10 - OSC Clock %dHz.\r\n", CLOCK_GetXtalInClkFreq());
                    break;
                case CLKCTL0_MAINCLKSELA_SEL(3):
                    vra_printf("VRA: CPU Clk Source from MAINCLKSELA 2'b11 - FRO_DIV1 Clock %dHz.\r\n", CLK_FRO_CLK);
                    break;
                default:
                    break;
            }
            break;

        case CLKCTL0_MAINCLKSELB_SEL(1):
            vra_printf("VRA: CPU Clk Source from MAINCLKSELB 2'b01 - Main System PLL Clock %dHz.\r\n", CLOCK_GetSysPfdFreq(kCLOCK_Pfd0) / ((CLKCTL0->MAINPLLCLKDIV & CLKCTL0_MAINPLLCLKDIV_DIV_MASK) + 1U));
            break;

        case CLKCTL0_MAINCLKSELB_SEL(2):
            vra_printf("VRA: CPU Clk Source from MAINCLKSELB 2'b10 - RTC 32KHz Clock %dHz.\r\n", CLOCK_GetOsc32KFreq());
            break;

        default:
            vra_printf("VRA: CPU Clk Source from MAINCLKSELB 2'b11 - Reserved.\r\n");
            break;
    }
    
    clkDiv = (CLKCTL0->SYSCPUAHBCLKDIV & CLKCTL0_SYSCPUAHBCLKDIV_DIV_MASK) >> CLKCTL0_SYSCPUAHBCLKDIV_DIV_SHIFT;
    vra_printf("VRA: CPU Clk Source Divider: %d.\r\n", (clkDiv + 1U));
    vra_printf("VRA: CPU Clk Frequency: %dHz.\r\n", CLOCK_GetFreq(kCLOCK_CoreSysClk));
#endif
}

static void flexspi_port_switch(FLEXSPI_Type *base, flexspi_port_t port, flexspi_pad_t pads)
{
}

static void flexspi_pin_init(FLEXSPI_Type *base, flexspi_port_t port, flexspi_pad_t pads)
{
    if (base == FLEXSPI0)
    {
    }
    else if (base == FLEXSPI1)
    {
        const uint32_t port4_pin11_config = (/* Pin is configured as FLEXSPI1_SCLK */
                                             IOPCTL_PIO_FUNC2 |
                                             /* Disable pull-up / pull-down function */
                                             IOPCTL_PIO_PUPD_DI |
                                             /* Enable pull-down function */
                                             IOPCTL_PIO_PULLDOWN_EN |
                                             /* Enables input buffer function */
                                             IOPCTL_PIO_INBUF_EN |
                                             /* Normal mode */
                                             IOPCTL_PIO_SLEW_RATE_NORMAL |
                                             /* Full drive enable */
                                             IOPCTL_PIO_FULLDRIVE_EN |
                                             /* Analog mux is disabled */
                                             IOPCTL_PIO_ANAMUX_DI |
                                             /* Pseudo Output Drain is disabled */
                                             IOPCTL_PIO_PSEDRAIN_DI |
                                             /* Input function is not inverted */
                                             IOPCTL_PIO_INV_DI);
        /* PORT4 PIN11 (coords: H2) is configured as FLEXSPI1_SCLK */
        IOPCTL_PinMuxSet(IOPCTL, 4U, 11U, port4_pin11_config);

        const uint32_t port4_pin12_config = (/* Pin is configured as FLEXSPI1_DATA0 */
                                             IOPCTL_PIO_FUNC2 |
                                             /* Disable pull-up / pull-down function */
                                             IOPCTL_PIO_PUPD_DI |
                                             /* Enable pull-down function */
                                             IOPCTL_PIO_PULLDOWN_EN |
                                             /* Enables input buffer function */
                                             IOPCTL_PIO_INBUF_EN |
                                             /* Normal mode */
                                             IOPCTL_PIO_SLEW_RATE_NORMAL |
                                             /* Full drive enable */
                                             IOPCTL_PIO_FULLDRIVE_EN |
                                             /* Analog mux is disabled */
                                             IOPCTL_PIO_ANAMUX_DI |
                                             /* Pseudo Output Drain is disabled */
                                             IOPCTL_PIO_PSEDRAIN_DI |
                                             /* Input function is not inverted */
                                             IOPCTL_PIO_INV_DI);
        /* PORT4 PIN12 (coords: H1) is configured as FLEXSPI1_DATA0 */
        IOPCTL_PinMuxSet(IOPCTL, 4U, 12U, port4_pin12_config);

        const uint32_t port4_pin13_config = (/* Pin is configured as FLEXSPI1_DATA1 */
                                             IOPCTL_PIO_FUNC2 |
                                             /* Disable pull-up / pull-down function */
                                             IOPCTL_PIO_PUPD_DI |
                                             /* Enable pull-down function */
                                             IOPCTL_PIO_PULLDOWN_EN |
                                             /* Enables input buffer function */
                                             IOPCTL_PIO_INBUF_EN |
                                             /* Normal mode */
                                             IOPCTL_PIO_SLEW_RATE_NORMAL |
                                             /* Full drive enable */
                                             IOPCTL_PIO_FULLDRIVE_EN |
                                             /* Analog mux is disabled */
                                             IOPCTL_PIO_ANAMUX_DI |
                                             /* Pseudo Output Drain is disabled */
                                             IOPCTL_PIO_PSEDRAIN_DI |
                                             /* Input function is not inverted */
                                             IOPCTL_PIO_INV_DI);
        /* PORT4 PIN13 (coords: G2) is configured as FLEXSPI1_DATA1 */
        IOPCTL_PinMuxSet(IOPCTL, 4U, 13U, port4_pin13_config);

        const uint32_t port4_pin14_config = (/* Pin is configured as FLEXSPI1_DATA2 */
                                             IOPCTL_PIO_FUNC2 |
                                             /* Disable pull-up / pull-down function */
                                             IOPCTL_PIO_PUPD_DI |
                                             /* Enable pull-down function */
                                             IOPCTL_PIO_PULLDOWN_EN |
                                             /* Enables input buffer function */
                                             IOPCTL_PIO_INBUF_EN |
                                             /* Normal mode */
                                             IOPCTL_PIO_SLEW_RATE_NORMAL |
                                             /* Full drive enable */
                                             IOPCTL_PIO_FULLDRIVE_EN |
                                             /* Analog mux is disabled */
                                             IOPCTL_PIO_ANAMUX_DI |
                                             /* Pseudo Output Drain is disabled */
                                             IOPCTL_PIO_PSEDRAIN_DI |
                                             /* Input function is not inverted */
                                             IOPCTL_PIO_INV_DI);
        /* PORT4 PIN14 (coords: F1) is configured as FLEXSPI1_DATA2 */
        IOPCTL_PinMuxSet(IOPCTL, 4U, 14U, port4_pin14_config);

        const uint32_t port4_pin15_config = (/* Pin is configured as FLEXSPI1_DATA3 */
                                             IOPCTL_PIO_FUNC2 |
                                             /* Disable pull-up / pull-down function */
                                             IOPCTL_PIO_PUPD_DI |
                                             /* Enable pull-down function */
                                             IOPCTL_PIO_PULLDOWN_EN |
                                             /* Enables input buffer function */
                                             IOPCTL_PIO_INBUF_EN |
                                             /* Normal mode */
                                             IOPCTL_PIO_SLEW_RATE_NORMAL |
                                             /* Full drive enable */
                                             IOPCTL_PIO_FULLDRIVE_EN |
                                             /* Analog mux is disabled */
                                             IOPCTL_PIO_ANAMUX_DI |
                                             /* Pseudo Output Drain is disabled */
                                             IOPCTL_PIO_PSEDRAIN_DI |
                                             /* Input function is not inverted */
                                             IOPCTL_PIO_INV_DI);
        /* PORT4 PIN15 (coords: K3) is configured as FLEXSPI1_DATA3 */
        IOPCTL_PinMuxSet(IOPCTL, 4U, 15U, port4_pin15_config);

        const uint32_t port4_pin16_config = (/* Pin is configured as FLEXSPI1_DQS */
                                             IOPCTL_PIO_FUNC2 |
                                             /* Enable pull-up / pull-down function */
                                             IOPCTL_PIO_PUPD_EN |
                                             /* Enable pull-down function */
                                             IOPCTL_PIO_PULLDOWN_EN |
                                             /* Enables input buffer function */
                                             IOPCTL_PIO_INBUF_EN |
                                             /* Normal mode */
                                             IOPCTL_PIO_SLEW_RATE_NORMAL |
                                             /* Full drive enable */
                                             IOPCTL_PIO_FULLDRIVE_EN |
                                             /* Analog mux is disabled */
                                             IOPCTL_PIO_ANAMUX_DI |
                                             /* Pseudo Output Drain is disabled */
                                             IOPCTL_PIO_PSEDRAIN_DI |
                                             /* Input function is not inverted */
                                             IOPCTL_PIO_INV_DI);
        /* PORT4 PIN16 (coords: H3) is configured as FLEXSPI1_DQS */
        IOPCTL_PinMuxSet(IOPCTL, 4U, 16U, port4_pin16_config);

        const uint32_t port4_pin18_config = (/* Pin is configured as FLEXSPI1_SS0_N */
                                             IOPCTL_PIO_FUNC2 |
                                             /* Disable pull-up / pull-down function */
                                             IOPCTL_PIO_PUPD_DI |
                                             /* Enable pull-down function */
                                             IOPCTL_PIO_PULLDOWN_EN |
                                             /* Enables input buffer function */
                                             IOPCTL_PIO_INBUF_EN |
                                             /* Normal mode */
                                             IOPCTL_PIO_SLEW_RATE_NORMAL |
                                             /* Full drive enable */
                                             IOPCTL_PIO_FULLDRIVE_EN |
                                             /* Analog mux is disabled */
                                             IOPCTL_PIO_ANAMUX_DI |
                                             /* Pseudo Output Drain is disabled */
                                             IOPCTL_PIO_PSEDRAIN_DI |
                                             /* Input function is not inverted */
                                             IOPCTL_PIO_INV_DI);
        /* PORT4 PIN18 (coords: E13) is configured as FLEXSPI1_SS0_N */
        IOPCTL_PinMuxSet(IOPCTL, 4U, 18U, port4_pin18_config);

        const uint32_t port5_pin15_config = (/* Pin is configured as FLEXSPI1_DATA4 */
                                             IOPCTL_PIO_FUNC2 |
                                             /* Disable pull-up / pull-down function */
                                             IOPCTL_PIO_PUPD_DI |
                                             /* Enable pull-down function */
                                             IOPCTL_PIO_PULLDOWN_EN |
                                             /* Enables input buffer function */
                                             IOPCTL_PIO_INBUF_EN |
                                             /* Normal mode */
                                             IOPCTL_PIO_SLEW_RATE_NORMAL |
                                             /* Full drive enable */
                                             IOPCTL_PIO_FULLDRIVE_EN |
                                             /* Analog mux is disabled */
                                             IOPCTL_PIO_ANAMUX_DI |
                                             /* Pseudo Output Drain is disabled */
                                             IOPCTL_PIO_PSEDRAIN_DI |
                                             /* Input function is not inverted */
                                             IOPCTL_PIO_INV_DI);
        /* PORT5 PIN15 (coords: H5) is configured as FLEXSPI1_DATA4 */
        IOPCTL_PinMuxSet(IOPCTL, 5U, 15U, port5_pin15_config);

        const uint32_t port5_pin16_config = (/* Pin is configured as FLEXSPI1_DATA5 */
                                             IOPCTL_PIO_FUNC2 |
                                             /* Disable pull-up / pull-down function */
                                             IOPCTL_PIO_PUPD_DI |
                                             /* Enable pull-down function */
                                             IOPCTL_PIO_PULLDOWN_EN |
                                             /* Enables input buffer function */
                                             IOPCTL_PIO_INBUF_EN |
                                             /* Normal mode */
                                             IOPCTL_PIO_SLEW_RATE_NORMAL |
                                             /* Full drive enable */
                                             IOPCTL_PIO_FULLDRIVE_EN |
                                             /* Analog mux is disabled */
                                             IOPCTL_PIO_ANAMUX_DI |
                                             /* Pseudo Output Drain is disabled */
                                             IOPCTL_PIO_PSEDRAIN_DI |
                                             /* Input function is not inverted */
                                             IOPCTL_PIO_INV_DI);
        /* PORT5 PIN16 (coords: H4) is configured as FLEXSPI1_DATA5 */
        IOPCTL_PinMuxSet(IOPCTL, 5U, 16U, port5_pin16_config);

        const uint32_t port5_pin17_config = (/* Pin is configured as FLEXSPI1_DATA6 */
                                             IOPCTL_PIO_FUNC2 |
                                             /* Disable pull-up / pull-down function */
                                             IOPCTL_PIO_PUPD_DI |
                                             /* Enable pull-down function */
                                             IOPCTL_PIO_PULLDOWN_EN |
                                             /* Enables input buffer function */
                                             IOPCTL_PIO_INBUF_EN |
                                             /* Normal mode */
                                             IOPCTL_PIO_SLEW_RATE_NORMAL |
                                             /* Full drive enable */
                                             IOPCTL_PIO_FULLDRIVE_EN |
                                             /* Analog mux is disabled */
                                             IOPCTL_PIO_ANAMUX_DI |
                                             /* Pseudo Output Drain is disabled */
                                             IOPCTL_PIO_PSEDRAIN_DI |
                                             /* Input function is not inverted */
                                             IOPCTL_PIO_INV_DI);
        /* PORT5 PIN17 (coords: J3) is configured as FLEXSPI1_DATA6 */
        IOPCTL_PinMuxSet(IOPCTL, 5U, 17U, port5_pin17_config);

        const uint32_t port5_pin18_config = (/* Pin is configured as FLEXSPI1_DATA7 */
                                             IOPCTL_PIO_FUNC2 |
                                             /* Disable pull-up / pull-down function */
                                             IOPCTL_PIO_PUPD_DI |
                                             /* Enable pull-down function */
                                             IOPCTL_PIO_PULLDOWN_EN |
                                             /* Enables input buffer function */
                                             IOPCTL_PIO_INBUF_EN |
                                             /* Normal mode */
                                             IOPCTL_PIO_SLEW_RATE_NORMAL |
                                             /* Full drive enable */
                                             IOPCTL_PIO_FULLDRIVE_EN |
                                             /* Analog mux is disabled */
                                             IOPCTL_PIO_ANAMUX_DI |
                                             /* Pseudo Output Drain is disabled */
                                             IOPCTL_PIO_PSEDRAIN_DI |
                                             /* Input function is not inverted */
                                             IOPCTL_PIO_INV_DI);
        /* PORT5 PIN18 (coords: J4) is configured as FLEXSPI1_DATA7 */
        IOPCTL_PinMuxSet(IOPCTL, 5U, 18U, port5_pin18_config);
    }
    else
    {
    }
}

static void flexspi_clock_init(FLEXSPI_Type *base, flexspi_root_clk_freq_t clkFreq)
{
    if (base == FLEXSPI0)
    {
        if (clkFreq == kFlexspiRootClkFreq_30MHz)
        {
            /* Move FLEXSPI clock source from main clock to FRO192M / 7 to avoid instruction/data fetch issue in XIP when
             * updating PLL and main clock.
             */
            BOARD_SetFlexspiClock(EXAMPLE_FLEXSPI, 3U, 7U);
        }
        else if (clkFreq == kFlexspiRootClkFreq_50MHz)
        {
            /* Set FlexSPI clock: source AUX0_PLL£¨396MHz£©, divide by 8 */
            BOARD_SetFlexspiClock(EXAMPLE_FLEXSPI, 2U, 8U);
        }
        else if (clkFreq == kFlexspiRootClkFreq_60MHz)
        {
            /* Set FlexSPI clock: source AUX0_PLL£¨396MHz£©, divide by 7 */
            BOARD_SetFlexspiClock(EXAMPLE_FLEXSPI, 2U, 7U);
        }
        else if (clkFreq == kFlexspiRootClkFreq_80MHz)
        {
            /* Set FlexSPI clock: source AUX0_PLL£¨396MHz£©, divide by 5 */
            BOARD_SetFlexspiClock(EXAMPLE_FLEXSPI, 2U, 5U);
        }
        else if (clkFreq == kFlexspiRootClkFreq_100MHz)
        {
            /* Set FlexSPI clock: source AUX0_PLL£¨396MHz£©, divide by 4 */
            BOARD_SetFlexspiClock(EXAMPLE_FLEXSPI, 2U, 4U);
        }
        else if (clkFreq == kFlexspiRootClkFreq_120MHz)
        {
            /* Set FlexSPI clock: source AUX1_PLL£¨327MHz£©, divide by 3 */
            BOARD_SetFlexspiClock(EXAMPLE_FLEXSPI, 4U, 3U);
        }
        else if (clkFreq == kFlexspiRootClkFreq_133MHz)
        {
            /* Set FlexSPI clock: source AUX0_PLL£¨396MHz£©, divide by 3 */
            BOARD_SetFlexspiClock(EXAMPLE_FLEXSPI, 2U, 3U);
        }
        else if (clkFreq == kFlexspiRootClkFreq_166MHz)
        {
            /* Set FlexSPI clock: source AUX1_PLL£¨327MHz£©, divide by 2 */
            BOARD_SetFlexspiClock(EXAMPLE_FLEXSPI, 4U, 2U);
        }
        else if (clkFreq == kFlexspiRootClkFreq_200MHz)
        {
            /* Set FlexSPI clock: source AUX0_PLL£¨396MHz£©, divide by 2 */
            BOARD_SetFlexspiClock(EXAMPLE_FLEXSPI, 2U, 2U);
        }
        else if (clkFreq == kFlexspiRootClkFreq_332MHz)
        {
            /* Set FlexSPI clock: source AUX1_PLL£¨327MHz£©, divide by 1 */
            BOARD_SetFlexspiClock(EXAMPLE_FLEXSPI, 4U, 1U);
        }
        else if (clkFreq == kFlexspiRootClkFreq_400MHz)
        {
            /* Set FlexSPI clock: source AUX0_PLL£¨396MHz£©, divide by 1 */
            BOARD_SetFlexspiClock(EXAMPLE_FLEXSPI, 2U, 1U);
        }
        else
        {
            vra_printf("VRA: This FlexSPI clock freq is not set.\r\n");
        }
    }
    else if (base == FLEXSPI1)
    {
    }
    else
    {
    }
}

static uint32_t flexspi_get_clock(FLEXSPI_Type *base)
{
    if (base == FLEXSPI0)
    {
        return CLOCK_GetFlexspiClkFreq(0);
    }
    else if (base == FLEXSPI1)
    {
        return CLOCK_GetFlexspiClkFreq(1);
    }
    else
    {
        return 0;
    }
}

static void flexspi_show_clock_source(FLEXSPI_Type *base)
{
#if VRA_DEBUG_LOG_INFO_ENABLE
    uint32_t index = 0;
    uint32_t clkSel;
    uint32_t clkDiv;
    if (base == FLEXSPI0)
    {
        index = 0;
        clkSel = CLKCTL0->FLEXSPI0FCLKSEL & CLKCTL0_FLEXSPI0FCLKSEL_SEL_MASK;
        clkDiv = CLKCTL0->FLEXSPI0FCLKDIV & CLKCTL0_FLEXSPI0FCLKDIV_DIV_MASK;
        switch (clkSel)
        {
            case CLKCTL0_FLEXSPI0FCLKSEL_SEL(0):
                vra_printf("VRA: FLEXSPI0 Clk Source from 3'b000 - Main Clock %dHz.\r\n", CLOCK_GetMainClkFreq());
                break;

            case CLKCTL0_FLEXSPI0FCLKSEL_SEL(1):
               {
                   uint32_t MainPllClkFreq = CLOCK_GetSysPfdFreq(kCLOCK_Pfd0) / ((CLKCTL0->MAINPLLCLKDIV & CLKCTL0_MAINPLLCLKDIV_DIV_MASK) + 1U);
                   vra_printf("VRA: FLEXSPI0 Clk Source from 3'b001 - Main System PLL Clock %dHz.\r\n", MainPllClkFreq);
                   break;
               }

            case CLKCTL0_FLEXSPI0FCLKSEL_SEL(2):
               {
                   uint32_t Aux0PllClkFreq = CLOCK_GetSysPfdFreq(kCLOCK_Pfd2) / ((CLKCTL0->AUX0PLLCLKDIV & CLKCTL0_AUX0PLLCLKDIV_DIV_MASK) + 1U);
                   vra_printf("VRA: FLEXSPI0 Clk Source from 3'b010 - SYSPLL0 AUX0 PLL Clock %dHz.\r\n", Aux0PllClkFreq);
                   break;
               }

            case CLKCTL0_FLEXSPI0FCLKSEL_SEL(3):
                vra_printf("VRA: FLEXSPI0 Clk Source from 3'b011 - FRO_DIV1 Clock %dHz.\r\n", CLK_FRO_CLK);
                break;

            case CLKCTL0_FLEXSPI0FCLKSEL_SEL(4):
               {
                   uint32_t Aux1PllClkFreq = CLOCK_GetSysPfdFreq(kCLOCK_Pfd3) / ((CLKCTL0->AUX1PLLCLKDIV & CLKCTL0_AUX1PLLCLKDIV_DIV_MASK) + 1U);
                   vra_printf("VRA: FLEXSPI0 Clk Source from 3'b100 - SYSPLL0 AUX1 PLL Clock %dHz.\r\n", Aux1PllClkFreq);
                   break;
               }

            default:
                break;
        }
    }
    else if (base == FLEXSPI1)
    {
        index = 1;
        clkSel = CLKCTL0->FLEXSPI1FCLKSEL & CLKCTL0_FLEXSPI1FCLKSEL_SEL_MASK;
        clkDiv = CLKCTL0->FLEXSPI1FCLKDIV & CLKCTL0_FLEXSPI1FCLKDIV_DIV_MASK;
        switch (clkSel)
        {
            case CLKCTL0_FLEXSPI1FCLKSEL_SEL(0):
                vra_printf("VRA: FLEXSPI1 Clk Source from 3'b000 - Main Clock %dHz.\r\n", CLOCK_GetMainClkFreq());
                break;

            case CLKCTL0_FLEXSPI1FCLKSEL_SEL(1):
               {
                   uint32_t MainPllClkFreq = CLOCK_GetSysPfdFreq(kCLOCK_Pfd0) / ((CLKCTL0->MAINPLLCLKDIV & CLKCTL0_MAINPLLCLKDIV_DIV_MASK) + 1U);
                   vra_printf("VRA: FLEXSPI1 Clk Source from 3'b001 - Main System PLL Clock %dHz.\r\n", MainPllClkFreq);
                   break;
               }

            case CLKCTL0_FLEXSPI1FCLKSEL_SEL(2):
               {
                   uint32_t Aux0PllClkFreq = CLOCK_GetSysPfdFreq(kCLOCK_Pfd2) / ((CLKCTL0->AUX0PLLCLKDIV & CLKCTL0_AUX0PLLCLKDIV_DIV_MASK) + 1U);
                   vra_printf("VRA: FLEXSPI1 Clk Source from 3'b010 - SYSPLL0 AUX0 PLL Clock %dHz.\r\n", Aux0PllClkFreq);
                   break;
               }

            case CLKCTL0_FLEXSPI1FCLKSEL_SEL(3):
                vra_printf("VRA: FLEXSPI1 Clk Source from 3'b011 - FRO_DIV1 Clock %dHz.\r\n", CLK_FRO_CLK);
                break;

            case CLKCTL0_FLEXSPI1FCLKSEL_SEL(4):
               {
                   uint32_t Aux1PllClkFreq = CLOCK_GetSysPfdFreq(kCLOCK_Pfd3) / ((CLKCTL0->AUX1PLLCLKDIV & CLKCTL0_AUX1PLLCLKDIV_DIV_MASK) + 1U);
                   vra_printf("VRA: FLEXSPI1 Clk Source from 3'b100 - SYSPLL0 AUX1 PLL Clock %dHz.\r\n", Aux1PllClkFreq);
                   break;
               }

            default:
                break;
        }
    }
    else
    {}
    vra_printf("VRA: FLEXSPI%d Clk Source Divider: %d.\r\n", index, (clkDiv + 1U));
    vra_printf("VRA: FLEXSPI%d Clk Frequency: %dHz.\r\n", index, flexspi_get_clock(EXAMPLE_FLEXSPI));
#endif
}

#endif /* _PORT_FLEXSPI_INFO_H_ */
