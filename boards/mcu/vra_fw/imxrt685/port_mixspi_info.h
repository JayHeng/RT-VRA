/*
 * Copyright 2023 NXP
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
#include "fsl_iopctl.h"
#include "fsl_power.h"
#include "fsl_flexspi.h"
#include "pin_mux.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define VRA_MIXSPI_MODULE VRA_MIXSPI_MODULE_IS_FLEXSPI

#define EXAMPLE_MIXSPI                 FLEXSPI
#define EXAMPLE_CACHE                  CACHE64
#define EXAMPLE_MIXSPI_AMBA_BASE       FlexSPI_AMBA_BASE
#define EXAMPLE_MIXSPI_PORT            kFLEXSPI_PortA1
#define DRAM_SIZE                      0x800000U

/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void bsp_mixspi_init(void)
{
    cache64_config_t cacheCfg;

    POWER_DisablePD(kPDRUNCFG_APD_FLEXSPI_SRAM);
    POWER_DisablePD(kPDRUNCFG_PPD_FLEXSPI_SRAM);
    POWER_ApplyPD();

    CLOCK_AttachClk(kAUX0_PLL_to_FLEXSPI_CLK);
    CLOCK_SetClkDiv(kCLOCK_DivFlexspiClk, 1);

    RESET_PeripheralReset(kFLEXSPI_RST_SHIFT_RSTn);
    /* Explicitly enable FlexSPI clock for PSRAM loader case which need to set FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL. */
    CLOCK_EnableClock(kCLOCK_Flexspi);

    /* As cache depends on FlexSPI power and clock, cache must be initialized after FlexSPI power/clock is set */
    CACHE64_GetDefaultConfig(&cacheCfg);
    CACHE64_Init(CACHE64_POLSEL, &cacheCfg);
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
                    vra_printf("VRA: CPU Clk Source from MAINCLKSELA 2'b00 - FFRO clock (48/60m_irc) div4 Clock %dHz.\r\n", CLOCK_GetFFroFreq()/4);
                    break;
                case CLKCTL0_MAINCLKSELA_SEL(1):
                    vra_printf("VRA: CPU Clk Source from MAINCLKSELA 2'b01 - OSC clock %dHz.\r\n", CLOCK_GetXtalInClkFreq());
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
            vra_printf("VRA: CPU Clk Source from MAINCLKSELB 2'b10 - Main System PLL Clock %dHz.\r\n", CLOCK_GetSysPllFreq());
            break;

        case CLKCTL0_MAINCLKSELB_SEL(3):
        default:
            vra_printf("VRA: CPU Clk Source from MAINCLKSELB 2'b11 - RTC 32KHz Clock %dHz.\r\n", CLOCK_GetOsc32KFreq());
            break;
    }
    
    clkDiv = (CLKCTL0->SYSCPUAHBCLKDIV & CLKCTL0_SYSCPUAHBCLKDIV_DIV_MASK) >> CLKCTL0_SYSCPUAHBCLKDIV_DIV_SHIFT;
    vra_printf("VRA: CPU Clk Source Divider: %d.\r\n", (clkDiv + 1U));
    vra_printf("VRA: CPU Clk Frequency: %dHz.\r\n", CLOCK_GetFreq(kCLOCK_CoreSysClk));
#endif
}

static void mixspi_port_switch(FLEXSPI_Type *base, flexspi_port_t port, flexspi_pad_t pads)
{
}

static void mixspi_pin_init(FLEXSPI_Type *base, flexspi_port_t port, flexspi_pad_t pads)
{
    if (base == FLEXSPI)
    {
        const uint32_t port1_pin18_config = (/* Pin is configured as FLEXSPI0A_SCLK */
                                             IOPCTL_PIO_FUNC1 |
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
        /* PORT1 PIN18 (coords: T9) is configured as FLEXSPI0A_SCLK */
        IOPCTL_PinMuxSet(IOPCTL, 1U, 18U, port1_pin18_config);

        const uint32_t port1_pin19_config = (/* Pin is configured as FLEXSPI0A_SS0_N */
                                             IOPCTL_PIO_FUNC1 |
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
        /* PORT1 PIN19 (coords: T4) is configured as FLEXSPI0A_SS0_N */
        IOPCTL_PinMuxSet(IOPCTL, 1U, 19U, port1_pin19_config);

        const uint32_t port1_pin20_config = (/* Pin is configured as FLEXSPI0A_DATA0 */
                                             IOPCTL_PIO_FUNC1 |
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
        /* PORT1 PIN20 (coords: T5) is configured as FLEXSPI0A_DATA0 */
        IOPCTL_PinMuxSet(IOPCTL, 1U, 20U, port1_pin20_config);

        const uint32_t port1_pin21_config = (/* Pin is configured as FLEXSPI0A_DATA1 */
                                             IOPCTL_PIO_FUNC1 |
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
        /* PORT1 PIN21 (coords: U5) is configured as FLEXSPI0A_DATA1 */
        IOPCTL_PinMuxSet(IOPCTL, 1U, 21U, port1_pin21_config);

        const uint32_t port1_pin22_config = (/* Pin is configured as FLEXSPI0A_DATA2 */
                                             IOPCTL_PIO_FUNC1 |
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
        /* PORT1 PIN22 (coords: P6) is configured as FLEXSPI0A_DATA2 */
        IOPCTL_PinMuxSet(IOPCTL, 1U, 22U, port1_pin22_config);

        const uint32_t port1_pin23_config = (/* Pin is configured as FLEXSPI0A_DATA3 */
                                             IOPCTL_PIO_FUNC1 |
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
        /* PORT1 PIN23 (coords: P7) is configured as FLEXSPI0A_DATA3 */
        IOPCTL_PinMuxSet(IOPCTL, 1U, 23U, port1_pin23_config);

        const uint32_t port1_pin24_config = (/* Pin is configured as FLEXSPI0A_DATA4 */
                                             IOPCTL_PIO_FUNC1 |
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
        /* PORT1 PIN24 (coords: T7) is configured as FLEXSPI0A_DATA4 */
        IOPCTL_PinMuxSet(IOPCTL, 1U, 24U, port1_pin24_config);

        const uint32_t port1_pin25_config = (/* Pin is configured as FLEXSPI0A_DATA5 */
                                             IOPCTL_PIO_FUNC1 |
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
        /* PORT1 PIN25 (coords: U7) is configured as FLEXSPI0A_DATA5 */
        IOPCTL_PinMuxSet(IOPCTL, 1U, 25U, port1_pin25_config);

        const uint32_t port1_pin26_config = (/* Pin is configured as FLEXSPI0A_DATA6 */
                                             IOPCTL_PIO_FUNC1 |
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
        /* PORT1 PIN26 (coords: R7) is configured as FLEXSPI0A_DATA6 */
        IOPCTL_PinMuxSet(IOPCTL, 1U, 26U, port1_pin26_config);

        const uint32_t port1_pin27_config = (/* Pin is configured as FLEXSPI0A_DATA7 */
                                             IOPCTL_PIO_FUNC1 |
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
        /* PORT1 PIN27 (coords: T8) is configured as FLEXSPI0A_DATA7 */
        IOPCTL_PinMuxSet(IOPCTL, 1U, 27U, port1_pin27_config);

        const uint32_t port1_pin28_config = (/* Pin is configured as FLEXSPI0A_DQS */
                                             IOPCTL_PIO_FUNC1 |
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
        /* PORT1 PIN28 (coords: U9) is configured as FLEXSPI0A_DQS */
        IOPCTL_PinMuxSet(IOPCTL, 1U, 28U, port1_pin28_config);
    }
    else
    {
    }
}

static void mixspi_clock_init(FLEXSPI_Type *base, mixspi_root_clk_freq_t clkFreq)
{
    if (base == FLEXSPI)
    {
        if (clkFreq == kMixspiRootClkFreq_30MHz)
        {
            /* Move FLEXSPI clock source from main clock to FFRO to avoid instruction/data fetch issue in XIP when
             * updating PLL and main clock.
             */
            BOARD_SetFlexspiClock(3U, 1U);
        }
        else if (clkFreq == kMixspiRootClkFreq_50MHz)
        {
            /* Set FlexSPI clock: source AUX0_PLL£¨396MHz£©, divide by 8 */
            BOARD_SetFlexspiClock(2U, 8U);
        }
        else if (clkFreq == kMixspiRootClkFreq_60MHz)
        {
            /* Set FlexSPI clock: source AUX0_PLL£¨396MHz£©, divide by 7 */
            BOARD_SetFlexspiClock(2U, 7U);
        }
        else if (clkFreq == kMixspiRootClkFreq_80MHz)
        {
            /* Set FlexSPI clock: source AUX0_PLL£¨396MHz£©, divide by 5 */
            BOARD_SetFlexspiClock(2U, 5U);
        }
        else if (clkFreq == kMixspiRootClkFreq_100MHz)
        {
            /* Set FlexSPI clock: source AUX0_PLL£¨396MHz£©, divide by 4 */
            BOARD_SetFlexspiClock(2U, 4U);
        }
        else if (clkFreq == kMixspiRootClkFreq_120MHz)
        {
            /* Set FlexSPI clock: source AUX1_PLL£¨327MHz£©, divide by 3 */
            BOARD_SetFlexspiClock(4U, 3U);
        }
        else if (clkFreq == kMixspiRootClkFreq_133MHz)
        {
            /* Set FlexSPI clock: source AUX0_PLL£¨396MHz£©, divide by 3 */
            BOARD_SetFlexspiClock(2U, 3U);
        }
        else if (clkFreq == kMixspiRootClkFreq_166MHz)
        {
            /* Set FlexSPI clock: source AUX1_PLL£¨327MHz£©, divide by 2 */
            BOARD_SetFlexspiClock(4U, 2U);
        }
        else if (clkFreq == kMixspiRootClkFreq_200MHz)
        {
            /* Set FlexSPI clock: source AUX0_PLL£¨396MHz£©, divide by 2 */
            BOARD_SetFlexspiClock(2U, 2U);
        }
        else if (clkFreq == kMixspiRootClkFreq_332MHz)
        {
            /* Set FlexSPI clock: source AUX1_PLL£¨327MHz£©, divide by 1 */
            BOARD_SetFlexspiClock(4U, 1U);
        }
        else if (clkFreq == kMixspiRootClkFreq_400MHz)
        {
            /* Set FlexSPI clock: source AUX0_PLL£¨396MHz£©, divide by 1 */
            BOARD_SetFlexspiClock(2U, 1U);
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
               {
                   uint32_t MainPllClkFreq = CLOCK_GetSysPfdFreq(kCLOCK_Pfd0) / ((CLKCTL0->MAINPLLCLKDIV & CLKCTL0_MAINPLLCLKDIV_DIV_MASK) + 1U);
                   vra_printf("VRA: FLEXSPI0 Clk Source from 3'b001 - Main System PLL Clock %dHz.\r\n", MainPllClkFreq);
                   break;
               }

            case CLKCTL0_FLEXSPIFCLKSEL_SEL(2):
               {
                   uint32_t Aux0PllClkFreq = CLOCK_GetSysPfdFreq(kCLOCK_Pfd2) / ((CLKCTL0->AUX0PLLCLKDIV & CLKCTL0_AUX0PLLCLKDIV_DIV_MASK) + 1U);
                   vra_printf("VRA: FLEXSPI0 Clk Source from 3'b010 - SYSPLL0 AUX0 PLL Clock %dHz.\r\n", Aux0PllClkFreq);
                   break;
               }

            case CLKCTL0_FLEXSPIFCLKSEL_SEL(3):
                vra_printf("VRA: FLEXSPI0 Clk Source from 3'b011 - FFRO Clock %dHz.\r\n", CLOCK_GetFFroFreq());
                break;

            case CLKCTL0_FLEXSPIFCLKSEL_SEL(4):
               {
                   uint32_t Aux1PllClkFreq = CLOCK_GetSysPfdFreq(kCLOCK_Pfd3) / ((CLKCTL0->AUX1PLLCLKDIV & CLKCTL0_AUX1PLLCLKDIV_DIV_MASK) + 1U);
                   vra_printf("VRA: FLEXSPI0 Clk Source from 3'b100 - SYSPLL0 AUX1 PLL Clock %dHz.\r\n", Aux1PllClkFreq);
                   break;
               }

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
