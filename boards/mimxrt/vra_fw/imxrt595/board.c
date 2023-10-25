/*
 * Copyright 2018-2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "fsl_clock.h"
#include "fsl_flexspi.h"
#include "fsl_cache.h"
#include "fsl_power.h"
#include "clock_config.h"
#include "board.h"
#if defined(SDK_I2C_BASED_COMPONENT_USED) && SDK_I2C_BASED_COMPONENT_USED
#include "fsl_i2c.h"
#endif /* SDK_I2C_BASED_COMPONENT_USED */
#if defined BOARD_USE_CODEC
#include "fsl_i3c.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_FLEXSPI_DLL_LOCK_RETRY (10)
/*******************************************************************************
 * Variables
 ******************************************************************************/
AT_QUICKACCESS_SECTION_DATA(static uint32_t s_ispPin[3]);
AT_QUICKACCESS_SECTION_DATA(static uint32_t s_flexspiPin[10]);
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/* Initialize debug console. */
void BOARD_InitDebugConsole(void)
{
    uint32_t uartClkSrcFreq;

    /* attach FRG0 clock to FLEXCOMM0 (debug console) */
    CLOCK_SetFRGClock(BOARD_DEBUG_UART_FRG_CLK);
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    uartClkSrcFreq = BOARD_DEBUG_UART_CLK_FREQ;

    DbgConsole_Init(BOARD_DEBUG_UART_INSTANCE, BOARD_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE, uartClkSrcFreq);
}

void BOARD_DeinitFlash(FLEXSPI_Type *base)
{
    /* Enable FLEXSPI clock again */
    CLKCTL0->PSCCTL0_SET = CLKCTL0_PSCCTL0_SET_FLEXSPI0_OTFAD_CLK_MASK;

    /* Enable FLEXSPI module */
    base->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;

    /* Wait until FLEXSPI is not busy */
    while (!((base->STS0 & FLEXSPI_STS0_ARBIDLE_MASK) && (base->STS0 & FLEXSPI_STS0_SEQIDLE_MASK)))
    {
    }
    /* Disable module during the reset procedure */
    base->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;
}

void BOARD_InitFlash(FLEXSPI_Type *base)
{
    uint32_t status;
    uint32_t lastStatus;
    uint32_t retry;

    /* If serial root clock is >= 100 MHz, DLLEN set to 1, OVRDEN set to 0, then SLVDLYTARGET setting of 0x0 is
     * recommended. */
    base->DLLCR[0] = 0x1U;

    /* Enable FLEXSPI module */
    base->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;

    base->MCR0 |= FLEXSPI_MCR0_SWRESET_MASK;
    while (base->MCR0 & FLEXSPI_MCR0_SWRESET_MASK)
    {
    }

    /* Need to wait DLL locked if DLL enabled */
    if (0U != (base->DLLCR[0] & FLEXSPI_DLLCR_DLLEN_MASK))
    {
        lastStatus = base->STS2;
        retry      = BOARD_FLEXSPI_DLL_LOCK_RETRY;
        /* Wait slave delay line locked and slave reference delay line locked. */
        do
        {
            status = base->STS2;
            if ((status & (FLEXSPI_STS2_AREFLOCK_MASK | FLEXSPI_STS2_ASLVLOCK_MASK)) ==
                (FLEXSPI_STS2_AREFLOCK_MASK | FLEXSPI_STS2_ASLVLOCK_MASK))
            {
                /* Locked */
                retry = 100;
                break;
            }
            else if (status == lastStatus)
            {
                /* Same delay cell number in calibration */
                retry--;
            }
            else
            {
                retry      = BOARD_FLEXSPI_DLL_LOCK_RETRY;
                lastStatus = status;
            }
        } while (retry > 0);
        /* According to ERR011377, need to delay at least 100 NOPs to ensure the DLL is locked. */
        for (; retry > 0U; retry--)
        {
            __NOP();
        }
    }
}

/* BOARD_SetFlexspiClock run in RAM used to configure FlexSPI clock source and divider when XIP. */
void BOARD_SetFlexspiClock(FLEXSPI_Type *base, uint32_t src, uint32_t divider)
{
    if (base == FLEXSPI0)
    {
        if ((CLKCTL0->FLEXSPI0FCLKSEL != CLKCTL0_FLEXSPI0FCLKSEL_SEL(src)) ||
            ((CLKCTL0->FLEXSPI0FCLKDIV & CLKCTL0_FLEXSPI0FCLKDIV_DIV_MASK) != (divider - 1)))
        {
            /* Always deinit FLEXSPI and init FLEXSPI for the flash to make sure the flash works correctly after the
             FLEXSPI root clock changed as the default FLEXSPI configuration may does not work for the new root clock
             frequency. */
            BOARD_DeinitFlash(base);

            /* Disable clock before changing clock source */
            CLKCTL0->PSCCTL0_CLR = CLKCTL0_PSCCTL0_CLR_FLEXSPI0_OTFAD_CLK_MASK;
            /* Update flexspi clock. */
            CLKCTL0->FLEXSPI0FCLKSEL = CLKCTL0_FLEXSPI0FCLKSEL_SEL(src);
            CLKCTL0->FLEXSPI0FCLKDIV |= CLKCTL0_FLEXSPI0FCLKDIV_RESET_MASK; /* Reset the divider counter */
            CLKCTL0->FLEXSPI0FCLKDIV = CLKCTL0_FLEXSPI0FCLKDIV_DIV(divider - 1);
            while ((CLKCTL0->FLEXSPI0FCLKDIV) & CLKCTL0_FLEXSPI0FCLKDIV_REQFLAG_MASK)
            {
            }
            /* Enable FLEXSPI clock again */
            CLKCTL0->PSCCTL0_SET = CLKCTL0_PSCCTL0_SET_FLEXSPI0_OTFAD_CLK_MASK;

            BOARD_InitFlash(base);
        }
    }
    else if (base == FLEXSPI1)
    {
        if ((CLKCTL0->FLEXSPI1FCLKSEL != CLKCTL0_FLEXSPI1FCLKSEL_SEL(src)) ||
            ((CLKCTL0->FLEXSPI1FCLKDIV & CLKCTL0_FLEXSPI1FCLKDIV_DIV_MASK) != (divider - 1)))
        {
            /* Always deinit FLEXSPI and init FLEXSPI for the flash to make sure the flash works correctly after the
             FLEXSPI root clock changed as the default FLEXSPI configuration may does not work for the new root clock
             frequency. */
            BOARD_DeinitFlash(base);

            /* Disable clock before changing clock source */
            CLKCTL0->PSCCTL0_CLR = CLKCTL0_PSCCTL0_CLR_FLEXSPI1_CLK_MASK;
            /* Update flexspi clock. */
            CLKCTL0->FLEXSPI1FCLKSEL = CLKCTL0_FLEXSPI1FCLKSEL_SEL(src);
            CLKCTL0->FLEXSPI1FCLKDIV |= CLKCTL0_FLEXSPI1FCLKDIV_RESET_MASK; /* Reset the divider counter */
            CLKCTL0->FLEXSPI1FCLKDIV = CLKCTL0_FLEXSPI1FCLKDIV_DIV(divider - 1);
            while ((CLKCTL0->FLEXSPI1FCLKDIV) & CLKCTL0_FLEXSPI1FCLKDIV_REQFLAG_MASK)
            {
            }
            /* Enable FLEXSPI clock again */
            CLKCTL0->PSCCTL0_SET = CLKCTL0_PSCCTL0_SET_FLEXSPI1_CLK_MASK;

            BOARD_InitFlash(base);
        }
    }
    else
    {
        return;
    }
}

/* This function is used to change FlexSPI clock to a stable source before clock sources(Such as PLL and Main clock)
 * updating in case XIP(execute code on FLEXSPI memory.) */
void BOARD_FlexspiClockSafeConfig(void)
{
    /* Move FLEXSPI clock source from main clock to FRO192M / 2 to avoid instruction/data fetch issue in XIP when
     * updating PLL and main clock.
     */
    BOARD_SetFlexspiClock(FLEXSPI0, 3U, 2U);
}

void BOARD_SetDeepSleepPinConfig(void)
{
    /* Backup Pin configuration. */
    s_ispPin[0]     = IOPCTL->PIO[1][15];
    s_ispPin[1]     = IOPCTL->PIO[3][28];
    s_ispPin[2]     = IOPCTL->PIO[3][29];
    s_flexspiPin[0] = IOPCTL->PIO[1][18];
    s_flexspiPin[1] = IOPCTL->PIO[1][19];
    s_flexspiPin[2] = IOPCTL->PIO[1][20];
    s_flexspiPin[3] = IOPCTL->PIO[1][21];
    s_flexspiPin[4] = IOPCTL->PIO[1][22];
    s_flexspiPin[5] = IOPCTL->PIO[1][23];
    s_flexspiPin[6] = IOPCTL->PIO[1][24];
    s_flexspiPin[7] = IOPCTL->PIO[1][25];
    s_flexspiPin[8] = IOPCTL->PIO[1][26];
    s_flexspiPin[9] = IOPCTL->PIO[1][27];

    /* Disable ISP Pin pull-ups and input buffers to avoid current leakage */
    IOPCTL->PIO[1][15] = 0;
    IOPCTL->PIO[3][28] = 0;
    IOPCTL->PIO[3][29] = 0;

    /* Disable unnecessary input buffers */
    IOPCTL->PIO[1][18] &= ~IOPCTL_PIO_IBENA_MASK;
    IOPCTL->PIO[1][19] &= ~IOPCTL_PIO_IBENA_MASK;

    /* Enable pull-ups floating FlexSPI0 pins */
    IOPCTL->PIO[1][20] |= IOPCTL_PIO_PUPDENA_MASK | IOPCTL_PIO_PUPDSEL_MASK;
    IOPCTL->PIO[1][21] |= IOPCTL_PIO_PUPDENA_MASK | IOPCTL_PIO_PUPDSEL_MASK;
    IOPCTL->PIO[1][22] |= IOPCTL_PIO_PUPDENA_MASK | IOPCTL_PIO_PUPDSEL_MASK;
    IOPCTL->PIO[1][23] |= IOPCTL_PIO_PUPDENA_MASK | IOPCTL_PIO_PUPDSEL_MASK;
    IOPCTL->PIO[1][24] |= IOPCTL_PIO_PUPDENA_MASK | IOPCTL_PIO_PUPDSEL_MASK;
    IOPCTL->PIO[1][25] |= IOPCTL_PIO_PUPDENA_MASK | IOPCTL_PIO_PUPDSEL_MASK;
    IOPCTL->PIO[1][26] |= IOPCTL_PIO_PUPDENA_MASK | IOPCTL_PIO_PUPDSEL_MASK;
    IOPCTL->PIO[1][27] |= IOPCTL_PIO_PUPDENA_MASK | IOPCTL_PIO_PUPDSEL_MASK;
}

void BOARD_RestoreDeepSleepPinConfig(void)
{
    /* Restore the Pin configuration. */
    IOPCTL->PIO[1][15] = s_ispPin[0];
    IOPCTL->PIO[3][28] = s_ispPin[1];
    IOPCTL->PIO[3][29] = s_ispPin[2];

    IOPCTL->PIO[1][18] = s_flexspiPin[0];
    IOPCTL->PIO[1][19] = s_flexspiPin[1];
    IOPCTL->PIO[1][20] = s_flexspiPin[2];
    IOPCTL->PIO[1][21] = s_flexspiPin[3];
    IOPCTL->PIO[1][22] = s_flexspiPin[4];
    IOPCTL->PIO[1][23] = s_flexspiPin[5];
    IOPCTL->PIO[1][24] = s_flexspiPin[6];
    IOPCTL->PIO[1][25] = s_flexspiPin[7];
    IOPCTL->PIO[1][26] = s_flexspiPin[8];
    IOPCTL->PIO[1][27] = s_flexspiPin[9];
}

void BOARD_EnterDeepSleep(const uint32_t exclude_from_pd[4])
{
    BOARD_SetDeepSleepPinConfig();
    POWER_EnterDeepSleep(exclude_from_pd);
    BOARD_RestoreDeepSleepPinConfig();
}

void BOARD_EnterDeepPowerDown(const uint32_t exclude_from_pd[4])
{
    BOARD_SetDeepSleepPinConfig();
    POWER_EnterDeepPowerDown(exclude_from_pd);
    /* After deep power down wakeup, the code will restart and cannot reach here. */
    BOARD_RestoreDeepSleepPinConfig();
}

#if defined(SDK_I2C_BASED_COMPONENT_USED) && SDK_I2C_BASED_COMPONENT_USED
void BOARD_I2C_Init(I2C_Type *base, uint32_t clkSrc_Hz)
{
    i2c_master_config_t i2cConfig = {0};

    I2C_MasterGetDefaultConfig(&i2cConfig);
    I2C_MasterInit(base, &i2cConfig, clkSrc_Hz);
}

status_t BOARD_I2C_Send(I2C_Type *base,
                        uint8_t deviceAddress,
                        uint32_t subAddress,
                        uint8_t subaddressSize,
                        uint8_t *txBuff,
                        uint8_t txBuffSize)
{
    i2c_master_transfer_t masterXfer;

    /* Prepare transfer structure. */
    masterXfer.slaveAddress   = deviceAddress;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = subAddress;
    masterXfer.subaddressSize = subaddressSize;
    masterXfer.data           = txBuff;
    masterXfer.dataSize       = txBuffSize;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    return I2C_MasterTransferBlocking(base, &masterXfer);
}

status_t BOARD_I2C_Receive(I2C_Type *base,
                           uint8_t deviceAddress,
                           uint32_t subAddress,
                           uint8_t subaddressSize,
                           uint8_t *rxBuff,
                           uint8_t rxBuffSize)
{
    i2c_master_transfer_t masterXfer;

    /* Prepare transfer structure. */
    masterXfer.slaveAddress   = deviceAddress;
    masterXfer.subaddress     = subAddress;
    masterXfer.subaddressSize = subaddressSize;
    masterXfer.data           = rxBuff;
    masterXfer.dataSize       = rxBuffSize;
    masterXfer.direction      = kI2C_Read;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    return I2C_MasterTransferBlocking(base, &masterXfer);
}
#endif

#if defined BOARD_USE_CODEC
void BOARD_I3C_Init(I3C_Type *base, uint32_t clkSrc_Hz)
{
    i3c_master_config_t i3cConfig;

    I3C_MasterGetDefaultConfig(&i3cConfig);
    I3C_MasterInit(base, &i3cConfig, clkSrc_Hz);
}

status_t BOARD_I3C_Send(I3C_Type *base,
                        uint8_t deviceAddress,
                        uint32_t subAddress,
                        uint8_t subaddressSize,
                        uint8_t *txBuff,
                        uint8_t txBuffSize)
{
    i3c_master_transfer_t masterXfer;

    /* Prepare transfer structure. */
    masterXfer.slaveAddress   = deviceAddress;
    masterXfer.direction      = kI3C_Write;
    masterXfer.busType        = kI3C_TypeI2C;
    masterXfer.subaddress     = subAddress;
    masterXfer.subaddressSize = subaddressSize;
    masterXfer.data           = txBuff;
    masterXfer.dataSize       = txBuffSize;
    masterXfer.flags          = kI3C_TransferDefaultFlag;
    masterXfer.busType        = kI3C_TypeI2C;

    return I3C_MasterTransferBlocking(base, &masterXfer);
}

status_t BOARD_I3C_Receive(I3C_Type *base,
                           uint8_t deviceAddress,
                           uint32_t subAddress,
                           uint8_t subaddressSize,
                           uint8_t *rxBuff,
                           uint8_t rxBuffSize)
{
    i3c_master_transfer_t masterXfer;

    /* Prepare transfer structure. */
    masterXfer.slaveAddress   = deviceAddress;
    masterXfer.subaddress     = subAddress;
    masterXfer.subaddressSize = subaddressSize;
    masterXfer.data           = rxBuff;
    masterXfer.dataSize       = rxBuffSize;
    masterXfer.direction      = kI3C_Read;
    masterXfer.busType        = kI3C_TypeI2C;
    masterXfer.flags          = kI3C_TransferDefaultFlag;
    masterXfer.busType        = kI3C_TypeI2C;

    return I3C_MasterTransferBlocking(base, &masterXfer);
}

void BOARD_Codec_I2C_Init(void)
{
#if BOARD_I3C_CODEC
    BOARD_I3C_Init(BOARD_CODEC_I2C_BASEADDR, BOARD_CODEC_I2C_CLOCK_FREQ);
#else
    BOARD_I2C_Init(BOARD_CODEC_I2C_BASEADDR, BOARD_CODEC_I2C_CLOCK_FREQ);
#endif
}

status_t BOARD_Codec_I2C_Send(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, const uint8_t *txBuff, uint8_t txBuffSize)
{
#if BOARD_I3C_CODEC
    return BOARD_I3C_Send(BOARD_CODEC_I2C_BASEADDR, deviceAddress, subAddress, subAddressSize, (uint8_t *)txBuff,
#else
    return BOARD_I2C_Send(BOARD_CODEC_I2C_BASEADDR, deviceAddress, subAddress, subAddressSize, (uint8_t *)txBuff,
#endif
                          txBuffSize);
}

status_t BOARD_Codec_I2C_Receive(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, uint8_t *rxBuff, uint8_t rxBuffSize)
{
#if BOARD_I3C_CODEC
    return BOARD_I3C_Receive(BOARD_CODEC_I2C_BASEADDR, deviceAddress, subAddress, subAddressSize, rxBuff, rxBuffSize);
#else
    return BOARD_I2C_Receive(BOARD_CODEC_I2C_BASEADDR, deviceAddress, subAddress, subAddressSize, rxBuff, rxBuffSize);
#endif
}
#endif

#if defined(SDK_I2C_BASED_COMPONENT_USED) && SDK_I2C_BASED_COMPONENT_USED
void BOARD_PMIC_I2C_Init(void)
{
    BOARD_I2C_Init(BOARD_PMIC_I2C_BASEADDR, BOARD_PMIC_I2C_CLOCK_FREQ);
}

status_t BOARD_PMIC_I2C_Send(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, const uint8_t *txBuff, uint8_t txBuffSize)
{
    return BOARD_I2C_Send(BOARD_PMIC_I2C_BASEADDR, deviceAddress, subAddress, subAddressSize, (uint8_t *)txBuff,
                          txBuffSize);
}

status_t BOARD_PMIC_I2C_Receive(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, uint8_t *rxBuff, uint8_t rxBuffSize)
{
    return BOARD_I2C_Receive(BOARD_PMIC_I2C_BASEADDR, deviceAddress, subAddress, subAddressSize, rxBuff, rxBuffSize);
}

void BOARD_MIPIPanelTouch_I2C_Init(void)
{
    BOARD_I2C_Init(BOARD_MIPI_PANEL_TOUCH_I2C_BASEADDR, BOARD_MIPI_PANEL_TOUCH_I2C_CLOCK_FREQ);
}

status_t BOARD_MIPIPanelTouch_I2C_Send(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, const uint8_t *txBuff, uint8_t txBuffSize)
{
    return BOARD_I2C_Send(BOARD_MIPI_PANEL_TOUCH_I2C_BASEADDR, deviceAddress, subAddress, subAddressSize,
                          (uint8_t *)txBuff, txBuffSize);
}

status_t BOARD_MIPIPanelTouch_I2C_Receive(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, uint8_t *rxBuff, uint8_t rxBuffSize)
{
    return BOARD_I2C_Receive(BOARD_MIPI_PANEL_TOUCH_I2C_BASEADDR, deviceAddress, subAddress, subAddressSize, rxBuff,
                             rxBuffSize);
}

void BOARD_Accel_I2C_Init(void)
{
    BOARD_I2C_Init(BOARD_ACCEL_I2C_BASEADDR, BOARD_ACCEL_I2C_CLOCK_FREQ);
}

status_t BOARD_Accel_I2C_Send(uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, uint32_t txBuff)
{
    uint8_t data = (uint8_t)txBuff;
    return BOARD_I2C_Send(BOARD_ACCEL_I2C_BASEADDR, deviceAddress, subAddress, subAddressSize, &data, 1);
}

status_t BOARD_Accel_I2C_Receive(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, uint8_t *rxBuff, uint8_t rxBuffSize)
{
    return BOARD_I2C_Receive(BOARD_ACCEL_I2C_BASEADDR, deviceAddress, subAddress, subAddressSize, rxBuff, rxBuffSize);
}
#endif /* SDK_I2C_BASED_COMPONENT_USED */
