/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "vra_pseudo_sram.h"
#include "fsl_debug_console.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t s_psram_write_buffer[1024];
static uint8_t s_psram_read_buffer[1024];

/* Main PSRAM paramenter structure */
psram_property_info_t g_psramPropertyInfo;

/*******************************************************************************
 * Code
 ******************************************************************************/

int vra_printf(const char *fmt_s, ...)
{
#if VRA_DEBUG_LOG_INFO_ENABLE
    PRINTF(fmt_s);
#endif
    
    return 0;
}

void vra_main(void)
{
    vra_printf("VRA: i.MXRT Various Ram Access solution.\r\n");
    vra_printf("VRA: Get CPU root clock.\r\n");
    /* Show CPU clock source */
    cpu_show_clock_source();

#if APMEMORY_DEVICE_SERIES
    vra_psram_set_param_for_apmemory();
#elif ISSI_DEVICE_SERIES
    vra_psram_set_param_for_issi();
#elif WINBOND_DEVICE_SERIES
    vra_psram_set_param_for_winbond();
#elif INFINEON_DEVICE_SERIES
    vra_psram_set_param_for_infineon();
#endif

    vra_printf("\r\nVRA: Set FlexSPI port to %d-bit pad.\r\n", 1u << (uint32_t)g_psramPropertyInfo.mixspiPad);
    /* Init FlexSPI pinmux */
    mixspi_pin_init(EXAMPLE_MIXSPI, EXAMPLE_MIXSPI_PORT, g_psramPropertyInfo.mixspiPad);
    vra_printf("VRA: Set FlexSPI root clock to %dMHz.\r\n", decode_mixspi_root_clk_defn(g_psramPropertyInfo.mixspiRootClkFreq));
    /* Set FlexSPI root clock */
    mixspi_clock_init(EXAMPLE_MIXSPI, g_psramPropertyInfo.mixspiRootClkFreq);
    /* Show FlexSPI clock source */
    mixspi_show_clock_source(EXAMPLE_MIXSPI);

    status_t status = mixspi_psram_init(EXAMPLE_MIXSPI, g_psramPropertyInfo.mixspiCustomLUTVendor, g_psramPropertyInfo.mixspiReadSampleClock);
    if (status != kStatus_Success)
    {
        assert(false);
    }
    vra_printf("VRA: FLEXSPI module is initialized.\r\n");

#if APMEMORY_DEVICE_SERIES
    status = vra_psram_set_registers_for_apmemory(EXAMPLE_MIXSPI);
#elif ISSI_DEVICE_SERIES
    status = vra_psram_set_registers_for_issi(EXAMPLE_MIXSPI);
#elif WINBOND_DEVICE_SERIES
    status = vra_psram_set_registers_for_winbond(EXAMPLE_MIXSPI);
#elif INFINEON_DEVICE_SERIES
    status = vra_psram_set_registers_for_infineon(EXAMPLE_MIXSPI);
#endif
    if (status != kStatus_Success)
    {
        assert(false);
    }

    vra_printf("FLEXSPI example started!\r\n");

    uint32_t i  = 0;
    for (i = 0; i < sizeof(s_psram_write_buffer); i++)
    {
        s_psram_write_buffer[i] = i;
    }

    /* IP command write/read, should notice that the start address should be even address and the write address/size
     * should be 1024 aligned.*/
    for (i = 0; i < DRAM_SIZE; i += 1024)
    {
        status = mixspi_psram_ipcommand_write_data(EXAMPLE_MIXSPI, i, (uint32_t *)s_psram_write_buffer,
                                                    sizeof(s_psram_write_buffer));

        if (status != kStatus_Success)
        {
            status = kStatus_Fail;
            vra_printf("IPG Command Write data Failure at 0x%x!\r\n", i);
        }

        status = mixspi_psram_ipcommand_read_data(EXAMPLE_MIXSPI, i, (uint32_t *)s_psram_read_buffer,
                                                   sizeof(s_psram_read_buffer));
        if (status != kStatus_Success)
        {
            status = kStatus_Fail;
            vra_printf("IPG Command Read data Failure at 0x%x!\r\n", i);
        }

        if (memcmp(s_psram_read_buffer, s_psram_write_buffer, sizeof(s_psram_write_buffer)) != 0)
        {
            vra_printf("IPG Command Read/Write data Failure at 0x%x - 0x%x!\r\n", i, i + 1023);
            return;
        }
    }

    vra_printf("IPG Command Read/Write data succeed at all address range !\r\n");

    /* Need to reset FlexSPI controller between IP/AHB access. */
    FLEXSPI_SoftwareReset(EXAMPLE_MIXSPI);

    for (i = 0; i < sizeof(s_psram_write_buffer); i++)
    {
        s_psram_write_buffer[i] = (i + 0xFFU);
    }

    memset(s_psram_read_buffer, 0, sizeof(s_psram_read_buffer));

    for (i = 0; i < DRAM_SIZE; i += 1024)
    {
        mixspi_psram_ahbcommand_write_data(EXAMPLE_MIXSPI, i, (uint32_t *)s_psram_write_buffer,
                                                sizeof(s_psram_write_buffer));
        mixspi_psram_ahbcommand_read_data(EXAMPLE_MIXSPI, i, (uint32_t *)s_psram_read_buffer,
                                               sizeof(s_psram_write_buffer));

        if (memcmp(s_psram_read_buffer, s_psram_write_buffer, sizeof(s_psram_write_buffer)) != 0)
        {
            vra_printf("AHB Command Read/Write data Failure at 0x%x - 0x%x!\r\n", i, i + 1023);
            return;
        }
    }

    for (i = 0; i < sizeof(s_psram_write_buffer); i++)
    {
        s_psram_write_buffer[i] = i;
    }
    memset(s_psram_read_buffer, 0, sizeof(s_psram_read_buffer));

    for (i = 1; i < DRAM_SIZE - 1024; i += 1024)
    {
        mixspi_psram_ahbcommand_write_data(EXAMPLE_MIXSPI, i, (uint32_t *)s_psram_write_buffer,
                                                sizeof(s_psram_write_buffer));
        mixspi_psram_ahbcommand_read_data(EXAMPLE_MIXSPI, i, (uint32_t *)s_psram_read_buffer,
                                               sizeof(s_psram_read_buffer));

        if (memcmp(s_psram_read_buffer, s_psram_write_buffer, sizeof(s_psram_write_buffer)) != 0)
        {
            vra_printf("AHB Command Read/Write data Failure at 0x%x - 0x%x!\r\n", i, i + 1023);
            return;
        }
    }

    for (i = 0; i < sizeof(s_psram_write_buffer); i++)
    {
        s_psram_write_buffer[i] = (i + 0xFFU);
    }
    memset(s_psram_read_buffer, 0, sizeof(s_psram_read_buffer));

    for (i = 2; i < DRAM_SIZE - 1024; i += 1024)
    {
        mixspi_psram_ahbcommand_write_data(EXAMPLE_MIXSPI, i, (uint32_t *)s_psram_write_buffer,
                                                sizeof(s_psram_write_buffer));
        mixspi_psram_ahbcommand_read_data(EXAMPLE_MIXSPI, i, (uint32_t *)s_psram_read_buffer,
                                               sizeof(s_psram_read_buffer));

        if (memcmp(s_psram_read_buffer, s_psram_write_buffer, sizeof(s_psram_write_buffer)) != 0)
        {
            vra_printf("AHB Command Read/Write data Failure at 0x%x - 0x%x!\r\n", i, i + 1023);
            return;
        }
    }

    for (i = 0; i < sizeof(s_psram_write_buffer); i++)
    {
        s_psram_write_buffer[i] = i;
    }
    memset(s_psram_read_buffer, 0, sizeof(s_psram_read_buffer));

    for (i = 3; i < DRAM_SIZE - 1024; i += 1024)
    {
        mixspi_psram_ahbcommand_write_data(EXAMPLE_MIXSPI, i, (uint32_t *)s_psram_write_buffer,
                                                sizeof(s_psram_write_buffer));
        mixspi_psram_ahbcommand_read_data(EXAMPLE_MIXSPI, i, (uint32_t *)s_psram_read_buffer,
                                               sizeof(s_psram_read_buffer));

        if (memcmp(s_psram_read_buffer, s_psram_write_buffer, sizeof(s_psram_write_buffer)) != 0)
        {
            vra_printf("AHB Command Read/Write data Failure at 0x%x - 0x%x!\r\n", i, i + 1023);
            return;
        }
    }

    vra_printf("AHB Command Read/Write data succeed at all address range !\r\n");
}
