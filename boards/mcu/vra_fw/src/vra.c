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

    vra_psram_set_param_for_apmemory();

    vra_printf("\r\nVRA: Set FlexSPI port to %d-bit pad.\r\n", 1u << (uint32_t)g_psramPropertyInfo.mixspiPad);
    /* Init FlexSPI pinmux */
    mixspi_pin_init(EXAMPLE_MIXSPI, EXAMPLE_MIXSPI_PORT, g_psramPropertyInfo.mixspiPad);

    uint32_t i  = 0;
    status_t st = kStatus_Success;

    status_t status = mixspi_psram_init(EXAMPLE_MIXSPI, g_psramPropertyInfo.mixspiCustomLUTVendor, g_psramPropertyInfo.mixspiReadSampleClock);
    if (status != kStatus_Success)
    {
        assert(false);
    }
    vra_printf("VRA: FLEXSPI module is initialized to xccela mode.\r\n");

    /* Show FlexSPI clock source */
    mixspi_show_clock_source(EXAMPLE_MIXSPI);

    PRINTF("FLEXSPI example started!\r\n");

    for (i = 0; i < sizeof(s_psram_write_buffer); i++)
    {
        s_psram_write_buffer[i] = i;
    }

    /* IP command write/read, should notice that the start address should be even address and the write address/size
     * should be 1024 aligned.*/
    for (i = 0; i < DRAM_SIZE; i += 1024)
    {
        st = mixspi_psram_ipcommand_write_data(EXAMPLE_MIXSPI, i, (uint32_t *)s_psram_write_buffer,
                                                    sizeof(s_psram_write_buffer));

        if (st != kStatus_Success)
        {
            st = kStatus_Fail;
            PRINTF("IP Command Write data Failure at 0x%x!\r\n", i);
        }

        st = mixspi_psram_ipcommand_read_data(EXAMPLE_MIXSPI, i, (uint32_t *)s_psram_read_buffer,
                                                   sizeof(s_psram_read_buffer));
        if (st != kStatus_Success)
        {
            st = kStatus_Fail;
            PRINTF("IP Command Read data Failure at 0x%x!\r\n", i);
        }

        if (memcmp(s_psram_read_buffer, s_psram_write_buffer, sizeof(s_psram_write_buffer)) != 0)
        {
            PRINTF("IP Command Read/Write data Failure at 0x%x - 0x%x!\r\n", i, i + 1023);
            return;
        }
    }

    PRINTF("IP Command Read/Write data succeed at all address range !\r\n");

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
            PRINTF("AHB Command Read/Write data Failure at 0x%x - 0x%x!\r\n", i, i + 1023);
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
            PRINTF("AHB Command Read/Write data Failure at 0x%x - 0x%x!\r\n", i, i + 1023);
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
            PRINTF("AHB Command Read/Write data Failure at 0x%x - 0x%x!\r\n", i, i + 1023);
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
            PRINTF("AHB Command Read/Write data Failure at 0x%x - 0x%x!\r\n", i, i + 1023);
            return;
        }
    }

    PRINTF("AHB Command Read/Write data succeed at all address range !\r\n");
}
