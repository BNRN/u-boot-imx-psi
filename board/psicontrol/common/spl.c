/*
 * SPL specific code for Compulab CM-FX6 board
 *
 * Copyright (C) 2014, Compulab Ltd - http://compulab.co.il/
 *
 * Author: Nikita Kiryanov <nikita@compulab.co.il>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <spl.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/mx6-ddr.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/crm_regs.h>
#include <asm/imx-common/iomux-v3.h>
#include <fsl_esdhc.h>
#include <watchdog.h>
#include "../common/common.h"

DECLARE_GLOBAL_DATA_PTR;

extern int memory_regions_post_test(int flags);

/* debug */
static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

/* testconnector */
static iomux_v3_cfg_t const uart2_pads[] = {
	MX6_PAD_EIM_D26__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D27__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

/* captouch */
static iomux_v3_cfg_t const uart3_pads[] = {
	MX6_PAD_EIM_D24__UART3_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D25__UART3_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};


static void ihmi_spl_setup_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
	imx_iomux_v3_setup_multiple_pads(uart3_pads, ARRAY_SIZE(uart3_pads));
	enable_uart_clk(1);
}


void board_init_f(ulong dummy)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	enable_led(0, 1);	
	enable_led(1, 1);
	
    int ret, i;
	
	gd = &gdata;
	/*
	 * We don't use DMA in SPL, but we do need it in U-Boot. U-Boot
	 * initializes DMA very early (before all board code), so the only
	 * opportunity we have to initialize APBHDMA clocks is in SPL.
	 */
	setbits_le32(&mxc_ccm->CCGR0, MXC_CCM_CCGR0_APBHDMA_MASK);
	enable_usdhc_clk(1, 2);

	arch_cpu_init();
	timer_init();
	ihmi_spl_setup_spi();
	ihmi_spl_setup_uart();
	get_clocks();
	preloader_console_init();

    puts("calling memory test!\n");
    
    ret = memory_regions_post_test(0);
    
    if (!ret)
    {    
        puts("memory test ok, calling board_init_r!\n");
        
        memset(__bss_start, 0, __bss_end - __bss_start);
        board_init_r(NULL, 0);
    }
    else
    {
        puts("memory test failed!");
    
        while (1)
        {            
            for (i = 0; i < 10; i++)
            {        
                WATCHDOG_RESET();    
                
                enable_led(0, i % 2);	
                enable_led(1, 1 - (i % 2));
                
                udelay(100000);
            }
            
        }
    }
}

void spl_board_init(void)
{
	u32 boot_device = spl_boot_device();

	if (boot_device == BOOT_DEVICE_SPI)
		puts("Booting from SPI flash\n");
	else if (boot_device == BOOT_DEVICE_MMC1)
		puts("Booting from MMC\n");
	else
		puts("Unknown boot device\n");
}
