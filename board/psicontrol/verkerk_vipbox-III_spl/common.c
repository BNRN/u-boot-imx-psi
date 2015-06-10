/*
 * Code used by both U-Boot and SPL for Compulab CM-FX6
 *
 * Copyright (C) 2014, Compulab Ltd - http://compulab.co.il/
 *
 * Author: Nikita Kiryanov <nikita@compulab.co.il>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <fsl_esdhc.h>
#include "common.h"

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_FSL_ESDHC

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)
	
static iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT6__GPIO6_IO18    | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};

static struct fsl_esdhc_cfg usdhc_cfg[1] = {
	{USDHC3_BASE_ADDR}
};
#define USDHC3_CD_GPIO	IMX_GPIO_NR(6, 18)

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	usdhc_cfg[0].max_bus_width = 4;
    imx_iomux_v3_setup_multiple_pads(usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
    gpio_direction_input(USDHC3_CD_GPIO);	
    
    status |= fsl_esdhc_initialize(bis, &usdhc_cfg[0]);    

	return status;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC3_BASE_ADDR:
		ret = gpio_get_value(USDHC3_CD_GPIO);
		break;
	}

	return ret;
}
#endif


void enable_ddr_led(unsigned number, unsigned on_off)
{
    iomux_v3_cfg_t config = (on_off) ? MUX_PAD_CTRL(WEAK_PULLDOWN) : MUX_PAD_CTRL(WEAK_PULLUP);

    switch(number)
    {
        case 0:
            // toggle the internal DDR LEDs by enabling/disabling pullup, pulldown
            imx_iomux_v3_setup_pad(MX6_PAD_NANDF_D0__GPIO2_IO00 | config);            
            break;
        case 1:
            imx_iomux_v3_setup_pad(MX6_PAD_NANDF_D5__GPIO2_IO05 | config);
            //gpio_direction_output(IMX_GPIO_NR(2, 5), on_off);
            break;
    }   
        
}
