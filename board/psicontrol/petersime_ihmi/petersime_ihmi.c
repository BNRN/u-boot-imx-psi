/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>
#include <malloc.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/sata.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/video.h>
#include <pwm.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <i2c.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include <input.h>
#include <netdev.h>
#include <usb/ehci-fsl.h>

DECLARE_GLOBAL_DATA_PTR;
#define GP_USB_OTG_PWR	IMX_GPIO_NR(3, 22)

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define ENET_PAD_CTRL_PULLDOWN  (PAD_CTL_PUS_100K_DOWN |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)
	
#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define BUTTON_PAD_CTRL (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define WEAK_PULLUP	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_SRE_SLOW)

#define WEAK_PULLDOWN	(PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define OUTPUT_40OHM (PAD_CTL_SPEED_MED|PAD_CTL_DSE_40ohm)


int dram_init(void)
{
	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);

	return 0;
}

iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const uart2_pads[] = {
	MX6_PAD_EIM_D26__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D27__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

/* I2C1, SGTL5000 */
/*static struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6_PAD_EIM_D21__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_EIM_D21__GPIO3_IO21 | PC,
		.gp = IMX_GPIO_NR(3, 21)
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D28__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_EIM_D28__GPIO3_IO28 | PC,
		.gp = IMX_GPIO_NR(3, 28)
	}
};*/

/* I2C2 PMIC */
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | PC,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | PC,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | PC,
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | PC,
		.gp = IMX_GPIO_NR(4, 13)
	}
};

/* I2C3, J15 - RGB connector */
/*static struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO_5__I2C3_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO_5__GPIO1_IO05 | PC,
		.gp = IMX_GPIO_NR(1, 5)
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO_16__I2C3_SDA | PC,
		.gpio_mode = MX6_PAD_GPIO_16__GPIO7_IO11 | PC,
		.gp = IMX_GPIO_NR(7, 11)
	}
};*/

static iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT6__GPIO6_IO18    | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};

static iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D6__GPIO2_IO06    | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};




static iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TXD0__ENET_TX_DATA0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TXD1__ENET_TX_DATA1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TX_EN__ENET_TX_EN	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RX_ER__ENET_RX_ER	| MUX_PAD_CTRL(ENET_PAD_CTRL_PULLDOWN),
	MX6_PAD_ENET_RXD0__ENET_RX_DATA0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RXD1__ENET_RX_DATA1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_CRS_DV__ENET_RX_EN	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO_19__ENET_TX_ER	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO_16__ENET_REF_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	
	/* Micrel KS8873 switch Reset */
	MX6_PAD_NANDF_CS0__GPIO6_IO11	| MUX_PAD_CTRL(NO_PAD_CTRL),
	
};



static iomux_v3_cfg_t const misc_pads[] = {
	MX6_PAD_GPIO_1__USB_OTG_ID		| MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_EIM_D21__USB_OTG_OC		| MUX_PAD_CTRL(WEAK_PULLUP),
	/* OTG Power enable */
	MX6_PAD_EIM_D22__GPIO3_IO22		| MUX_PAD_CTRL(OUTPUT_40OHM)
};

/* wl1271 pads on nitrogen6x */
static iomux_v3_cfg_t const wl12xx_pads[] = {
	(MX6_PAD_NANDF_CS1__GPIO6_IO14 & ~MUX_PAD_CTRL_MASK)
		| MUX_PAD_CTRL(WEAK_PULLDOWN),
	(MX6_PAD_NANDF_CS2__GPIO6_IO15 & ~MUX_PAD_CTRL_MASK)
		| MUX_PAD_CTRL(OUTPUT_40OHM),
	(MX6_PAD_NANDF_CS3__GPIO6_IO16 & ~MUX_PAD_CTRL_MASK)
		| MUX_PAD_CTRL(OUTPUT_40OHM),
};
#define WL12XX_WL_IRQ_GP	IMX_GPIO_NR(6, 14)
#define WL12XX_WL_ENABLE_GP	IMX_GPIO_NR(6, 15)
#define WL12XX_BT_ENABLE_GP	IMX_GPIO_NR(6, 16)

// /* Button assignments for J14 */
// static iomux_v3_cfg_t const button_pads[] = {
	// /* Menu */
	// MX6_PAD_NANDF_D1__GPIO2_IO01	| MUX_PAD_CTRL(BUTTON_PAD_CTRL),
	// /* Back */
	// MX6_PAD_NANDF_D2__GPIO2_IO02	| MUX_PAD_CTRL(BUTTON_PAD_CTRL),
	// /* Labelled Search (mapped to Power under Android) */
	// MX6_PAD_NANDF_D3__GPIO2_IO03	| MUX_PAD_CTRL(BUTTON_PAD_CTRL),
	// /* Home */
	// MX6_PAD_NANDF_D4__GPIO2_IO04	| MUX_PAD_CTRL(BUTTON_PAD_CTRL),
	// /* Volume Down */
	// MX6_PAD_GPIO_19__GPIO4_IO05	| MUX_PAD_CTRL(BUTTON_PAD_CTRL),
	// /* Volume Up */
	// MX6_PAD_GPIO_18__GPIO7_IO13	| MUX_PAD_CTRL(BUTTON_PAD_CTRL),
// };

static void setup_iomux_enet(void)
{

	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
	/* Reset Micrel KS8873 switch */
	gpio_direction_output(IMX_GPIO_NR(6, 11) , 0);
	udelay(500);
	gpio_set_value(IMX_GPIO_NR(6, 11), 1);

}

static iomux_v3_cfg_t const usb_pads[] = {
	MX6_PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL),
    /* USB Host 1 overcurrent detection */
	MX6_PAD_EIM_D30__USB_H1_OC		| MUX_PAD_CTRL(WEAK_PULLUP),

};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
}

static iomux_v3_cfg_t const led_pads[] = {
	MX6_PAD_NANDF_D0__GPIO2_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D5__GPIO2_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL),
	
};

static void setup_iomux_leds(void)
{
	imx_iomux_v3_setup_multiple_pads(led_pads, ARRAY_SIZE(led_pads));
}



#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	imx_iomux_v3_setup_multiple_pads(usb_pads, ARRAY_SIZE(usb_pads));

	/* Reset USB hub */
	gpio_direction_output(IMX_GPIO_NR(7, 12), 0);
	mdelay(2);
	gpio_set_value(IMX_GPIO_NR(7, 12), 1);

	return 0;
}

int board_ehci_power(int port, int on)
{
	if (port)
		return 0;
	gpio_set_value(GP_USB_OTG_PWR, on);
	return 0;
}

#endif

#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};


#define USDHC3_CD_GPIO	IMX_GPIO_NR(6, 18)


int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC3_BASE_ADDR:
		ret = !gpio_get_value(USDHC3_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return ret;
}


int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	u32 index = 0;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

	usdhc_cfg[0].max_bus_width = 4;
	usdhc_cfg[1].max_bus_width = 8;

	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			gpio_direction_input(USDHC3_CD_GPIO);	
			break;
			
		case 1:
		       imx_iomux_v3_setup_multiple_pads(
			       usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
		       break;
		default:
		       printf("Warning: you configured more USDHC controllers"
			       "(%d) than supported by the board (%d)\n",
			       index + 1, CONFIG_SYS_FSL_USDHC_NUM);
		       return status;
		}

		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
	}

	return status;
}
#endif

#ifdef CONFIG_MXC_SPI

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 2 && cs == 1) ? (IMX_GPIO_NR(4, 24)) : -1;
}


static iomux_v3_cfg_t const ecspi3_pads[] = {
    /* SS0 */
	MX6_PAD_DISP0_DAT3__GPIO4_IO24  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT4__ECSPI3_SS1  | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT2__ECSPI3_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT1__ECSPI3_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT0__ECSPI3_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};




static void setup_spi(void)
{
	imx_iomux_v3_setup_multiple_pads(ecspi3_pads,
					 ARRAY_SIZE(ecspi3_pads));
}
#endif

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
    {
	   phydev->drv->config(phydev);
	   printf("phy: %s\n",phydev->drv->name );
	}  
	return 0;
}


int board_eth_init(bd_t *bis)
{
	uint32_t base = IMX_FEC_BASE;
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	int ret;

	setup_iomux_enet();

	bus = fec_get_miibus(base, -1);
	if (!bus)
	{
		printf("%s: fec_get_miibus failed\n", __func__);
	    return -ENOMEM;
    }
	/* check if phy registers are readable through RMII */
	phydev = phy_find_by_mask(bus, (0x0001 << CONFIG_FEC_MXC_PHYADDR), PHY_INTERFACE_MODE_RGMII);
	if (!phydev) {
		printf("%s: can not find micrel switch\n", __func__);
		free(bus);
		return -ENOMEM;
	}

	ret = fec_probe(bis, -1, base, bus, phydev);
	if (ret) {
		printf("FEC MXC: %s:failed\n", __func__);
		free(phydev);
		free(bus);
		return -ENOMEM;		
	}
    ret = eth_init(bis);
     
	return ret;
}


static void setup_buttons(void)
{
	// imx_iomux_v3_setup_multiple_pads(button_pads,
					 // ARRAY_SIZE(button_pads));
}

#if defined(CONFIG_VIDEO_IPUV3)

static iomux_v3_cfg_t const backlight_pads[] = {
	/* Backlight on LVDS connector */
    /* BL_EN */
	MX6_PAD_DISP0_DAT10__GPIO4_IO31		| MUX_PAD_CTRL(OUTPUT_40OHM),
#define LVDS_BACKLIGHT_GP IMX_GPIO_NR(4, 31)
    /* PWM */
	MX6_PAD_DISP0_DAT9__PWM2_OUT		| MUX_PAD_CTRL(WEAK_PULLUP)
#define LVDS_BLACKLIGHT_PWM  1
};

static int detect_i2c(struct display_info_t const *dev)
{
	return ((0 == i2c_set_bus_num(dev->bus))
		&&
		(0 == i2c_probe(dev->addr)));
}

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
	writel(reg, &iomux->gpr[2]);
	gpio_direction_output(LVDS_BACKLIGHT_GP, 1);
}


struct display_info_t const displays[] = {{

/*	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "LDB-WXGA-S",
		.refresh        = 60,
		.xres           = 1280,
		.yres           = 800,
		.pixclock       = 14065,
		.left_margin    = 40,
		.right_margin   = 40,
		.upper_margin   = 3,
		.lower_margin   = 80,
		.hsync_len      = 10,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED} }, {
	.bus	= 2,
	.addr	= 0x4,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_i2c,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "Hannstar-XGA",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, { */
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "LG-9.7",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385, /* ~65MHz */
		.left_margin    = 480,
		.right_margin   = 260,
		.upper_margin   = 16,
		.lower_margin   = 6,
		.hsync_len      = 250,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
/*} }, {
	.bus	= 2,
	.addr	= 0x38,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_i2c,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "wsvga-lvds",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 600,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 2,
	.addr	= 0x41,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_i2c,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "amp1024x600",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 600,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= 0,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "wvga-lvds",
		.refresh        = 57,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED*/
} } };
size_t display_count = ARRAY_SIZE(displays);

int board_cfb_skip(void)
{
	return NULL != getenv("novideo");
}

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	enable_ipu_clock();
	/* imx_setup_hdmi(); */
	/* Turn on LDB0,IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 |MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      |(3<<MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<<MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     |IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
	     |IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     |IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     |IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     |IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
	     |IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK
			|IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       <<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);

	/* backlights off until needed */
	imx_iomux_v3_setup_multiple_pads(backlight_pads,
					 ARRAY_SIZE(backlight_pads));
	gpio_direction_input(LVDS_BACKLIGHT_GP);
    
    // bl out for test
    gpio_direction_output(LVDS_BACKLIGHT_GP, 1);
    gpio_set_value(LVDS_BACKLIGHT_GP, 1);
    
    pwm_init(LVDS_BLACKLIGHT_PWM, 0, 0);
    pwm_config(LVDS_BLACKLIGHT_PWM, 25000, 50000);
    pwm_enable(LVDS_BLACKLIGHT_PWM);
}
#endif

static iomux_v3_cfg_t const init_pads[] = {
	/* SGTL5000 sys_mclk */
	//NEW_PAD_CTRL(MX6_PAD_GPIO_0__CCM_CLKO1, OUTPUT_40OHM),

	/* J5 - Camera MCLK */
	//NEW_PAD_CTRL(MX6_PAD_GPIO_3__CCM_CLKO2, OUTPUT_40OHM),

	/* wl1271 pads on nitrogen6x */
	/* WL12XX_WL_IRQ_GP */
	// NEW_PAD_CTRL(MX6_PAD_NANDF_CS1__GPIO6_IO14, WEAK_PULLDOWN),
	// /* WL12XX_WL_ENABLE_GP */
	// NEW_PAD_CTRL(MX6_PAD_NANDF_CS2__GPIO6_IO15, OUTPUT_40OHM),
	// /* WL12XX_BT_ENABLE_GP */
	// NEW_PAD_CTRL(MX6_PAD_NANDF_CS3__GPIO6_IO16, OUTPUT_40OHM),
	/* USB otg power */
	NEW_PAD_CTRL(MX6_PAD_EIM_D22__GPIO3_IO22, OUTPUT_40OHM),
	NEW_PAD_CTRL(MX6_PAD_NANDF_D5__GPIO2_IO05, OUTPUT_40OHM),
	NEW_PAD_CTRL(MX6_PAD_NANDF_WP_B__GPIO6_IO09, OUTPUT_40OHM),
	NEW_PAD_CTRL(MX6_PAD_GPIO_8__GPIO1_IO08, OUTPUT_40OHM),
	NEW_PAD_CTRL(MX6_PAD_GPIO_6__GPIO1_IO06, OUTPUT_40OHM),
};

#define WL12XX_WL_IRQ_GP	IMX_GPIO_NR(6, 14)

static unsigned gpios_out_low[] = {
	/* Disable wl1271 */
	IMX_GPIO_NR(6, 15),	/* disable wireless */
	IMX_GPIO_NR(6, 16),	/* disable bluetooth */
	IMX_GPIO_NR(3, 22),	/* disable USB otg power */
	IMX_GPIO_NR(2, 5),	/* ov5640 mipi camera reset */
	IMX_GPIO_NR(1, 8),	/* ov5642 reset */
};

static unsigned gpios_out_high[] = {
	IMX_GPIO_NR(1, 6),	/* ov5642 powerdown */
	IMX_GPIO_NR(6, 9),	/* ov5640 mipi camera power down */
};

static void set_gpios(unsigned *p, int cnt, int val)
{
	int i;

	for (i = 0; i < cnt; i++)
		gpio_direction_output(*p++, val);
}

int board_early_init_f(void)
{

      // debug !!
	setup_iomux_leds();

	gpio_direction_output(IMX_GPIO_NR(2, 0) , 0);
	gpio_direction_output(IMX_GPIO_NR(2, 5) , 0);

	gpio_set_value(IMX_GPIO_NR(2, 0), 1);

	setup_iomux_uart();

	//set_gpios(gpios_out_high, ARRAY_SIZE(gpios_out_high), 1);
	//set_gpios(gpios_out_low, ARRAY_SIZE(gpios_out_low), 0);
	//gpio_direction_input(WL12XX_WL_IRQ_GP);

	//imx_iomux_v3_setup_multiple_pads(wl12xx_pads, ARRAY_SIZE(wl12xx_pads));
	//imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));
	//setup_buttons();

#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif
	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

static int pfuze_init(void)
{
	struct pmic *p;
	int ret;
	unsigned int reg;

	ret = power_pfuze100_init(1);
	if (ret)
		return ret;

	p = pmic_get("PFUZE100");
	ret = pmic_probe(p);
	if (ret)
		return ret;

	pmic_reg_read(p, PFUZE100_DEVICEID, &reg);
	printf("PMIC:  PFUZE100 ID=0x%02x\n", reg);
	return 0;
}

int board_init(void)
{
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;

	clrsetbits_le32(&iomuxc_regs->gpr[1],
			IOMUXC_GPR1_OTG_ID_MASK,
			IOMUXC_GPR1_OTG_ID_GPIO1);

	imx_iomux_v3_setup_multiple_pads(misc_pads, ARRAY_SIZE(misc_pads));

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif
	// imx_iomux_v3_setup_multiple_pads(
		// usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
	//setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1); // PMIC
	//setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif

    pfuze_init();
    
    // last boot LED
	gpio_set_value(IMX_GPIO_NR(2, 5), 1);

	return 0;
}

int checkboard(void)
{
    puts("Board: PeterSime HMI\n");

	return 0;
}

struct button_key {
	char const	*name;
	unsigned	gpnum;
	char		ident;
};

static struct button_key const buttons[] = {
	{"back",	IMX_GPIO_NR(2, 2),	'B'},
	{"home",	IMX_GPIO_NR(2, 4),	'H'},
	{"menu",	IMX_GPIO_NR(2, 1),	'M'},
	{"search",	IMX_GPIO_NR(2, 3),	'S'},
	{"volup",	IMX_GPIO_NR(7, 13),	'V'},
	{"voldown",	IMX_GPIO_NR(4, 5),	'v'},
};

/*
 * generate a null-terminated string containing the buttons pressed
 * returns number of keys pressed
 */
static int read_keys(char *buf)
{
	int i, numpressed = 0;
	for (i = 0; i < ARRAY_SIZE(buttons); i++) {
		if (!gpio_get_value(buttons[i].gpnum))
			buf[numpressed++] = buttons[i].ident;
	}
	buf[numpressed] = '\0';
	return numpressed;
}

static int do_kbd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char envvalue[ARRAY_SIZE(buttons)+1];
	int numpressed = read_keys(envvalue);
	setenv("keybd", envvalue);
	return numpressed == 0;
}


U_BOOT_CMD(
	kbd, 1, 1, do_kbd,
	"Tests for keypresses, sets 'keybd' environment variable",
	"Returns 0 (true) to shell if key is pressed."
);
static int change_blacklight_pwm(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    uint32_t duty_cycle;
    uint32_t period;
    
    if (argc > 2)
    {
        duty_cycle = simple_strtoul(argv[1], NULL, 10);
        period = simple_strtoul(argv[2], NULL, 10);
    
        printf("change blacklight: \n    duty cycle: %u ns\n    period: %u ns\n", duty_cycle, period);
        pwm_disable(LVDS_BLACKLIGHT_PWM);
        pwm_config(LVDS_BLACKLIGHT_PWM, duty_cycle, period);
        pwm_enable(LVDS_BLACKLIGHT_PWM);
    }
    else
    {
        printf("change blacklight: too few arguments\n\n");
    }
    
    return 0;
}

static int change_blacklight_enable(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    uint32_t enable;

    if (argc > 1)
    {
        enable = (simple_strtoul(argv[1], NULL, 10) & 0x1);
        printf("set blacklight enable to: %u\n\n", enable);
        gpio_set_value(LVDS_BACKLIGHT_GP, enable);
    }
}

U_BOOT_CMD(
	blpwm, 3, 1, change_blacklight_pwm,
	"Configure the blacklight pwm: ",
	"blpwm duty_cycle_in_ns clock_period_in_ns"
);

U_BOOT_CMD(
	blen, 2, 1, change_blacklight_enable,
	"Enable/disable the blacklight pwm: ",
	"blen 1/0"
);

#ifdef CONFIG_PREBOOT
static char const kbd_magic_prefix[] = "key_magic";
static char const kbd_command_prefix[] = "key_cmd";

static void preboot_keys(void)
{
	int numpressed;
	char keypress[ARRAY_SIZE(buttons)+1];
	numpressed = read_keys(keypress);
	if (numpressed) {
		char *kbd_magic_keys = getenv("magic_keys");
		char *suffix;
		/*
		 * loop over all magic keys
		 */
		for (suffix = kbd_magic_keys; *suffix; ++suffix) {
			char *keys;
			char magic[sizeof(kbd_magic_prefix) + 1];
			sprintf(magic, "%s%c", kbd_magic_prefix, *suffix);
			keys = getenv(magic);
			if (keys) {
				if (!strcmp(keys, keypress))
					break;
			}
		}
		if (*suffix) {
			char cmd_name[sizeof(kbd_command_prefix) + 1];
			char *cmd;
			sprintf(cmd_name, "%s%c", kbd_command_prefix, *suffix);
			cmd = getenv(cmd_name);
			if (cmd) {
				setenv("preboot", cmd);
				return;
			}
		}
	}
}
#endif

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{"mmc1",	MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,		0},
};
#endif

int misc_init_r(void)
{
#ifdef CONFIG_PREBOOT
	preboot_keys();
#endif

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	return 0;
}
