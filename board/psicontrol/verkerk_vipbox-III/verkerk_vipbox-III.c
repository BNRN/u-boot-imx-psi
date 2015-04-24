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

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

int dram_init(void)
{
	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);

	return 0;
}

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
/*static iomux_v3_cfg_t const uart3_pads[] = {
	MX6_PAD_EIM_D24__UART3_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D25__UART3_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};*/

/* I2C1, Eeprom */
static struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6_PAD_CSI0_DAT9__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_CSI0_DAT9__GPIO5_IO27 | PC,
		.gp = IMX_GPIO_NR(5, 27)
	},
	.sda = {
		.i2c_mode = MX6_PAD_CSI0_DAT8__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_CSI0_DAT8__GPIO5_IO26 | PC,
		.gp = IMX_GPIO_NR(5, 26)
	}
};

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

/* I2C3 testpad */
static struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_EIM_D17__I2C3_SCL | PC,
		.gpio_mode = MX6_PAD_EIM_D17__GPIO3_IO17 | PC,
		.gp = IMX_GPIO_NR(3, 17)
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D18__I2C3_SDA | PC,
		.gpio_mode = MX6_PAD_EIM_D18__GPIO3_IO18 | PC,
		.gp = IMX_GPIO_NR(3, 18)
	}
};

static iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT6__GPIO6_IO18    | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};

/*static iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};*/



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

static void enable_led(unsigned number, unsigned on_off)
{
    iomux_v3_cfg_t config = (on_off) ? MUX_PAD_CTRL(WEAK_PULLDOWN) : MUX_PAD_CTRL(WEAK_PULLUP);

    switch(number)
    {
        case 0:
            imx_iomux_v3_setup_pad(MX6_PAD_NANDF_D0__GPIO2_IO00 | config);
            gpio_direction_output(IMX_GPIO_NR(2, 0), on_off);
            break;
        case 1:
            imx_iomux_v3_setup_pad(MX6_PAD_NANDF_D5__GPIO2_IO05 | config);
            gpio_direction_output(IMX_GPIO_NR(2, 5), on_off);
            break;
    }   
    
    /* todo change with new boards */
    
}

static iomux_v3_cfg_t const led_pads[] = {
	MX6_PAD_NANDF_D0__GPIO2_IO00 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_NANDF_D5__GPIO2_IO05 | MUX_PAD_CTRL(WEAK_PULLUP),
	
};

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
	}

	return ret;
}


int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	u32 index = 0;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);

	usdhc_cfg[0].max_bus_width = 4;

	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			gpio_direction_input(USDHC3_CD_GPIO);	
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
	enable_enet_clk(1);

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

static iomux_v3_cfg_t const init_pads[] = {
	/* USB otg power */
	NEW_PAD_CTRL(MX6_PAD_EIM_D22__GPIO3_IO22, OUTPUT_40OHM),
	NEW_PAD_CTRL(MX6_PAD_NANDF_D5__GPIO2_IO05, OUTPUT_40OHM),
	NEW_PAD_CTRL(MX6_PAD_NANDF_WP_B__GPIO6_IO09, OUTPUT_40OHM),
	NEW_PAD_CTRL(MX6_PAD_GPIO_8__GPIO1_IO08, OUTPUT_40OHM),
	NEW_PAD_CTRL(MX6_PAD_GPIO_6__GPIO1_IO06, OUTPUT_40OHM),
};

int board_early_init_f(void)
{
    // LEDS on
    enable_led(0, 1);
    enable_led(1, 1);
    
    // uart
	setup_iomux_uart();

    // blink LEDs
    enable_led(0, 0);
    enable_led(1, 0);

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

	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0); // Eeprom
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1); // PMIC
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2); // testpad


    pfuze_init();
        
    // done booting
    
	enable_led(0, 0);

	return 0;
}

int checkboard(void)
{
    puts("Board: Verkerk Vipbox-III\n");

	return 0;
}

int misc_init_r(void)
{
	return 0;
}

#ifdef CONFIG_SHOW_BOOT_PROGRESS
void show_boot_progress(int progress)
{
    static int on_off = 0;
    on_off = !on_off;

	if (progress > 0)
		return;

	enable_led(0, on_off);
	enable_led(1, 0);
}
#endif