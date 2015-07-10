/*
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <fsl_esdhc.h>
#include <malloc.h>
#include <miiphy.h>
#include <micrel.h>
#include <netdev.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/iomux.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/video.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <i2c.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "common.h"

DECLARE_GLOBAL_DATA_PTR;


#ifdef CONFIG_SYS_I2C_MXC
#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_LOW | \
			PAD_CTL_DSE_40ohm | PAD_CTL_HYS | \
			PAD_CTL_ODE | PAD_CTL_SRE_FAST)

			
/* I2C1, Eeprom */
static struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6_PAD_CSI0_DAT9__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_CSI0_DAT9__GPIO5_IO27 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 27)
	},
	.sda = {
		.i2c_mode = MX6_PAD_CSI0_DAT8__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_CSI0_DAT8__GPIO5_IO26 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 26)
	}
};

/* I2C2 PMIC */
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 13)
	}
};

/* I2C3 audio codec */
static struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_EIM_D17__I2C3_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_EIM_D17__GPIO3_IO17 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(3, 17)
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D18__I2C3_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_EIM_D18__GPIO3_IO18 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(3, 18)
	}
};	
		

static void verkerk_vipbox_spl_setup_i2c(void)
{
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0); // eeprom
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1); // PMIC
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2); // testpad
}
#else
static void verkerk_vipbox_spl_setup_i2c(void) { }
#endif

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	return 0;
}

int board_ehci_power(int port, int on)
{
	return 0;
}

#define OUTPUT_40OHM (PAD_CTL_SPEED_MED|PAD_CTL_DSE_40ohm)

static iomux_v3_cfg_t const usb_otg_pads[] = {
	MX6_PAD_GPIO_1__USB_OTG_ID		| MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_EIM_D21__USB_OTG_OC		| MUX_PAD_CTRL(WEAK_PULLUP),
	/* OTG Power enable */
	MX6_PAD_EIM_D22__GPIO3_IO22		| MUX_PAD_CTRL(WEAK_PULLDOWN)
};

static void setup_usb_otg(void)
{
/* initialise USB OTG ID line     */
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;

	clrsetbits_le32(&iomuxc_regs->gpr[1],IOMUXC_GPR1_OTG_ID_MASK,IOMUXC_GPR1_OTG_ID_GPIO1);

	imx_iomux_v3_setup_multiple_pads(usb_otg_pads, ARRAY_SIZE(usb_otg_pads));
}
#else
static void setup_usb_otg(void) {}
#endif

#ifdef CONFIG_FEC_MXC
#define ENET_PAD_CTRL		(PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_LOW | \
				 PAD_CTL_DSE_40ohm | PAD_CTL_HYS)
#define ENET_PAD_CTRL_PULLDOWN  (PAD_CTL_PUS_100K_DOWN |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

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
	
	/* not connected PHY reset */
	MX6_PAD_NANDF_CS0__GPIO6_IO11	| MUX_PAD_CTRL(NO_PAD_CTRL),
	
};

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
    
    /* Reset Micrel KSZ8081 PHY */
	gpio_direction_output(IMX_GPIO_NR(6, 11) , 0);
	udelay(500);
	gpio_set_value(IMX_GPIO_NR(6, 11), 1);
}

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
    int val;
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;

	setup_iomux_enet();
	    
	// ENKEL VOOR OMGEBOUWDE BOARDS
    char* enable_enet_clk_env = getenv("enable_enet_clk");
	if (NULL != enable_enet_clk_env && 0 == strcmp(enable_enet_clk_env, "1"))
    {
        printf("Enabling i.MX6 Ethernet clock\n");
        
        // enable enet clock gating (on for all power modes)
        enable_enet_clk(1);  
        
        // set PLL at 50 MHz
        enable_fec_anatop_clock(ENET_50MHz);
        
        // enable PLL (set gpr1[ENET_CLK_SEL])
        setbits_le32(&iomuxc_regs->gpr[1], IOMUXC_GPR1_ENET_CLK_SEL_MASK);   
    }
    
	bus = fec_get_miibus(base, -1);
	if (!bus)
	{
		printf("%s: fec_get_miibus failed\n", __func__);
	    return -ENOMEM;
    }
	/* check if phy registers are readable through RMII */
	phydev = phy_find_by_mask(bus, (0x0001 << CONFIG_FEC_MXC_PHYADDR), PHY_INTERFACE_MODE_RMII);
	if (!phydev) {
		printf("%s: can not find micrel phy\n", __func__);
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
    
	// ENKEL VOOR OMGEBOUWDE BOARDS
	if (NULL != enable_enet_clk_env && 0 == strcmp(enable_enet_clk_env, "1"))
    {     
        val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1f);
        printf("DBG: current val of reg 0x1f: 0x%x\n", val);
        phy_write(phydev, MDIO_DEVAD_NONE, 0x1f, val | 0x80); // enable bit 7        
        val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1f);
        printf("DBG: new val of reg 0x1f: 0x%x\n", val);        
    }
    
    ret = eth_init(bis);
	
	return ret;
}
#endif


#ifdef CONFIG_POWER_PFUZE100
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
#else
static int pfuze_init(void) {}
#endif


#ifdef CONFIG_MXC_SPI
#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_SLOW)

static iomux_v3_cfg_t const ecspi_pads[] = {
	MX6_PAD_KEY_ROW1__GPIO4_IO09  | MUX_PAD_CTRL(NO_PAD_CTRL), /* CS0 */
	MX6_PAD_KEY_COL2__GPIO4_IO10  | MUX_PAD_CTRL(NO_PAD_CTRL), /* CS1 */
	MX6_PAD_KEY_COL1__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_KEY_ROW0__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_KEY_COL0__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

void verkerk_vipbox_spl_setup_spi(void)
{
	SETUP_IOMUX_PADS(ecspi_pads);
}

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 1) ? (IMX_GPIO_NR(4, 9)) : -1;
}
#else
void verkerk_vipbox_spl_setup_spi(void) {}
#endif

#define LED_GREEN 0
#define LED_RED 1

void enable_user_led(int number, int on_off)
{
    imx_iomux_v3_setup_pad(MX6_PAD_NANDF_D6__GPIO2_IO06 | NO_PAD_CTRL);  
    imx_iomux_v3_setup_pad(MX6_PAD_NANDF_D7__GPIO2_IO07 | NO_PAD_CTRL);

    switch(number)
    {
        case 0: 
            gpio_direction_output(IMX_GPIO_NR(2, 6), on_off);         
            break;
        case 1:
            gpio_direction_output(IMX_GPIO_NR(2, 7), on_off);
            break;
    }  
}

static iomux_v3_cfg_t const gpio_pads[] = {
    // SAN-RF GPIO
        MX6_PAD_EIM_D19__GPIO3_IO19      | MUX_PAD_CTRL(NO_PAD_CTRL),
        MX6_PAD_EIM_D20__GPIO3_IO20      | MUX_PAD_CTRL(NO_PAD_CTRL),
        
    // DIN
        MX6_PAD_DISP0_DAT18__GPIO5_IO12  | MUX_PAD_CTRL(NO_PAD_CTRL),
        MX6_PAD_DISP0_DAT19__GPIO5_IO13  | MUX_PAD_CTRL(NO_PAD_CTRL),
        MX6_PAD_DISP0_DAT20__GPIO5_IO14  | MUX_PAD_CTRL(NO_PAD_CTRL),
        MX6_PAD_DISP0_DAT6__GPIO4_IO27  | MUX_PAD_CTRL(NO_PAD_CTRL),
        MX6_PAD_DISP0_DAT22__GPIO5_IO16  | MUX_PAD_CTRL(NO_PAD_CTRL),
        MX6_PAD_DISP0_DAT23__GPIO5_IO17  | MUX_PAD_CTRL(NO_PAD_CTRL),
        
    // DOUT
        MX6_PAD_DISP0_DAT14__GPIO5_IO08  | MUX_PAD_CTRL(NO_PAD_CTRL),
        MX6_PAD_DISP0_DAT15__GPIO5_IO09  | MUX_PAD_CTRL(NO_PAD_CTRL),
        MX6_PAD_DISP0_DAT16__GPIO5_IO10  | MUX_PAD_CTRL(NO_PAD_CTRL),
        MX6_PAD_DISP0_DAT17__GPIO5_IO11  | MUX_PAD_CTRL(NO_PAD_CTRL),
        
    // RELAIS
        MX6_PAD_DISP0_DAT13__GPIO5_IO07  | MUX_PAD_CTRL(NO_PAD_CTRL),
        
    // AUDIO REN & TEN
        MX6_PAD_DISP0_DAT11__GPIO5_IO05  | MUX_PAD_CTRL(NO_PAD_CTRL),
        MX6_PAD_DISP0_DAT12__GPIO5_IO06  | MUX_PAD_CTRL(NO_PAD_CTRL),
};

void setup_gpio_pads(void)
{
    SETUP_IOMUX_PADS(gpio_pads);
}

#define AUD_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_PKE | PAD_CTL_SPEED_LOW |		\
                        PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define IOMUXC_AUD4_INPUT_DA_AMX_SELECT_INPUT        0x20e0798
#define IOMUXC_AUD4_INPUT_DB_AMX_SELECT_INPUT        0x20e079C
#define IOMUXC_AUD4_INPUT_TXCLK_AMX_SELECT_INPUT     0x20e07A8
#define IOMUXC_AUD4_INPUT_TXFS_AMX_SELECT_INPUT      0x20e07AC                  
                        
static iomux_v3_cfg_t const audmux_pads[] = {
        MX6_PAD_SD2_DAT0__AUD4_RXD      | MUX_PAD_CTRL(AUD_PAD_CTRL | PAD_CTL_PUS_100K_DOWN),
        MX6_PAD_SD2_DAT1__AUD4_TXFS     | MUX_PAD_CTRL(AUD_PAD_CTRL | PAD_CTL_PUS_100K_DOWN),
        MX6_PAD_SD2_DAT2__AUD4_TXD      | MUX_PAD_CTRL(AUD_PAD_CTRL),
        MX6_PAD_SD2_DAT3__AUD4_TXC      | MUX_PAD_CTRL(AUD_PAD_CTRL | PAD_CTL_PUS_100K_DOWN),
};

void setup_audmux_pads(void)
{    
    SETUP_IOMUX_PADS(audmux_pads);
    
    // Daisy chain registers to select the SD2 as AUDMUX I/O
    setbits_le32(IOMUXC_AUD4_INPUT_DA_AMX_SELECT_INPUT, 1);  
    setbits_le32(IOMUXC_AUD4_INPUT_DB_AMX_SELECT_INPUT, 1);  
    setbits_le32(IOMUXC_AUD4_INPUT_TXCLK_AMX_SELECT_INPUT, 1);  
    setbits_le32(IOMUXC_AUD4_INPUT_TXFS_AMX_SELECT_INPUT, 1);   
}

int board_init(void)
{	
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	
	verkerk_vipbox_spl_setup_spi();

	verkerk_vipbox_spl_setup_i2c();
	
	setup_usb_otg();
	
	pfuze_init();	
    
    setup_gpio_pads();
    
    setup_audmux_pads();
	
	enable_user_led(LED_GREEN, 1);
	//enable_user_led(LED_RED, 1);
	
	return 0;
}

int checkboard(void)
{
    puts("Board: Verkerk Vipbox-III (with SPL)\n");

	return 0;
}

int dram_init(void)
{
	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);

	return 0;
}
